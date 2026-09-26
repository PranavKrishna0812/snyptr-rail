#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>

// DEVKIT V1 WILL CREATE ITS OWN HOTSPOT (AND BROADCAST ESP-NOW TO SERVO TARGET)
const char* ssid = "ESP32_Camera";
const char* password = "password123";

WebServer server(80);
uint8_t espNowBroadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
volatile unsigned long latest_hit_time = 0;
volatile int latest_hit_pixels = 0;
String latest_hit_zone = "OPTIMAL";

// Hardware Serial 2 on DevKit V1 (Safe general-purpose G21 pin)
#define UART_BAUD 115200
#define RX_PIN 21
#define TX_PIN 17

const char* STREAM_BOUNDARY = "123456789000000000000987654321";

// Frame buffers
uint8_t *rx_buf = NULL;
uint8_t *latest_buf = NULL;
uint8_t *tx_buf = NULL;

uint32_t max_jpeg_size = 92160; // 90 KB max JPEG size
bool use_triple_buffering = false;

// Shared volatile variables protected by frame_mutex
volatile uint32_t latest_len = 0;
volatile uint32_t latest_frame_id = 0;
volatile bool latest_new = false;
volatile bool tx_in_progress = false; // Sync lock for double buffering fallback
volatile bool client_connected = false;

volatile uint32_t tx_len = 0;
volatile uint32_t tx_frame_id = 0;

// Diagnostics counters
volatile uint32_t packet_received_count = 0;
volatile uint32_t crc_ok_count = 0;
volatile uint32_t crc_error_count = 0;
volatile uint32_t resync_count = 0;
volatile uint32_t jpeg_too_large_count = 0;
volatile uint32_t dropped_frames_count = 0;
volatile uint32_t latest_frame_size = 0;

SemaphoreHandle_t frame_mutex = NULL;
TaskHandle_t uart_rx_task_handle = NULL;

// Calculate CRC32 (Standard IEEE 802.3 ethernet representation)
uint32_t calculate_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// Background task to continuously read and parse binary packet protocol from UART
void uart_rx_task(void *arg) {
    enum ParseState {
        STATE_SYNC1,
        STATE_SYNC2,
        STATE_HEADER,
        STATE_PAYLOAD
    };

    ParseState state = STATE_SYNC1;
    uint32_t header_bytes_read = 0;
    uint32_t payload_bytes_read = 0;
    uint8_t hdr_buf[13]; // 13 bytes remaining in header after 0xAA 0x55

    uint8_t version = 0;
    uint32_t rx_frame_id = 0;
    uint32_t rx_payload_len = 0;
    uint32_t rx_crc32 = 0;

    Serial.println("UART RX Task: Started parser loop.");

    while (true) {
        int avail = Serial2.available();
        if (avail <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Yield
            continue;
        }

        // Process available bytes
        for (int i = 0; i < avail; i++) {
            uint8_t b = Serial2.read();
            
            switch (state) {
                case STATE_SYNC1:
                    if (b == 0xAA) {
                        state = STATE_SYNC2;
                    }
                    break;

                case STATE_SYNC2:
                    if (b == 0x55) {
                        state = STATE_HEADER;
                        header_bytes_read = 0;
                    } else if (b == 0xAA) {
                        state = STATE_SYNC2; // Stay in SYNC2
                    } else {
                        state = STATE_SYNC1;
                    }
                    break;

                case STATE_HEADER:
                    hdr_buf[header_bytes_read++] = b;
                    if (header_bytes_read == 13) {
                        version = hdr_buf[0];
                        rx_frame_id = hdr_buf[1] | (hdr_buf[2] << 8) | (hdr_buf[3] << 16) | (hdr_buf[4] << 24);
                        rx_payload_len = hdr_buf[5] | (hdr_buf[6] << 8) | (hdr_buf[7] << 16) | (hdr_buf[8] << 24);
                        rx_crc32 = hdr_buf[9] | (hdr_buf[10] << 8) | (hdr_buf[11] << 16) | (hdr_buf[12] << 24);

                        if (version != 0x01) {
                            Serial.printf("PROTOCOL ERROR: Invalid version %d\n", version);
                            resync_count++;
                            state = STATE_SYNC1;
                            break;
                        }

                        if (rx_payload_len > max_jpeg_size) {
                            jpeg_too_large_count++;
                            Serial.printf("\nJPEG TOO LARGE | size: %u | maximum: %u\n\n", rx_payload_len, max_jpeg_size);
                            resync_count++;
                            state = STATE_SYNC1;
                            break;
                        }

                        payload_bytes_read = 0;
                        state = STATE_PAYLOAD;
                    }
                    break;

                case STATE_PAYLOAD:
                    rx_buf[payload_bytes_read++] = b;
                    if (payload_bytes_read == rx_payload_len) {
                        uint32_t calc_crc = calculate_crc32(rx_buf, rx_payload_len);
                        if (calc_crc == rx_crc32) {
                            crc_ok_count++;
                            
                            // Check if it is a UART diagnostic test packet
                            if (rx_payload_len == 23 && memcmp(rx_buf, "UART_TEST_PACKET_123456", 23) == 0) {
                                Serial.printf("\nUART TEST RECEIVED\nCRC OK\nPAYLOAD LENGTH: %u\n\n", rx_payload_len);
                            } else {
                                // Swap buffers safely under mutex lock
                                xSemaphoreTake(frame_mutex, portMAX_DELAY);
                                if (use_triple_buffering) {
                                    uint8_t *temp = rx_buf;
                                    rx_buf = latest_buf;
                                    latest_buf = temp;
                                    latest_len = rx_payload_len;
                                    latest_frame_id = rx_frame_id;
                                    latest_new = true;
                                } else {
                                    // Double buffering fallback
                                    if (!tx_in_progress) {
                                        uint8_t *temp = rx_buf;
                                        rx_buf = latest_buf;
                                        latest_buf = temp;
                                        latest_len = rx_payload_len;
                                        latest_frame_id = rx_frame_id;
                                        latest_new = true;
                                    } else {
                                        dropped_frames_count++;
                                    }
                                }
                                xSemaphoreGive(frame_mutex);
                            }
                            packet_received_count++;
                            latest_frame_size = rx_payload_len;
                        } else {
                            crc_error_count++;
                            Serial.printf("CRC ERROR: Expected 0x%08X, calculated 0x%08X\n", rx_crc32, calc_crc);
                        }
                        state = STATE_SYNC1; // Search for next frame sync
                    }
                    break;
            }
        }
    }
}

void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=";
  response += STREAM_BOUNDARY;
  response += "\r\n\r\n";
  client.print(response);

  Serial.println("Browser connected! Starting stream...");
  client_connected = true;

  uint32_t last_sent_frame_id = 0;

  while (client.connected()) {
    bool has_new_frame = false;
    uint32_t current_len = 0;
    uint8_t *current_ptr = NULL;

    xSemaphoreTake(frame_mutex, portMAX_DELAY);
    if (latest_len > 0 && latest_frame_id != last_sent_frame_id) {
        if (use_triple_buffering) {
            uint8_t *temp = tx_buf;
            tx_buf = latest_buf;
            latest_buf = temp;
            tx_len = latest_len;
            tx_frame_id = latest_frame_id;
            latest_new = false;
            
            current_len = tx_len;
            current_ptr = tx_buf;
            last_sent_frame_id = tx_frame_id;
            has_new_frame = true;
        } else {
            // Double buffering fallback
            tx_in_progress = true;
            current_len = latest_len;
            current_ptr = latest_buf;
            last_sent_frame_id = latest_frame_id;
            latest_new = false;
            has_new_frame = true;
        }
    }
    xSemaphoreGive(frame_mutex);

    if (has_new_frame && current_ptr != NULL && current_len > 0) {
        String header = "--";
        header += STREAM_BOUNDARY;
        header += "\r\nContent-Type: image/jpeg\r\nContent-Length: ";
        header += String(current_len);
        header += "\r\n\r\n";
        client.print(header);

        // Stream JPEG payload
        uint32_t written = 0;
        while (written < current_len && client.connected()) {
            size_t w = client.write(current_ptr + written, current_len - written);
            if (w == 0) {
                delay(1);
            } else {
                written += w;
            }
        }
        
        client.print("\r\n");

        if (!use_triple_buffering) {
            // Release double buffering lock
            xSemaphoreTake(frame_mutex, portMAX_DELAY);
            tx_in_progress = false;
            xSemaphoreGive(frame_mutex);
        }
    } else {
        delay(10); // Yield if no new frame
    }
  }

  Serial.println("Browser disconnected.");
  client_connected = false;
  
  if (!use_triple_buffering) {
      xSemaphoreTake(frame_mutex, portMAX_DELAY);
      tx_in_progress = false;
      xSemaphoreGive(frame_mutex);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n--- DevKit Booting ---");
  size_t free_heap = ESP.getFreeHeap();
  Serial.printf("Free heap before buffer allocation: %d bytes\n", free_heap);

  // Attempt triple buffering (3x90KB)
  size_t buf_size = 92160; 
  rx_buf = (uint8_t*)malloc(buf_size);
  latest_buf = (uint8_t*)malloc(buf_size);
  tx_buf = (uint8_t*)malloc(buf_size);

  if (rx_buf && latest_buf && tx_buf) {
      Serial.println("Allocated triple buffer (3 x 90 KB). Running in triple-buffering mode.");
      max_jpeg_size = buf_size;
      use_triple_buffering = true;
  } else {
      // Clean up failed allocations
      if (rx_buf) free(rx_buf);
      if (latest_buf) free(latest_buf);
      if (tx_buf) free(tx_buf);
      rx_buf = latest_buf = tx_buf = NULL;

      // Fallback to double buffering (2x90KB)
      rx_buf = (uint8_t*)malloc(buf_size);
      latest_buf = (uint8_t*)malloc(buf_size);
      if (rx_buf && latest_buf) {
          Serial.println("Allocated double buffer (2 x 90 KB). Running in double-buffering fallback mode.");
          max_jpeg_size = buf_size;
          use_triple_buffering = false;
      } else {
          // Fallback to double buffering with smaller size (2x64KB)
          if (rx_buf) free(rx_buf);
          if (latest_buf) free(latest_buf);
          buf_size = 65536; 
          rx_buf = (uint8_t*)malloc(buf_size);
          latest_buf = (uint8_t*)malloc(buf_size);
          if (rx_buf && latest_buf) {
              Serial.println("Allocated double buffer (2 x 64 KB). Running in double-buffering fallback mode.");
              max_jpeg_size = buf_size;
              use_triple_buffering = false;
          } else {
              Serial.println("FATAL ERROR: Failed to allocate frame buffers!");
              while(true) { delay(1000); }
          }
      }
  }
  
  Serial.printf("Free heap after buffer allocation: %d bytes\n", ESP.getFreeHeap());

  frame_mutex = xSemaphoreCreateMutex();
  if (frame_mutex == NULL) {
      Serial.println("FATAL ERROR: Failed to create mutex!");
      while(true) { delay(1000); }
  }

  Serial.println("Starting Wi-Fi Hotspot + ESP-NOW...");
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  
  Serial.println("\n===========================");
  Serial.println("WiFi AP started");
  Serial.print("IP: ");
  Serial.println(IP);
  Serial.println("===========================");

  // Initialize ESP-NOW Broadcast Peer to transmit commands to the Servo ESP32
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, espNowBroadcastMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    Serial.println("ESP-NOW Broadcast Transmitter: ACTIVE");
  } else {
    Serial.println("ESP-NOW Init Failed!");
  }

  // Set Rx buffer size to 64KB for safety and start UART2
  Serial2.setRxBufferSize(65536); 
  Serial2.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // Start background task to parse UART on Core 1 (prioritized)
  xTaskCreatePinnedToCore(
      uart_rx_task,
      "uart_rx_task",
      8192,
      NULL,
      configMAX_PRIORITIES - 1,
      &uart_rx_task_handle,
      1
  );

  server.on("/", HTTP_GET, handleStream);
  server.on("/stream", HTTP_GET, handleStream);

  // Wireless Servo Control Endpoint: /cmd?action=UP,1 or /cmd?action=DOWN,1
  server.on("/cmd", HTTP_ANY, []() {
    String action = server.hasArg("action") ? server.arg("action") : "";
    action.trim();
    if (action.length() > 0) {
      esp_now_send(espNowBroadcastMac, (const uint8_t*)action.c_str(), action.length());
      Serial.printf("[WiFi -> ESP-NOW] Sent Servo Command: %s\n", action.c_str());
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/plain", "OK:" + action);
    } else {
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(400, "text/plain", "ERR:MISSING_ACTION");
    }
  });

  // Wireless Hit Notification Endpoint: immediately drops target servo via ESP-NOW
  server.on("/hit", HTTP_ANY, []() {
    String px = server.hasArg("px") ? server.arg("px") : "500";
    String dropCmd = "DOWN,1";
    esp_now_send(espNowBroadcastMac, (const uint8_t*)dropCmd.c_str(), dropCmd.length());
    latest_hit_time = millis();
    latest_hit_pixels = px.toInt();
    Serial.printf("[HIT DETECTED] %d px -> Sent immediate DOWN,1 via ESP-NOW!\n", latest_hit_pixels);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"status\":\"DROPPED\",\"cmd\":\"DOWN,1\"}");
  });

  // Wireless Telemetry Polling Endpoint for Laptop Dashboard
  server.on("/telemetry", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    unsigned long now = millis();
    bool activeHit = (latest_hit_time > 0 && (now - latest_hit_time < 3000));
    String json = "{";
    json += "\"hit\":" + String(activeHit ? "true" : "false") + ",";
    json += "\"dartPixels\":" + String(latest_hit_pixels) + ",";
    json += "\"zone\":\"" + latest_hit_zone + "\",";
    json += "\"uptimeMs\":" + String(now);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  server.handleClient();

  // Print diagnostics every 2 seconds
  static unsigned long last_diag_time = 0;
  unsigned long now = millis();
  if (now - last_diag_time >= 2000) {
      Serial.println("\n--- DEVKIT DIAGNOSTICS ---");
      Serial.printf("WiFi: OK\n");
      Serial.print("IP: ");
      Serial.println(WiFi.softAPIP());
      Serial.printf("UART baud: %d\n", UART_BAUD);
      Serial.printf("Packets received: %lu\n", packet_received_count);
      Serial.printf("CRC OK: %lu\n", crc_ok_count);
      Serial.printf("CRC errors: %lu\n", crc_error_count);
      Serial.printf("Resync events: %lu\n", resync_count);
      Serial.printf("JPEG Too Large: %lu\n", jpeg_too_large_count);
      if (!use_triple_buffering) {
          Serial.printf("Dropped frames: %lu\n", dropped_frames_count);
      }
      Serial.printf("Latest JPEG: %u KB\n", latest_frame_size / 1024);
      Serial.printf("Streaming client: %s\n", client_connected ? "YES" : "NO");
      Serial.println("--------------------------\n");

      last_diag_time = now;
  }
  
  delay(1);
}

    