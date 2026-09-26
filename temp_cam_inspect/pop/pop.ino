#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================================
// BUILT-IN PCA9685 I2C SERVO DRIVER (Zero External Library Dependencies!)
// Replaces <Adafruit_PWMServoDriver.h> using standard ESP32 <Wire.h>
// ============================================================================
struct BuiltinPCA9685 {
  uint8_t _i2caddr;
  BuiltinPCA9685(uint8_t addr = 0x40) : _i2caddr(addr) {}

  void write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
  }

  void begin() {
    write8(0x00, 0x00); // MODE1 reset
    delay(10);
  }

  void setPWMFreq(float freq) {
    // PCA9685 25MHz internal oscillator -> prescale = round(25000000 / (4096 * freq)) - 1
    float prescaleval = 25000000.0f / (4096.0f * freq) - 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);
    uint8_t oldmode = read8(0x00);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Sleep bit to set prescale
    write8(0x00, newmode);
    write8(0xFE, prescale);                    // PRESCALE register (0xFE)
    write8(0x00, oldmode);
    delay(5);
    write8(0x00, oldmode | 0xA0);              // Restart + Auto-Increment (AI) enabled
  }

  void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(0x06 + 4 * channel);            // LED0_ON_L + 4 * channel
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
  }
};

// ============================================================================
// POP-UP TARGET ESP32 FIRMWARE (BATTERY + WIRED TO ESP32-P4 + WIFI TO LAPTOP)
// 1. Wired directly to ESP32-P4 Camera Board via UART (115200 baud)
// 2. Controls PCA9685 Servo Driver over I2C (SDA = 21, SCL = 22)
// 3. Hosts Wi-Fi Hotspot ("ESP32_Camera" / "password123" @ 192.168.4.1)
//    to send HIT / NOT HIT signals to the Laptop Dashboard and receive UP/DOWN
// ============================================================================

const char* WIFI_SSID = "ESP32_Camera";
const char* WIFI_PASS = "password123";

WebServer server(80);
BuiltinPCA9685 pwm(0x40);

// UART2 pins for direct wire from ESP32-P4 (GPIO 21 TX on P4 -> GPIO 16 RX2 on Pop ESP32)
// Note: Also listens on Serial (RX0 / GPIO 3) simultaneously!
#define P4_UART_RX_PIN 16
#define P4_UART_TX_PIN 17

const int SERVO_DOWN = 350;
const int SERVO_UP   = 150;

int currentPos[7] = {350, 350, 350, 350, 350, 350, 350};
String targetStateStr = "DOWN";

// Live Hit Telemetry State (sent to Laptop Dashboard over Wi-Fi)
volatile bool g_hitActive = false;
volatile unsigned long g_hitTimestampMs = 0;
volatile unsigned long g_targetUpTimestampMs = 0;
volatile int g_hitX = 0;
volatile int g_hitY = 0;
volatile int g_hitPixels = 0;
volatile int g_lastIgnoredX = -1;
volatile int g_lastIgnoredY = -1;
volatile unsigned long g_lastP4StatMs = 0;
volatile float g_cvDarkPct = 0;
volatile int g_cvYellowPx = 0;
volatile int g_cvOrangePx = 0;

volatile bool pendingWirelessCmd = false;
String wirelessCmdString = "";

void moveServoSmooth(int channel, int targetPos, int stepDelay = 3) {
  int startPos = currentPos[channel];
  if (startPos == targetPos) return;

  int step = (targetPos > startPos) ? 2 : -2;
  
  if (step > 0) {
    for (int p = startPos; p <= targetPos; p += step) {
      pwm.setPWM(channel, 0, p);
      delay(stepDelay);
    }
  } else {
    for (int p = startPos; p >= targetPos; p += step) {
      pwm.setPWM(channel, 0, p);
      delay(stepDelay);
    }
  }
  pwm.setPWM(channel, 0, targetPos);
  currentPos[channel] = targetPos;
}

void executeCommand(String line, bool fromP4 = false) {
  line.trim();
  if (line.length() == 0) return;

  // If P4 sends raw "DOWN,1" before "HIT,1,...", ignore raw "DOWN,1" on Serial2
  // because validated "HIT,1,..." below immediately drops the servo itself!
  if (fromP4 && line.startsWith("DOWN,")) {
    return;
  }

  // 1. Direct HIT telemetry from wired ESP32-P4: "HIT,1,x,y,pixels"
  if (line.startsWith("HIT,")) {
    // Parse HIT,1,x,y,pixels
    int parsedX = 400, parsedY = 400, parsedPixels = 0;
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    if (c2 > 0 && c3 > 0 && c4 > 0) {
      parsedX = line.substring(c2 + 1, c3).toInt();
      parsedY = line.substring(c3 + 1, c4).toInt();
      parsedPixels = line.substring(c4 + 1).toInt();
    }

    // Ignore transients if target is already DOWN or still in initial 750ms servo swing
    unsigned long now = millis();
    if (targetStateStr != "UP" || (now - g_targetUpTimestampMs) < 750) {
      return;
    }

    // GENUINE 4-STAGE BULLET IMPACT CONFIRMED BY ESP32-P4 WHILE TARGET IS UP!
    g_hitX = parsedX;
    g_hitY = parsedY;
    g_hitPixels = parsedPixels;
    g_hitActive = true;
    g_hitTimestampMs = now;

    // Immediately drop Target 1 servo!
    moveServoSmooth(0, SERVO_DOWN, 1);
    targetStateStr = "DOWN";

    Serial.printf("[P4 -> POP] GENUINE HIT (%d px at %d,%d) -> TARGET DROPPED IMMEDIATELY!\n",
                  g_hitPixels, g_hitX, g_hitY);
    return;
  }

  // 2. Live CV stats from P4: "STAT,darkPct,yellowPx,orangePx,cx,cy"
  if (line.startsWith("STAT,")) {
    g_lastP4StatMs = millis();
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 > 0 && c2 > 0 && c3 > 0) {
      g_cvDarkPct = line.substring(c1 + 1, c2).toFloat();
      g_cvYellowPx = line.substring(c2 + 1, c3).toInt();
      int c4 = line.indexOf(',', c3 + 1);
      if (c4 > 0) {
        g_cvOrangePx = line.substring(c3 + 1, c4).toInt();
      } else {
        g_cvOrangePx = line.substring(c3 + 1).toInt();
      }
    }
    return;
  }

  // 3. 3-second Auto-Clear from wired ESP32-P4: "CLEAR,1"
  if (line.startsWith("CLEAR")) {
    g_hitActive = false;
    return;
  }

  // 3. Raise Target: "UP,1"
  if (line.startsWith("UP,")) {
    String idStr = line.substring(3);
    int targetId = idStr.toInt(); // 1 to 7
    if (targetId >= 1 && targetId <= 7) {
      int channel = targetId - 1;
      g_hitActive = false;
      moveServoSmooth(channel, SERVO_UP, 3);
      targetStateStr = "UP";
      g_targetUpTimestampMs = millis();
      // Command ESP32-P4 over TX2 (GPIO 17 -> P4 GPIO 22) to re-calibrate upright baseline!
      Serial2.println("ARM,1");
      Serial.print("UP_CONFIRMED,");
      Serial.println(targetId);
    }
  }
  // 4. Lower Target: "DOWN,1" or "DOWN,ALL" (from Wi-Fi Dashboard / USB / ESP-NOW)
  else if (line.startsWith("DOWN,")) {
    String idStr = line.substring(5);
    if (idStr == "ALL") {
      for (int channel = 0; channel < 7; channel++) {
        moveServoSmooth(channel, SERVO_DOWN, 1);
      }
      targetStateStr = "DOWN";
      Serial.println("DOWN_CONFIRMED,ALL");
    } else {
      int targetId = idStr.toInt(); // 1 to 7
      if (targetId >= 1 && targetId <= 7) {
        int channel = targetId - 1;
        moveServoSmooth(channel, SERVO_DOWN, 1); // Fast immediate drop!
        targetStateStr = "DOWN";
        Serial.print("DOWN_CONFIRMED,");
        Serial.println(targetId);
      }
    }
  }
  else if (line == "PING") {
    Serial.println("OK: PING_ACK (POP ESP32 + P4 BRIDGE READY)");
  }
  else if (line == "STATUS") {
    Serial.println("STATUS: READY");
  }
}

// ESP-NOW Wireless Callback (if additional wireless nodes are used)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  char buf[64];
  int copyLen = (len < 63) ? len : 63;
  memcpy(buf, data, copyLen);
  buf[copyLen] = '\0';
  wirelessCmdString = String(buf);
  pendingWirelessCmd = true;
}

void sendCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
}

void handleOptions() {
  sendCorsHeaders();
  server.send(204);
}

void setup() {
  // 0. CRITICAL FOR 4.04V BATTERY INPUT:
  //    At 4.04V on VIN, the onboard AMS1117 regulator outputs ~2.85V (4.04V - 1.15V dropout),
  //    which trips the default 2.80V brownout detector when Wi-Fi spikes to 350mA!
  //    1) Disable Brownout Reset immediately
  //    2) Drop CPU clock from 240MHz -> 80MHz (cuts baseline current from 68mA -> 22mA!)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  setCpuFrequencyMhz(80);

  // Onboard Blue LED (GPIO 2) -> SOLID ON when ESP32_Camera Wi-Fi is broadcasting
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // 1. Initialize UART0 (USB Serial & RX0 pin 3) at 115200 baud
  Serial.begin(115200);

  // 2. Initialize UART2 (RX2 = GPIO 16, TX2 = GPIO 17) at 115200 baud for ESP32-P4 wire
  Serial2.begin(115200, SERIAL_8N1, P4_UART_RX_PIN, P4_UART_TX_PIN);

  // 3. Start Wi-Fi Access Point FIRST ("ESP32_Camera" @ 192.168.4.1) with Low-Current RF mode!
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 4); // Channel 1, SSID visible (0), max 4 connections
  WiFi.setTxPower(WIFI_POWER_8_5dBm);         // Cuts RF TX current spike from 350mA -> 120mA for 4.04V battery!
  digitalWrite(2, HIGH);                      // Blue LED SOLID ON = Wi-Fi Hotspot is LIVE!
  IPAddress ip = WiFi.softAPIP();
  Serial.print("Wi-Fi Hotspot Active! Connect Laptop to '");
  Serial.print(WIFI_SSID);
  Serial.print("' -> IP: ");
  Serial.println(ip);

  // 4. Initialize ESP-NOW
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
  }

  // 5. Initialize PCA9685 Servo Driver on I2C (SDA = 21, SCL = 22) with non-blocking timeout
  Wire.begin(21, 22);
  Wire.setTimeOut(50);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);

  // Start all 7 targets DOWN
  for (int channel = 0; channel < 7; channel++) {
    pwm.setPWM(channel, 0, SERVO_DOWN);
    currentPos[channel] = SERVO_DOWN;
  }

  // 6. HTTP Endpoints for Laptop Dashboard
  server.on("/cmd", HTTP_OPTIONS, handleOptions);
  server.on("/telemetry", HTTP_OPTIONS, handleOptions);
  server.on("/status", HTTP_OPTIONS, handleOptions);
  server.on("/ping", HTTP_OPTIONS, handleOptions);
  server.on("/", HTTP_OPTIONS, handleOptions);

  // Laptop sends UP,1 / DOWN,1 over Wi-Fi: http://192.168.4.1/cmd?action=UP,1
  server.on("/cmd", HTTP_ANY, []() {
    sendCorsHeaders();
    String action = "";
    if (server.hasArg("action")) {
      action = server.arg("action");
    } else if (server.hasArg("plain")) {
      action = server.arg("plain");
    }
    if (action.length() > 0) {
      executeCommand(action);
    }
    server.send(200, "text/plain", "OK:" + action);
  });

  // Laptop polls Hit / Miss status over Wi-Fi: http://192.168.4.1/telemetry
  server.on("/telemetry", HTTP_ANY, []() {
    sendCorsHeaders();
    unsigned long now = millis();
    if (g_hitActive && (now - g_hitTimestampMs > 3000)) {
      g_hitActive = false; // Auto-clear after 3.0s
    }
    // Scale 800x800 P4 coordinates to 160x160 dashboard viewport coordinates
    int cx160 = (g_hitX * 160) / 800;
    int cy160 = (g_hitY * 160) / 800;

    bool p4Link = (g_lastP4StatMs > 0) && ((now - g_lastP4StatMs) < 2500);
    String json = "{";
    json += "\"hit\":" + String(g_hitActive ? "true" : "false") + ",";
    json += "\"x\":" + String(cx160) + ",";
    json += "\"y\":" + String(cy160) + ",";
    json += "\"dartPixels\":" + String(g_hitPixels) + ",";
    json += "\"yellowPx\":" + String(g_cvYellowPx) + ",";
    json += "\"orangePx\":" + String(g_cvOrangePx) + ",";
    json += "\"darkPct\":" + String(g_cvDarkPct, 1) + ",";
    json += "\"p4Link\":" + String(p4Link ? "true" : "false") + ",";
    json += "\"zone\":\"OPTIMAL\",";
    json += "\"targetState\":\"" + targetStateStr + "\",";
    json += "\"uptimeMs\":" + String(now);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/", HTTP_ANY, []() {
    sendCorsHeaders();
    server.send(200, "text/plain", "SNYPTR Pop-Up Target ESP32 Ready. State: " + targetStateStr);
  });

  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      handleOptions();
      return;
    }
    sendCorsHeaders();
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("STATUS: READY");
}

void loop() {
  // 1. Handle Wi-Fi HTTP requests from Laptop Dashboard
  server.handleClient();

  // 2. Non-blocking read from ESP32-P4 on Serial2 (GPIO 16) with timeout guard
  static char s2_buf[128];
  static size_t s2_idx = 0;
  static unsigned long s2_last_byte_ms = 0;
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    s2_last_byte_ms = millis();
    if (c == '\n' || c == '\r') {
      if (s2_idx > 0) {
        s2_buf[s2_idx] = '\0';
        executeCommand(String(s2_buf), true);
        s2_idx = 0;
      }
    } else if (s2_idx < sizeof(s2_buf) - 1) {
      s2_buf[s2_idx++] = c;
    } else {
      s2_idx = 0; // Reset buffer on overflow
    }
  }
  if (s2_idx > 0 && (millis() - s2_last_byte_ms > 120)) {
    s2_idx = 0; // Clear partial noise after 120ms
  }

  // 3. Non-blocking read from USB Serial (GPIO 3 / UART0)
  static char s0_buf[128];
  static size_t s0_idx = 0;
  static unsigned long s0_last_byte_ms = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    s0_last_byte_ms = millis();
    if (c == '\n' || c == '\r') {
      if (s0_idx > 0) {
        s0_buf[s0_idx] = '\0';
        executeCommand(String(s0_buf), false);
        s0_idx = 0;
      }
    } else if (s0_idx < sizeof(s0_buf) - 1) {
      s0_buf[s0_idx++] = c;
    } else {
      s0_idx = 0;
    }
  }
  if (s0_idx > 0 && (millis() - s0_last_byte_ms > 120)) {
    s0_idx = 0;
  }

  // 4. Handle ESP-NOW commands
  if (pendingWirelessCmd) {
    pendingWirelessCmd = false;
    executeCommand(wirelessCmdString, false);
  }
}