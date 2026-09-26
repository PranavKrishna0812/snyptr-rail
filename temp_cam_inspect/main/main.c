#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/jpeg_encode.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_cam_sensor.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/uart_vfs.h"
#include <math.h>

// 32-byte Standalone Target Scoring Metadata Structure (Steganographically injected after JPEG EOI)
typedef struct __attribute__((packed)) {
    uint8_t magic[4];       // 0xDE, 0xAD, 0xBE, 0xEF
    uint8_t laser_found;    // 0 = no, 1 = yes
    uint8_t zone;           // 0 = none, 1 = green, 2 = red
    uint8_t score_ring;     // 0 = miss, 1..10 = ring, 11 = 10X
    uint8_t is_calibrated;  // 0 = no, 1 = yes
    uint16_t laser_x_px;    // Camera pixel coordinates
    uint16_t laser_y_px;
    float laser_x_mm;       // Calibrated coordinates in mm relative to target card
    float laser_y_mm;
    float laser_dist_mm;    // Radial distance from center in mm
    uint8_t padding[8];     // Padding to ensure exactly 32 bytes
} shot_metadata_t;

// Global Calibration State
static float g_homography[9] = {1.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,  0.0f, 0.0f, 1.0f}; // Identity matrix
static bool g_is_calibrated = false;

static const char *TAG = "P4_UART_STREAM";

// TEST CONFIGURATION MODES (ONLY ONE SHOULD BE 1 AT A TIME FOR ISOLATED TESTING)
#define UART_DIAGNOSTIC_MODE    0
#define CAMERA_TEST_MODE        0
#define SINGLE_FRAME_TEST_MODE  0
#define SINGLE_RAW_FRAME_TEST_MODE 0
#define USB_TRANSPORT_TEST_MODE 0
#define USB_TEST_PAYLOAD_SIZE   4096

// TARGET STREAM RATE (FPS) - Paces the stream to not overload the UART bandwidth
#define STREAM_MAX_FPS          12

// I2C Pin definitions for ESP32-P4 Pico board (Camera)
#define I2C_MASTER_SCL_IO   GPIO_NUM_8
#define I2C_MASTER_SDA_IO   GPIO_NUM_7
#define I2C_MASTER_NUM      I2C_NUM_0

// UART Pin definitions for Bridge (GPIO 21 TX -> Pop-Up RX2, GPIO 22 RX -> Pop-Up TX2)
#define BRIDGE_UART_NUM     UART_NUM_1
#define BRIDGE_TX_PIN       GPIO_NUM_21
#define BRIDGE_RX_PIN       GPIO_NUM_22
#define BRIDGE_BAUD         115200

#if !UART_DIAGNOSTIC_MODE
static int g_video_fd = -1;
static uint8_t *g_buffers[2] = {NULL, NULL};
static uint32_t g_buf_len[2] = {0, 0};

static jpeg_encoder_handle_t g_jpeg_handle = NULL;
static uint8_t *g_jpeg_out_buf = NULL;
static uint32_t g_jpeg_out_size = 0;

// Diagnostics counters
static uint32_t __attribute__((unused)) stat_captured = 0;
static uint32_t __attribute__((unused)) stat_encoded = 0;
static uint64_t __attribute__((unused)) stat_jpeg_bytes_total = 0;
static uint32_t __attribute__((unused)) stat_transmitted = 0;
static uint64_t __attribute__((unused)) stat_tx_bytes_total = 0;
#endif

// Calculate CRC32 (Standard IEEE 802.3 ethernet representation)
static uint32_t calculate_crc32(const uint8_t *data, size_t length)
{
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

// Send binary packet over UART with Sync, Version, Frame ID, Length, and CRC32
static void __attribute__((unused)) send_packet(uint32_t frame_id, const uint8_t *payload, uint32_t payload_len)
{
    uint8_t header[15];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = 0x01; // Version

    // Little-endian frame ID
    header[3] = (uint8_t)(frame_id & 0xFF);
    header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
    header[5] = (uint8_t)((frame_id >> 16) & 0xFF);
    header[6] = (uint8_t)((frame_id >> 24) & 0xFF);

    // Little-endian payload length
    header[7] = (uint8_t)(payload_len & 0xFF);
    header[8] = (uint8_t)((payload_len >> 8) & 0xFF);
    header[9] = (uint8_t)((payload_len >> 16) & 0xFF);
    header[10] = (uint8_t)((payload_len >> 24) & 0xFF);

    // Calculate CRC32 of the payload
    uint32_t crc32 = calculate_crc32(payload, payload_len);
    header[11] = (uint8_t)(crc32 & 0xFF);
    header[12] = (uint8_t)((crc32 >> 8) & 0xFF);
    header[13] = (uint8_t)((crc32 >> 16) & 0xFF);
    header[14] = (uint8_t)((crc32 >> 24) & 0xFF);

    // Send over USB-Serial-JTAG (if driver is installed and connected) - Commented out because we are using standard UART0 console over COM15
    // if (usb_serial_jtag_is_driver_installed() && usb_serial_jtag_is_connected()) {
    //     usb_serial_jtag_write_bytes(header, sizeof(header), pdMS_TO_TICKS(100));
    //     usb_serial_jtag_write_bytes(payload, payload_len, pdMS_TO_TICKS(100));
    //     usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    // }

    // Send directly over raw UART0 (COM15) to bypass VFS console overhead
    uart_write_bytes(UART_NUM_0, header, sizeof(header));
    uart_write_bytes(UART_NUM_0, payload, payload_len);
    // ESP_LOGI(TAG, "stdout write return: w1=%d, w2=%d", (int)w1, (int)w2);

    // Send header (Commented out to bypass the slow 115200 baud UART bridge bottleneck)
    // uart_write_bytes(BRIDGE_UART_NUM, (const char *)header, sizeof(header));
    // Send payload
    // uart_write_bytes(BRIDGE_UART_NUM, (const char *)payload, payload_len);

    // Block until UART TX is completed to regulate throughput & avoid buffer overflow
    // uart_wait_tx_done(BRIDGE_UART_NUM, pdMS_TO_TICKS(150));
}

// Calculate ring score and red/green zone based on 50x50mm target specs (2cm diameter center circle)
static void get_target_score(float target_x, float target_y, uint8_t *ring, uint8_t *zone, float *distance)
{
    float dx = target_x - 25.0f; // Center is at 25.0mm on a 50x50mm target
    float dy = target_y - 25.0f;
    float r = sqrtf(dx * dx + dy * dy);
    *distance = r;
    
    // Zone classification: White circle is Green Zone (diameter 2cm -> radius 10mm)
    if (r <= 10.0f) {
        *zone = 1; // Green Zone
    } else {
        *zone = 2; // Red Zone
    }
    
    // Ring classification (Rings 7-10 map to Green Zone, Rings 1-6 map to Red Zone)
    if (r <= 0.25f) {
        *ring = 11; // 10X
    } else if (r <= 1.5f) {
        *ring = 10;
    } else if (r <= 4.0f) {
        *ring = 9;
    } else if (r <= 7.0f) {
        *ring = 8;
    } else if (r <= 10.0f) {
        *ring = 7;
    } else if (r <= 13.0f) {
        *ring = 6;
    } else if (r <= 16.0f) {
        *ring = 5;
    } else if (r <= 19.0f) {
        *ring = 4;
    } else if (r <= 22.0f) {
        *ring = 3;
    } else if (r <= 25.0f) {
        *ring = 2;
    } else if (r <= 28.0f) {
        *ring = 1;
    } else {
        *ring = 0; // Miss
    }
}

// Background task to receive target homography calibration matrices over UART0
static void uart_rx_task(void *arg)
{
    uint8_t buffer[128];
    int idx = 0;
    int64_t last_byte_time = 0;
    
    ESP_LOGI("STANDALONE_CV", "UART RX Calibration task started...");
    
    while (1) {
        uint8_t byte;
        int len = uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            int64_t now = esp_timer_get_time();
            if (idx > 0 && (now - last_byte_time > 100000)) { // 100ms packet timeout
                idx = 0;
            }
            last_byte_time = now;
            
            if (idx == 0) {
                if (byte == 0xCC) {
                    buffer[idx++] = byte;
                }
            } else if (idx == 1) {
                if (byte == 0x01) {
                    buffer[idx++] = byte;
                } else {
                    idx = 0;
                }
            } else if (idx >= 2 && idx < 38) {
                buffer[idx++] = byte;
            } else if (idx == 38) {
                buffer[idx] = byte;
                
                // Verify XOR checksum
                uint8_t check = 0;
                for (int i = 1; i < 38; i++) {
                    check ^= buffer[i];
                }
                
                if (check == buffer[38]) {
                    memcpy(g_homography, &buffer[2], 36);
                    g_is_calibrated = true;
                    ESP_LOGI("STANDALONE_CV", "Homography calibrated successfully!");
                } else {
                    ESP_LOGE("STANDALONE_CV", "Checksum error! Calculated: 0x%02X, Recv: 0x%02X", check, buffer[38]);
                }
                idx = 0; // Reset packet state
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// Initialize USB-Serial-JTAG driver
static void init_usb_serial_jtag(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = 65536; // 64KB TX buffer to hold larger JPEG frames
    cfg.rx_buffer_size = 1024;
    
    esp_err_t ret = usb_serial_jtag_driver_install(&cfg);
    if (ret == ESP_OK) {
        if (usb_serial_jtag_is_connected()) {
            usb_serial_jtag_vfs_use_driver();
            usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
            ESP_LOGI(TAG, "USB-Serial-JTAG connected. VFS switched to driver mode.");
        } else {
            ESP_LOGI(TAG, "USB-Serial-JTAG driver installed. Keeping console on UART0.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to install USB-Serial-JTAG driver: %s", esp_err_to_name(ret));
    }
}

// Initialize UART for bridging
static void init_bridge_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_NUM, 4096, 32768, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART_NUM, BRIDGE_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Bridge UART%d initialized at %d baud on TX:%d RX:DISABLED", BRIDGE_UART_NUM, BRIDGE_BAUD, BRIDGE_TX_PIN);
}

#if UART_DIAGNOSTIC_MODE
// UART Diagnostic Mode Task
static void uart_diagnostic_task(void *arg)
{
    uint32_t frame_id = 0;
    const char *payload = "UART_TEST_PACKET_123456";
    uint32_t payload_len = 23;

    ESP_LOGI(TAG, "UART Diagnostic Mode active. Sending test packets...");

    while (true) {
        frame_id++;
        send_packet(frame_id, (const uint8_t *)payload, payload_len);
        ESP_LOGI(TAG, "Sent UART diagnostic packet %lu", frame_id);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#if !UART_DIAGNOSTIC_MODE
// Helper to send data over USB-Serial-JTAG in chunks with a finite timeout
static esp_err_t send_via_usb_serial_jtag(const uint8_t *buffer, size_t length, TickType_t timeout_per_write)
{
    size_t offset = 0;
    const size_t chunk_size = 1024;
    int timeout_retries = 0;
    const int max_retries = 10; // Abort after 10 consecutive timeouts with no progress

    while (offset < length) {
        size_t bytes_to_write = length - offset;
        if (bytes_to_write > chunk_size) {
            bytes_to_write = chunk_size;
        }

        int written = usb_serial_jtag_write_bytes(buffer + offset, bytes_to_write, timeout_per_write);
        if (written < 0) {
            ESP_LOGE(TAG, "usb_serial_jtag_write_bytes returned error: %d", written);
            return ESP_FAIL;
        }
        if (written == 0) {
            timeout_retries++;
            if (timeout_retries >= max_retries) {
                ESP_LOGE(TAG, "USB JTAG write timeout: Host stopped consuming data.");
                return ESP_ERR_TIMEOUT;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        timeout_retries = 0; // Reset retries on successful write progress
        offset += written;
    }
    return ESP_OK;
}

static esp_err_t send_via_uart0(const uint8_t *buffer, size_t length)
{
    size_t offset = 0;
    const size_t chunk_size = 1024;
    while (offset < length) {
        size_t bytes_to_write = length - offset;
        if (bytes_to_write > chunk_size) {
            bytes_to_write = chunk_size;
        }
        int written = uart_write_bytes(UART_NUM_0, buffer + offset, bytes_to_write);
        if (written < 0) {
            ESP_LOGE(TAG, "uart_write_bytes returned error: %d", written);
            return ESP_FAIL;
        }
        offset += written;
    }
    return ESP_OK;
}

static void set_camera_exposure_target(int value)
{
#if !SINGLE_FRAME_TEST_MODE
    struct v4l2_ext_control ext_ctrl = {
        .id = V4L2_CID_EXPOSURE,
        .value = value,
    };
    struct v4l2_ext_controls ext_ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &ext_ctrl,
    };
    if (ioctl(g_video_fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls) == 0) {
        ESP_LOGI("CAMERA", "Successfully set camera AE exposure target to %d", value);
    } else {
        ESP_LOGW("CAMERA", "Failed to set camera AE exposure target");
    }
#endif
}

// --- EMBEDDED LIGHTWEIGHT MODEL: YELLOW NERF BULLET DETECTOR WITH BASELINE DIFFERENCING ---
// Scans the ENTIRE BLACK OPTIMAL REGION (Center 400, 400, Radius 275 px on 800x800 frame -> matches r=55 on 160x160)
#define NERF_ROI_SIZE 110 // 110x110 grid sampled with step 5 = 550x550 pixels (Center 400 +/- 275 px)
static uint16_t s_nerf_baseline_roi[NERF_ROI_SIZE][NERF_ROI_SIZE] = {0};
static int s_baseline_sample_count = 0;
static bool s_baseline_ready = false;
static int64_t s_hit_cooldown_until_us = 0;
static uint32_t s_last_dart_pixel_count = 0;

typedef struct {
    float last_x;
    float last_y;
    int stable_frames;
    bool is_stuck;
    bool in_optimal_zone;
} nerf_tracker_t;

static nerf_tracker_t g_nerf_tracker = {0};

static bool detect_yellow_nerf_bullet(const uint16_t *pixels, int width, int height,
                                      float *out_x, float *out_y, bool *out_stuck, bool *out_optimal)
{
    int64_t now_us = esp_timer_get_time();

    // Check if Pop-Up ESP32 sent "ARM,1" over TX2 -> P4 GPIO 22 when target popped UP
    uint8_t bridge_rx_buf[32];
    int rx_len = uart_read_bytes(BRIDGE_UART_NUM, bridge_rx_buf, sizeof(bridge_rx_buf) - 1, 0);
    if (rx_len > 0) {
        bridge_rx_buf[rx_len] = '\0';
        if (strstr((const char *)bridge_rx_buf, "ARM") != NULL) {
            // Wait 600ms for servo motion & mechanical wobble to stop before sampling upright baseline
            s_hit_cooldown_until_us = now_us + 600000ULL;
            s_baseline_ready = false;
            s_baseline_sample_count = 0;
            memset(&g_nerf_tracker, 0, sizeof(g_nerf_tracker));
            ESP_LOGI("NERF_CV", "🎯 Target Popped UP (ARM received) -> Re-zeroing baseline in 600ms!");
            *out_stuck = false;
            *out_optimal = false;
            return false;
        }
    }

    // If in post-hit or post-ARM settle window, wait until timer expires then re-calibrate clean baseline
    if (s_hit_cooldown_until_us > 0) {
        if (now_us < s_hit_cooldown_until_us) {
            *out_stuck = false;
            *out_optimal = false;
            return false;
        }
        // Cooldown elapsed: clear shot and re-arm baseline for next pop
        s_hit_cooldown_until_us = 0;
        s_baseline_ready = false;
        s_baseline_sample_count = 0;
        memset(&g_nerf_tracker, 0, sizeof(g_nerf_tracker));
        const char *clear_msg = "CLEAR,1\n";
        uart_write_bytes(BRIDGE_UART_NUM, clear_msg, strlen(clear_msg));
        ESP_LOGI("NERF_CV", "🔄 Cooldown Complete: Shot cleared, re-arming optimal black region baseline.");
    }

    // Auto-learn clean target baseline for the optimal black silhouette during first 8 frames
    if (!s_baseline_ready) {
        for (int gy = 0; gy < NERF_ROI_SIZE; gy++) {
            int y = 125 + gy * 5; // Center 400 +/- 275 px
            int row = y * width;
            for (int gx = 0; gx < NERF_ROI_SIZE; gx++) {
                int x = 125 + gx * 5;
                s_nerf_baseline_roi[gy][gx] = pixels[row + x];
            }
        }
        s_baseline_sample_count++;
        if (s_baseline_sample_count >= 8) {
            s_baseline_ready = true;
            ESP_LOGI("NERF_CV", "🎯 Optimal Black Region Baseline Calibrated (r=275px)! Armed for yellow Nerf impact.");
        }
        *out_stuck = false;
        *out_optimal = false;
        return false;
    }

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint32_t dart_changed_count = 0;

    // Scan ENTIRE BLACK SILHOUETTE (Center 400, 400, Radius 275 pixels)
    for (int gy = 0; gy < NERF_ROI_SIZE; gy++) {
        int y = 125 + gy * 5;
        int dy = y - 400;
        int row = y * width;

        for (int gx = 0; gx < NERF_ROI_SIZE; gx++) {
            int x = 125 + gx * 5;
            int dx = x - 400;

            if (dx * dx + dy * dy > 275 * 275) {
                continue; // Strictly within optimal black region circle
            }

            uint16_t p = pixels[row + x];
            uint16_t base = s_nerf_baseline_roi[gy][gx];

            // Convert RGB565 to 8-bit RGB888 equivalents (0..255)
            int r8 = ((p >> 11) & 0x1F) * 8;
            int g8 = ((p >> 5) & 0x3F) * 4;
            int b8 = (p & 0x1F) * 8;

            int br8 = ((base >> 11) & 0x1F) * 8;
            int bg8 = ((base >> 5) & 0x3F) * 4;
            int bb8 = (base & 0x1F) * 8;

            // 1. Baseline pixel MUST have been part of the DARK/BLACK target silhouette (not bright background wall!)
            bool base_was_black_target = (br8 <= 115 && bg8 <= 115 && bb8 <= 115);

            // 2. Current pixel MUST be bright Yellow/Orange Nerf foam/tip
            bool is_nerf_dart = (
                r8 >= 135 &&
                b8 <= 70 &&
                (r8 - b8) >= 65 &&
                g8 >= 50 &&
                (g8 - b8) >= 20 &&
                r8 >= (g8 - 20)
            );

            int delta = (abs(r8 - br8) + abs(g8 - bg8) + abs(b8 - bb8)) / 3;

            // Only count pixels that transitioned from BLACK target -> BRIGHT YELLOW/ORANGE Nerf dart!
            if (base_was_black_target && delta >= 35 && is_nerf_dart) {
                sum_x += x;
                sum_y += y;
                dart_changed_count++;
            }
        }
    }

    s_last_dart_pixel_count = dart_changed_count;

    // Trigger ONLY on genuine changed Black->Yellow Nerf bullet cluster (35..6000 grid points)
    // (Removed raw_dart_count fallback so static background yellow objects NEVER false-trigger!)
    bool is_bullet_hit = (dart_changed_count >= 35 && dart_changed_count <= 6000);

    if (is_bullet_hit) {
        uint32_t active_cnt = dart_changed_count;
        float cx = (float)sum_x / (float)dart_changed_count;
        float cy = (float)sum_y / (float)dart_changed_count;
        *out_x = cx;
        *out_y = cy;

        float dx = cx - g_nerf_tracker.last_x;
        float dy = cy - g_nerf_tracker.last_y;
        float shift = sqrtf(dx * dx + dy * dy);

        if (shift <= 45.0f) {
            g_nerf_tracker.stable_frames++;
        } else {
            g_nerf_tracker.stable_frames = 1;
        }

        g_nerf_tracker.last_x = cx;
        g_nerf_tracker.last_y = cy;

        // Confirm STUCK in >= 2 consecutive frames (~100ms) for immediate drop reaction
        bool stuck = (g_nerf_tracker.stable_frames >= 2);
        g_nerf_tracker.is_stuck = stuck;
        g_nerf_tracker.in_optimal_zone = stuck;

        *out_stuck = stuck;
        *out_optimal = stuck;

        if (stuck) {
            // Send HIT telemetry FIRST, then DOWN,1 to drop the servo!
            char hit_telemetry[64];
            int hlen = snprintf(hit_telemetry, sizeof(hit_telemetry), "HIT,1,%.0f,%.0f,%lu\n",
                                cx, cy, (unsigned long)active_cnt);
            uart_write_bytes(BRIDGE_UART_NUM, hit_telemetry, hlen);

            const char *drop_cmd = "DOWN,1\n";
            uart_write_bytes(BRIDGE_UART_NUM, drop_cmd, strlen(drop_cmd));

            // Start 3.0s lock before auto-clearing
            s_hit_cooldown_until_us = now_us + 3000000ULL;
        }

        return true;
    } else {
        // Adapt baseline slowly for ambient lighting when empty
        if (dart_changed_count < 5 && s_baseline_ready) {
            for (int gy = 0; gy < NERF_ROI_SIZE; gy++) {
                int y = 125 + gy * 5;
                int row = y * width;
                for (int gx = 0; gx < NERF_ROI_SIZE; gx++) {
                    int x = 125 + gx * 5;
                    s_nerf_baseline_roi[gy][gx] = pixels[row + x];
                }
            }
        }
        if (g_nerf_tracker.stable_frames > 0) {
            g_nerf_tracker.stable_frames--;
        }
        *out_stuck = false;
        *out_optimal = false;
        return false;
    }
}

// Camera Capture and Stream Task
static void camera_stream_task(void *arg)
{
#if USB_TRANSPORT_TEST_MODE
    ESP_LOGI(TAG, "USB_TRANSPORT_TEST_MODE active. Testing USB transport with %d bytes...", USB_TEST_PAYLOAD_SIZE);
    
    // Allocate a buffer for the test pattern
    uint8_t *test_payload = malloc(USB_TEST_PAYLOAD_SIZE);
    if (!test_payload) {
        ESP_LOGE(TAG, "Failed to allocate test payload");
        vTaskDelete(NULL);
        return;
    }
    // Fill the payload with a simple predictable test pattern
    for (int i = 0; i < USB_TEST_PAYLOAD_SIZE; i++) {
        test_payload[i] = (uint8_t)(i & 0xFF);
    }
    
    // Allow host USB port initialization to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t out_len = USB_TEST_PAYLOAD_SIZE;
    extern uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len);
    uint32_t crc32 = esp_rom_crc32_le(0xFFFFFFFF, test_payload, out_len) ^ 0xFFFFFFFF;

    printf("RAW CRC: %08lX\n", (unsigned long)crc32);
    fflush(stdout);

    uint8_t header[15];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = 0x01;
    
    uint32_t frame_id = 1;
    header[3] = (uint8_t)(frame_id & 0xFF);
    header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
    header[5] = (uint8_t)((frame_id >> 16) & 0xFF);
    header[6] = (uint8_t)((frame_id >> 24) & 0xFF);

    header[7] = (uint8_t)(out_len & 0xFF);
    header[8] = (uint8_t)((out_len >> 8) & 0xFF);
    header[9] = (uint8_t)((out_len >> 16) & 0xFF);
    header[10] = (uint8_t)((out_len >> 24) & 0xFF);

    header[11] = (uint8_t)(crc32 & 0xFF);
    header[12] = (uint8_t)((crc32 >> 8) & 0xFF);
    header[13] = (uint8_t)((crc32 >> 16) & 0xFF);
    header[14] = (uint8_t)((crc32 >> 24) & 0xFF);

    printf("Sending RAW Bayer packet (frame=1, len=%lu) exclusively over COM15...\n", (unsigned long)out_len);
    fflush(stdout);
    
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t err = send_via_uart0(header, sizeof(header));
    if (err == ESP_OK) {
        err = send_via_uart0(test_payload, out_len);
    }

    if (err == ESP_OK) {
        esp_err_t tx_done_err = uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(5000));
        if (tx_done_err == ESP_OK) {
            printf("RAW TRANSMISSION COMPLETE\n");
            fflush(stdout);
        } else {
            ESP_LOGE(TAG, "uart_wait_tx_done timed out: %s", esp_err_to_name(tx_done_err));
        }
    } else {
        ESP_LOGE(TAG, "Failed to transmit test packet over UART0: %s", esp_err_to_name(err));
    }

    free(test_payload);
    vTaskDelete(NULL);
    return;
#else // !USB_TRANSPORT_TEST_MODE
#if SINGLE_RAW_FRAME_TEST_MODE
    struct v4l2_buffer buf = {0};
    
    // Discard the first 100 frames to let the sensor wake up, stabilize exposure, and output valid pixels
    ESP_LOGI(TAG, "SINGLE_RAW_FRAME_TEST_MODE: Skipping first 100 frames for sensor stabilization...");
    for (int i = 0; i < 100; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
            ioctl(g_video_fd, VIDIOC_QBUF, &buf);
        }
    }

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_LOGI(TAG, "SINGLE_RAW_FRAME_TEST_MODE: Dequeuing stabilized raw frame (Frame 26)...");
    if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(g_video_fd, VIDIOC_G_FMT, &fmt);

        char fmt_str[5] = {0};
        memcpy(fmt_str, &fmt.fmt.pix.pixelformat, 4);

        printf("RAW DIAGNOSTICS: width=%ld, height=%ld, bytesperline=%ld, pixelformat=%s (0x%lX), bytesused=%ld, expected_size=%d, buffer_length=%ld, buffer_index=%ld\n",
               (long)fmt.fmt.pix.width, 
               (long)fmt.fmt.pix.height, 
               (long)fmt.fmt.pix.bytesperline, 
               fmt_str, 
               (long)fmt.fmt.pix.pixelformat, 
               (long)buf.bytesused, 
               800*800, 
               (long)buf.length, 
               (long)buf.index);

        uint8_t *frame_ptr = g_buffers[buf.index];
        uint32_t out_len = buf.bytesused;

        if (frame_ptr != NULL && out_len > 0) {
            extern uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len);
            uint32_t crc32 = esp_rom_crc32_le(0xFFFFFFFF, frame_ptr, out_len) ^ 0xFFFFFFFF;

            printf("RAW CRC: %08lX\n", (unsigned long)crc32);

            uint8_t header[15];
            header[0] = 0xAA;
            header[1] = 0x55;
            header[2] = 0x01;
            
            uint32_t frame_id = 1;
            header[3] = (uint8_t)(frame_id & 0xFF);
            header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
            header[5] = (uint8_t)((frame_id >> 16) & 0xFF);
            header[6] = (uint8_t)((frame_id >> 24) & 0xFF);

            header[7] = (uint8_t)(out_len & 0xFF);
            header[8] = (uint8_t)((out_len >> 8) & 0xFF);
            header[9] = (uint8_t)((out_len >> 16) & 0xFF);
            header[10] = (uint8_t)((out_len >> 24) & 0xFF);

            header[11] = (uint8_t)(crc32 & 0xFF);
            header[12] = (uint8_t)((crc32 >> 8) & 0xFF);
            header[13] = (uint8_t)((crc32 >> 16) & 0xFF);
            header[14] = (uint8_t)((crc32 >> 24) & 0xFF);

            printf("Sending RAW Bayer packet (frame=1, len=%lu) exclusively over COM15...\n", (unsigned long)out_len);
            fflush(stdout);
            
            // Allow previous stdout/console logs to fully flush
            vTaskDelay(pdMS_TO_TICKS(100));

            // Write raw bytes directly to USB-Serial-JTAG driver in chunked fashion to avoid VFS/console corruption
            esp_err_t err = send_via_uart0(header, sizeof(header));
            if (err == ESP_OK) {
                err = send_via_uart0(frame_ptr, out_len);
            }

            if (err == ESP_OK) {
                esp_err_t tx_done_err = uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(5000));
                if (tx_done_err == ESP_OK) {
                    printf("RAW TRANSMISSION COMPLETE\n");
                    fflush(stdout);
                } else {
                    ESP_LOGE(TAG, "uart_wait_tx_done timed out: %s", esp_err_to_name(tx_done_err));
                }
            } else {
                ESP_LOGE(TAG, "Failed to transmit raw packet over UART0: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "Invalid buffer or length (ptr=%p, len=%lu)", frame_ptr, (unsigned long)out_len);
        }

        ioctl(g_video_fd, VIDIOC_QBUF, &buf);
    } else {
        ESP_LOGE(TAG, "Failed to dequeue buffer from /dev/video0");
    }

    ESP_LOGI(TAG, "SINGLE_RAW_FRAME_TEST_MODE: Test complete. Deleting task.");
    vTaskDelete(NULL);
#else
    struct v4l2_buffer buf = {0};
    int frame_count = 0;
    int64_t last_transmit_time = 0;
    int64_t last_stat_time = esp_timer_get_time();
    const int64_t frame_interval_us = 1000000 / STREAM_MAX_FPS;

    ESP_LOGI(TAG, "Starting camera capture loop (Max Target FPS: %d)...", STREAM_MAX_FPS);

    while (true) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Dequeue frame from V4L2 camera driver (blocks until a frame is ready)
        if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
            frame_count++;
            
            // Pace the frame stream to target FPS
            int64_t now = esp_timer_get_time();
            if (now - last_transmit_time < frame_interval_us) {
                // Drop this frame to limit FPS in software without encoding / transmitting it
                ioctl(g_video_fd, VIDIOC_QBUF, &buf);
                continue;
            }
            last_transmit_time = now;
            stat_captured++;

            uint8_t *frame_ptr = g_buffers[buf.index];
            uint32_t out_len = buf.bytesused;

            if (frame_ptr != NULL && out_len > 0) {
                // Dynamically manage camera exposure based on calibration state
                static bool s_was_calibrated = false;
                if (g_is_calibrated != s_was_calibrated) {
                    s_was_calibrated = g_is_calibrated;
                    if (g_is_calibrated) {
                        // Dim camera (AE target = 15) to prevent laser saturation on the white card
                        set_camera_exposure_target(15);
                    } else {
                        // Restore normal brightness (AE target = 60) for target scanning
                        set_camera_exposure_target(60);
                    }
                }

                // --- STANDALONE COMPUTER VISION PIPELINE ---
                int best_x = -1;
                int best_y = -1;
                int best_score = -100;
                
                uint16_t *pixels = (uint16_t *)frame_ptr;
                // Scan every 2nd pixel for speed (scans 160,000 pixels in ~1ms)
                for (int y = 4; y < 796; y += 2) {
                    for (int x = 4; x < 796; x += 2) {
                        uint16_t pixel = pixels[y * 800 + x];
                        int r = (pixel >> 11) & 0x1F;
                        int g = (pixel >> 5) & 0x3F;
                        int b = pixel & 0x1F;
                        
                        int g_norm = g >> 1; // Normalize green (6-bit) to 5-bit
                        int score = r * 2 - g_norm - b; // High-pass red difference
                        
                        // Adaptive thresholding: Use threshold of 12 in white circle (high green/blue) and 18 in dark/background areas
                        int thresh = (g_norm > 15 && b > 15) ? 12 : 18;
                        if (score >= thresh) {
                            int relative_score = score - thresh;
                            if (relative_score > best_score) {
                                best_score = relative_score;
                                best_x = x;
                                best_y = y;
                            }
                        }
                    }
                }
                
                bool laser_found = (best_score >= 0); // Exceeded adaptive threshold if relative_score >= 0
                double laser_x = best_x;
                double laser_y = best_y;
                
                if (laser_found) {
                    // Refine coordinates using 9x9 centroid around the peak
                    double sum_x = 0;
                    double sum_y = 0;
                    double sum_w = 0;
                    for (int dy = -4; dy <= 4; dy++) {
                        for (int dx = -4; dx <= 4; dx++) {
                            int px = best_x + dx;
                            int py = best_y + dy;
                            if (px >= 0 && px < 800 && py >= 0 && py < 800) {
                                uint16_t p = pixels[py * 800 + px];
                                int r = (p >> 11) & 0x1F;
                                int g = (p >> 5) & 0x3F;
                                int b = p & 0x1F;
                                int g_norm = g >> 1;
                                int w = r * 2 - g_norm - b;
                                int min_w = (g_norm > 15 && b > 15) ? 8 : 10;
                                if (w >= min_w) {
                                    sum_x += px * w;
                                    sum_y += py * w;
                                    sum_w += w;
                                }
                            }
                        }
                    }
                    if (sum_w > 0) {
                        laser_x = sum_x / sum_w;
                        laser_y = sum_y / sum_w;
                    }
                }
                
                // Homography mapping and scoring calculations
                float target_x = 0.0f;
                float target_y = 0.0f;
                float distance = 0.0f;
                uint8_t ring = 0;
                uint8_t zone = 0;
                
                bool valid_laser_on_card = false;
                if (laser_found && g_is_calibrated) {
                    float h00 = g_homography[0];
                    float h01 = g_homography[1];
                    float h02 = g_homography[2];
                    float h10 = g_homography[3];
                    float h11 = g_homography[4];
                    float h12 = g_homography[5];
                    float h20 = g_homography[6];
                    float h21 = g_homography[7];
                    float h22 = g_homography[8];
                    
                    double denom = h20 * laser_x + h21 * laser_y + h22;
                    if (fabs(denom) > 1e-5) {
                        target_x = (h00 * laser_x + h01 * laser_y + h02) / denom;
                        target_y = (h10 * laser_x + h11 * laser_y + h12) / denom;
                        
                        // Limit detection to target card boundary (50x50mm target, strict 0mm margin to ignore table reflections)
                        if (target_x >= 0.0f && target_x <= 50.0f && target_y >= 0.0f && target_y <= 50.0f) {
                            valid_laser_on_card = true;
                        }
                    }
                    
                    get_target_score(target_x, target_y, &ring, &zone, &distance);
                }

                // --- RUN EMBEDDED YELLOW NERF BULLET DETECTOR ---
                float nerf_x = 0.0f, nerf_y = 0.0f;
                bool nerf_stuck = false, nerf_optimal = false;
                bool nerf_detected = detect_yellow_nerf_bullet(pixels, 800, 800, &nerf_x, &nerf_y, &nerf_stuck, &nerf_optimal);

                if (nerf_detected && nerf_stuck) {
                    valid_laser_on_card = true;
                    laser_x = nerf_x;
                    laser_y = nerf_y;
                    if (nerf_optimal) {
                        zone = 1; // Green Optimal Zone
                        ring = 11; // 10X Bullseye
                    } else {
                        zone = 2; // Outer Zone
                        ring = 8;
                    }
                    static int64_t last_nerf_log = 0;
                    int64_t now_us = esp_timer_get_time();
                    if (now_us - last_nerf_log > 1000000) {
                        last_nerf_log = now_us;
                        ESP_LOGI("NERF_CV", "🎯 Yellow Nerf Bullet STUCK! Pos: (%.1f, %.1f) | Optimal Zone: %s",
                                 nerf_x, nerf_y, nerf_optimal ? "YES (OPTIMAL GREEN BULLSEYE)" : "NO");
                    }
                }

                // Encode RAW8 to JPEG (Grayscale)
                jpeg_encode_cfg_t enc_config = {
                    .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                    .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
                    .image_quality = 35,
                    .width = 800,
                    .height = 800,
                };
                
                uint32_t jpeg_encoded_size = 0;
                esp_err_t ret = jpeg_encoder_process(g_jpeg_handle, &enc_config, frame_ptr, out_len, g_jpeg_out_buf, g_jpeg_out_size, &jpeg_encoded_size);
                
                if (ret == ESP_OK && jpeg_encoded_size > 0) {
                    // Verify JPEG integrity
                    bool jpeg_ok = (jpeg_encoded_size >= 4 &&
                                    g_jpeg_out_buf[0] == 0xFF &&
                                    g_jpeg_out_buf[1] == 0xD8);
                    
                    bool has_ffd9 = false;
                    if (jpeg_ok) {
                        for (int idx = (int)jpeg_encoded_size - 1; idx >= (int)jpeg_encoded_size - 10 && idx >= 0; idx--) {
                            if (idx > 0 && g_jpeg_out_buf[idx-1] == 0xFF && g_jpeg_out_buf[idx] == 0xD9) {
                                has_ffd9 = true;
                                break;
                            }
                        }
                    }

                    if (jpeg_ok) {
                        stat_encoded++;
                        stat_jpeg_bytes_total += jpeg_encoded_size;

                        if (!has_ffd9) {
                            ESP_LOGW(TAG, "JPEG: FF D9 marker not found in trailing 10 bytes of frame %d", frame_count);
                        }

                        // Inject Trailing 32-byte Metadata Steganographically
                        shot_metadata_t meta = {
                            .magic = {0xDE, 0xAD, 0xBE, 0xEF},
                            .laser_found = valid_laser_on_card ? 1 : 0,
                            .zone = zone,
                            .score_ring = ring,
                            .is_calibrated = g_is_calibrated ? 1 : 0,
                            .laser_x_px = (uint16_t)laser_x,
                            .laser_y_px = (uint16_t)laser_y,
                            .laser_x_mm = target_x,
                            .laser_y_mm = target_y,
                            .laser_dist_mm = distance
                        };
                        
                        // Append metadata immediately after the JPEG data
                        memcpy(g_jpeg_out_buf + jpeg_encoded_size, &meta, sizeof(meta));
                        uint32_t total_payload_len = jpeg_encoded_size + sizeof(meta);

                        #if !CAMERA_TEST_MODE
                            #if SINGLE_FRAME_TEST_MODE
                                static bool single_frame_sent = false;
                                if (!single_frame_sent) {
                                    send_packet(frame_count, g_jpeg_out_buf, total_payload_len);
                                    single_frame_sent = true;
                                    stat_transmitted++;
                                    stat_tx_bytes_total += (15 + total_payload_len);
                                    ESP_LOGI(TAG, "SINGLE_FRAME_TEST_MODE: Sent first frame, stopping further transmission.");
                                }
                            #else
                                send_packet(frame_count, g_jpeg_out_buf, total_payload_len);
                                stat_transmitted++;
                                stat_tx_bytes_total += (15 + total_payload_len);
                            #endif
                        #else
                            // In Camera-only test mode, print captured size to USB serial without sending it over UART
                            printf("Frame %d | JPEG size: %lu\n", frame_count, jpeg_encoded_size);
                        #endif
                    } else {
                        ESP_LOGE(TAG, "JPEG check failed (size=%lu, start=%02X%02X)", 
                                 jpeg_encoded_size, g_jpeg_out_buf[0], g_jpeg_out_buf[1]);
                    }
                } else {
                    ESP_LOGE(TAG, "JPEG Encode failed: %s", esp_err_to_name(ret));
                }
            }

            // Re-queue the buffer back to camera driver
            ioctl(g_video_fd, VIDIOC_QBUF, &buf);
        }

        // Reset diagnostics counters every 2 seconds silently to prevent overflow
        int64_t now_stat = esp_timer_get_time();
        int64_t delta_us = now_stat - last_stat_time;
        if (delta_us >= 2000000) {
            stat_captured = 0;
            stat_encoded = 0;
            stat_jpeg_bytes_total = 0;
            stat_transmitted = 0;
            stat_tx_bytes_total = 0;
            last_stat_time = now_stat;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small yield
    }
#endif
#endif // !USB_TRANSPORT_TEST_MODE
}
#endif

void app_main(void)
{
    // Initialize UART0 at 3,000,000 baud and bind VFS console
    uart_config_t uart_cfg = {
        .baud_rate = 3000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 65536, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
    uart_vfs_dev_use_driver(0);

    // Configure VFS UART0 to raw LF line endings (disable CRLF conversion)
    uart_vfs_dev_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_LF);

    // Launch standalone target calibration receiver task
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL, 1);

    // Initialize USB-Serial-JTAG Driver
    init_usb_serial_jtag();

    // 1. Initialize UART Bridge
    init_bridge_uart();

#if !UART_DIAGNOSTIC_MODE
    // 2. Initialize Camera & Video Engine (ESP32-P4 v2.x API)
    esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = I2C_MASTER_NUM,
                .scl_pin = I2C_MASTER_SCL_IO,
                .sda_pin = I2C_MASTER_SDA_IO,
            },
            .freq = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
    };
    esp_video_init_config_t video_cfg = {
        .csi = &csi_config,
    };
    
    ESP_LOGI(TAG, "Initializing video subsystem...");
    ESP_ERROR_CHECK(esp_video_init(&video_cfg));

    g_video_fd = open("/dev/video0", O_RDWR);
    if (g_video_fd < 0) {
        ESP_LOGE(TAG, "Failed to open video hardware driver!");
        return;
    }
    ESP_LOGI(TAG, "OV5647 detected and initialized successfully.");

    // Configure OV5647 -> RGB565 @ 800x800 via Hardware ISP
    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {
            .width = 800,
            .height = 800,
            .pixelformat = V4L2_PIX_FMT_RGB565, // Hardware ISP converts Bayer to RGB565
        }
    };
    if (ioctl(g_video_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "Failed to set format on /dev/video0");
    }

    // Initialize Hardware JPEG Encoder
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 5000,
    };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&encode_eng_cfg, &g_jpeg_handle));
    
    // Allocate buffer for JPEG output (max size = width * height)
    g_jpeg_out_size = 800 * 800;
    g_jpeg_out_buf = heap_caps_aligned_alloc(64, g_jpeg_out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_jpeg_out_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
        return;
    }

    // Request & Queue Memory Buffers
    struct v4l2_requestbuffers req = {
        .count = 2,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    ioctl(g_video_fd, VIDIOC_REQBUFS, &req);

    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i
        };
        ioctl(g_video_fd, VIDIOC_QUERYBUF, &buf);
        
        g_buffers[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, g_video_fd, buf.m.offset);
        g_buf_len[i] = buf.length;
        
        ioctl(g_video_fd, VIDIOC_QBUF, &buf);
    }

    // Start Streaming Engine Hardware
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(g_video_fd, VIDIOC_STREAMON, &type);
#endif

    // 3. Start the Capture or Diagnostic Task
#if UART_DIAGNOSTIC_MODE
    xTaskCreatePinnedToCore(uart_diagnostic_task, "diag_task", 4096, NULL, 5, NULL, 1);
#else
    xTaskCreatePinnedToCore(camera_stream_task, "cam_task", 8192, NULL, 5, NULL, 1);
#endif
}