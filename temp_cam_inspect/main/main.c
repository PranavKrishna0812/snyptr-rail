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
#define STREAM_MAX_FPS          20

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
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART_NUM, BRIDGE_TX_PIN, BRIDGE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Bridge UART%d initialized at %d baud on TX:%d RX:%d", BRIDGE_UART_NUM, BRIDGE_BAUD, BRIDGE_TX_PIN, BRIDGE_RX_PIN);
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

// --- EMBEDDED LIGHTWEIGHT MODEL: HYBRID DIFFERENTIAL + NORMALIZED CHROMATICITY DETECTOR ---
// Works across wide lighting conditions (indoor LED, 80,000 lux outdoor sunlight, shade, golden hour)
// ============================================================================
// HIGH-SPEED DETERMINISTIC TARGET EVENT DETECTOR (FULL ACTIVE FRAME)
// ============================================================================
// Physical Setup:
// The camera is zoomed directly into the black target region across the entire view.
// Active Region: Full frame [30, 770] x [30, 770] with STEP=4 (186 x 186 grid = 34,596 points).
//
// Lifecycle:
// 1. TARGET DOWN: Camera is IDLE. Zero calibration. Zero triggers.
// 2. TARGET POPPED UP: Pop ESP32 moves servo + 400ms settle -> sends "ARM,1"
// 3. CALIBRATION: Flushes stale queue frames, averages 4 stationary frames of the full black target.
// 4. ARMED: Continuous full-frame monitoring against the calibrated baseline.
// 5. HIT DETECTION:
//    - When a Nerf projectile enters/hits, a LARGE AMOUNT of pixels suddenly changes!
//    - changed_px_count >= 70 (approx 1,120+ full pixels): IMMEDIATE HIT CONFIRMED!
//    - changed_px_count >= 35 with 2-frame persistence: HIT CONFIRMED!
//    - Servo reverses instantly (DOWN,1) and locks state until next cycle.
// ============================================================================

#define FULL_ROI_X_MIN   30
#define FULL_ROI_X_MAX   770
#define FULL_ROI_Y_MIN   30
#define FULL_ROI_Y_MAX   770
#define FULL_ROI_STEP    4

#define ROI_GRID_COLS  ((FULL_ROI_X_MAX - FULL_ROI_X_MIN) / FULL_ROI_STEP + 1) // 186
#define ROI_GRID_ROWS  ((FULL_ROI_Y_MAX - FULL_ROI_Y_MIN) / FULL_ROI_STEP + 1) // 186

typedef enum {
    DETECTOR_STATE_DOWN = 0,         // Target is lowered / retracted. Detection is LOCKED. Zero calibration.
    DETECTOR_STATE_CALIBRATING,      // Target is upright & stationary. Averaging 4 frames for reference.
    DETECTOR_STATE_ARMED,            // Stationary reference established. Continuous full-frame monitoring.
    DETECTOR_STATE_HIT_LOCKED        // Hit confirmed! Reverse command dispatched. Detection locked.
} detector_state_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} roi_pixel_t;

// Persistent reference in PSRAM
static roi_pixel_t *s_ref_grid = NULL;
static uint16_t *s_accum_r = NULL;
static uint16_t *s_accum_g = NULL;
static uint16_t *s_accum_b = NULL;

static detector_state_t g_detector_state = DETECTOR_STATE_DOWN;
static bool s_ref_ready = false;
static bool s_byte_swap_needed = false;
static bool s_swap_calibrated = false;

static int64_t s_hit_cooldown_until_us = 0;
static int s_ref_accum_frames = 0;
static int s_flush_frames = 0;
static int s_diag_frame_div = 0;

static int s_consec_hit_frames = 0;
static float s_last_hit_cx = 400.0f;
static float s_last_hit_cy = 400.0f;

#define STANDALONE_NO_VIDEO_STREAM 0

static inline void unpack_rgb565(uint16_t p, bool swap, int *r, int *g, int *b)
{
    if (swap) {
        p = (uint16_t)((p >> 8) | (p << 8));
    }
    int r5 = (p >> 11) & 0x1F;
    int g6 = (p >> 5) & 0x3F;
    int b5 = p & 0x1F;
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}

static void init_detector_memory(void)
{
    if (s_ref_grid == NULL) {
        size_t grid_bytes = ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(roi_pixel_t);
        size_t accum_bytes = ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(uint16_t);
        s_ref_grid = (roi_pixel_t *)heap_caps_calloc(1, grid_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_accum_r = (uint16_t *)heap_caps_calloc(1, accum_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_accum_g = (uint16_t *)heap_caps_calloc(1, accum_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_accum_b = (uint16_t *)heap_caps_calloc(1, accum_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        ESP_LOGI("EVENT_DETECTOR", "Detector memory allocated: %u KB in PSRAM (Full Frame %dx%d)",
                 (unsigned int)((grid_bytes + accum_bytes * 3) / 1024), ROI_GRID_COLS, ROI_GRID_ROWS);
    }
}

static bool process_target_event_frame(const uint16_t *pixels, int width, int height,
                                       float *out_x, float *out_y, int *out_delta, int *out_color, int *out_motion)
{
    (void)height;
    int64_t frame_start_us = esp_timer_get_time();

    // 1. Process UART commands from Pop ESP32 (Non-blocking)
    uint8_t rx_buf[48];
    int rx_len = uart_read_bytes(BRIDGE_UART_NUM, rx_buf, sizeof(rx_buf) - 1, 0);
    if (rx_len > 0) {
        rx_buf[rx_len] = '\0';
        if (strstr((const char *)rx_buf, "ARM") != NULL || strstr((const char *)rx_buf, "CALIBRATE") != NULL) {
            // Target is ALREADY fully upright and stationary! Start full-frame calibration.
            g_detector_state = DETECTOR_STATE_CALIBRATING;
            s_flush_frames = 3; // Flush stale queued frames from camera buffer
            s_ref_accum_frames = 0;
            s_ref_ready = false;
            s_consec_hit_frames = 0;
            s_hit_cooldown_until_us = 0;
            memset(s_accum_r, 0, ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(uint16_t));
            memset(s_accum_g, 0, ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(uint16_t));
            memset(s_accum_b, 0, ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(uint16_t));
            ESP_LOGI("EVENT_DETECTOR", ">>> [ARM RECEIVED] Target upright. Calibrating full-frame baseline... <<<");
            return false;
        } else if (strstr((const char *)rx_buf, "DISARM") != NULL || strstr((const char *)rx_buf, "DOWN") != NULL) {
            g_detector_state = DETECTOR_STATE_DOWN;
            s_ref_ready = false;
            s_ref_accum_frames = 0;
            s_consec_hit_frames = 0;
            ESP_LOGI("EVENT_DETECTOR", ">>> [DISARM RECEIVED] Target down. Detection locked. <<<");
            return false;
        }
    }

    int64_t now_us = esp_timer_get_time();

    // 2. FSM State Management
    if (g_detector_state == DETECTOR_STATE_DOWN) {
        return false;
    }

    if (g_detector_state == DETECTOR_STATE_HIT_LOCKED) {
        if (now_us > s_hit_cooldown_until_us) {
            g_detector_state = DETECTOR_STATE_DOWN;
            s_ref_ready = false;
            const char *clear_msg = "CLEAR,1\n";
            uart_write_bytes(BRIDGE_UART_NUM, clear_msg, strlen(clear_msg));
            ESP_LOGI("EVENT_DETECTOR", "Cooldown expired -> Target returned to DOWN");
        }
        return false;
    }

    // Auto-detect byte swap on the first upright frame if needed
    if (!s_swap_calibrated) {
        int disp_no_swap = 0, disp_swap = 0;
        for (int dy = -10; dy <= 10; dy += 2) {
            int row = (400 + dy) * width;
            for (int dx = -10; dx <= 10; dx += 2) {
                uint16_t p = pixels[row + 400 + dx];
                int r1, g1, b1, r2, g2, b2;
                unpack_rgb565(p, false, &r1, &g1, &b1);
                unpack_rgb565(p, true, &r2, &g2, &b2);
                disp_no_swap += abs(r1 - g1) + abs(g1 - b1) + abs(b1 - r1);
                disp_swap += abs(r2 - g2) + abs(g2 - b2) + abs(b2 - r2);
            }
        }
        s_byte_swap_needed = (disp_swap < disp_no_swap);
        s_swap_calibrated = true;
        ESP_LOGI("EVENT_DETECTOR", "Camera Endianness calibrated: swap=%s",
                 s_byte_swap_needed ? "TRUE" : "FALSE");
    }

    // 3. CALIBRATION PHASE: Accumulate 4 stationary frames across full frame
    if (g_detector_state == DETECTOR_STATE_CALIBRATING) {
        if (s_flush_frames > 0) {
            s_flush_frames--;
            return false;
        }

        for (int gy = 0; gy < ROI_GRID_ROWS; gy++) {
            int y = FULL_ROI_Y_MIN + gy * FULL_ROI_STEP;
            int row_offset = y * width;
            int grid_row_idx = gy * ROI_GRID_COLS;
            for (int gx = 0; gx < ROI_GRID_COLS; gx++) {
                int x = FULL_ROI_X_MIN + gx * FULL_ROI_STEP;
                uint16_t p = pixels[row_offset + x];
                int r, g, b;
                unpack_rgb565(p, s_byte_swap_needed, &r, &g, &b);
                int idx = grid_row_idx + gx;
                s_accum_r[idx] += (uint16_t)r;
                s_accum_g[idx] += (uint16_t)g;
                s_accum_b[idx] += (uint16_t)b;
            }
        }
        s_ref_accum_frames++;

        if (s_ref_accum_frames >= 4) {
            uint64_t total_r = 0, total_g = 0, total_b = 0;
            uint32_t point_count = ROI_GRID_ROWS * ROI_GRID_COLS;

            for (int gy = 0; gy < ROI_GRID_ROWS; gy++) {
                int grid_row_idx = gy * ROI_GRID_COLS;
                for (int gx = 0; gx < ROI_GRID_COLS; gx++) {
                    int idx = grid_row_idx + gx;
                    s_ref_grid[idx].r = (uint8_t)(s_accum_r[idx] / 4);
                    s_ref_grid[idx].g = (uint8_t)(s_accum_g[idx] / 4);
                    s_ref_grid[idx].b = (uint8_t)(s_accum_b[idx] / 4);
                    total_r += s_ref_grid[idx].r;
                    total_g += s_ref_grid[idx].g;
                    total_b += s_ref_grid[idx].b;
                }
            }

            int avg_r = (int)(total_r / point_count);
            int avg_g = (int)(total_g / point_count);
            int avg_b = (int)(total_b / point_count);

            s_ref_ready = true;
            g_detector_state = DETECTOR_STATE_ARMED;
            s_consec_hit_frames = 0;
            const char *armed_msg = "ARMED,1\n";
            uart_write_bytes(BRIDGE_UART_NUM, armed_msg, strlen(armed_msg));
            ESP_LOGI("EVENT_DETECTOR", "✅ FULL-FRAME TARGET CALIBRATED (Mean RGB: %d,%d,%d). SYSTEM ARMED!",
                     avg_r, avg_g, avg_b);
        }
        return false;
    }

    // 4. ARMED STATE: Continuous Full-Frame Large Pixel Change Detection
    if (g_detector_state != DETECTOR_STATE_ARMED || !s_ref_ready) {
        return false;
    }

    uint32_t changed_px_count = 0;
    uint32_t color_count = 0;

    int min_x = 9999, max_x = -1;
    int min_y = 9999, max_y = -1;
    uint64_t sum_x = 0, sum_y = 0;

    // Scan full frame grid
    for (int gy = 0; gy < ROI_GRID_ROWS; gy++) {
        int y = FULL_ROI_Y_MIN + gy * FULL_ROI_STEP;
        int row_offset = y * width;
        int grid_row_idx = gy * ROI_GRID_COLS;

        for (int gx = 0; gx < ROI_GRID_COLS; gx++) {
            int x = FULL_ROI_X_MIN + gx * FULL_ROI_STEP;
            uint16_t p = pixels[row_offset + x];
            int r, g, b;
            unpack_rgb565(p, s_byte_swap_needed, &r, &g, &b);

            int idx = grid_row_idx + gx;
            int ref_r = s_ref_grid[idx].r;
            int ref_g = s_ref_grid[idx].g;
            int ref_b = s_ref_grid[idx].b;

            int d_ref = abs(r - ref_r) + abs(g - ref_g) + abs(b - ref_b);
            int gain = (r + g + b) - (ref_r + ref_g + ref_b);

            // Nerf projectile chromaticity characteristics
            bool is_yellow = ((r + g >= 155) && (b <= 90) && (r >= b + 18) && (g >= b + 10));
            bool is_orange = ((r >= 115) && (g >= 35) && (b <= 80) && (r >= g + 18) && (r >= b + 25));

            bool is_changed = (is_yellow || is_orange) || (d_ref >= 42 && gain >= 28);
            if (is_changed) {
                changed_px_count++;
                if (is_yellow || is_orange) {
                    color_count++;
                }
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                sum_x += (uint64_t)x;
                sum_y += (uint64_t)y;
            }
        }
    }

    int span_x = (changed_px_count > 0) ? (max_x - min_x) : 0;
    int span_y = (changed_px_count > 0) ? (max_y - min_y) : 0;
    float cx = (changed_px_count > 0) ? ((float)sum_x / (float)changed_px_count) : 400.0f;
    float cy = (changed_px_count > 0) ? ((float)sum_y / (float)changed_px_count) : 400.0f;

    *out_x = cx;
    *out_y = cy;
    *out_delta = (int)changed_px_count;
    *out_color = (int)color_count;
    *out_motion = 0;

    bool hit_confirmed = false;
    int confidence = 0;

    // Direct Hit Detection:
    // When a Nerf projectile strikes/enters, a large amount of pixels changes!
    // Large impact: >= 70 sample points (each point is 16 full image pixels -> 1,120+ pixels!)
    if (changed_px_count >= 70) {
        hit_confirmed = true;
        confidence = 100;
    }
    // Moderate impact: >= 35 sample points (560+ pixels) persisting for 2 frames
    else if (changed_px_count >= 35) {
        if (s_consec_hit_frames >= 1) {
            float dx = cx - s_last_hit_cx;
            float dy = cy - s_last_hit_cy;
            if ((dx * dx + dy * dy) <= (50.0f * 50.0f)) {
                hit_confirmed = true;
                confidence = 95;
            }
        }
        s_consec_hit_frames++;
        s_last_hit_cx = cx;
        s_last_hit_cy = cy;
    } else {
        s_consec_hit_frames = 0;
    }

    float latency_ms = (float)(esp_timer_get_time() - frame_start_us) / 1000.0f;

    if (hit_confirmed) {
        // --- 1. IMMEDIATELY DISPATCH REVERSE COMMAND OVER UART ---
        char hit_cmd[64];
        int hlen = snprintf(hit_cmd, sizeof(hit_cmd), "HIT,1,%.0f,%.0f,%lu\n",
                            cx, cy, (unsigned long)(changed_px_count * 16));
        uart_write_bytes(BRIDGE_UART_NUM, hit_cmd, hlen);

        // 2. Dispatch detailed diagnostic telemetry
        char diag_msg[96];
        int dlen = snprintf(diag_msg, sizeof(diag_msg), "DIAG,HIT,%lu,%lu,0,%d,%d,%d,%.1f\n",
                            (unsigned long)changed_px_count, (unsigned long)color_count,
                            span_x, span_y, confidence, latency_ms);
        uart_write_bytes(BRIDGE_UART_NUM, diag_msg, dlen);

        // 3. Immediately lock state to prevent double-triggering
        g_detector_state = DETECTOR_STATE_HIT_LOCKED;
        s_hit_cooldown_until_us = esp_timer_get_time() + 3000000ULL;

        ESP_LOGI("EVENT_DETECTOR", "🎯 >>> HIT CONFIRMED! <<< Pos:(%.0f,%.0f) ChangedPx:%lu Color:%lu Conf:%d%% Latency:%.2fms -> SERVO REVERSED!",
                 cx, cy, (unsigned long)changed_px_count, (unsigned long)color_count, confidence, latency_ms);
        return true;
    } else {
        // Periodic diagnostic telemetry every 6 frames (~100ms)
        if (++s_diag_frame_div >= 6) {
            s_diag_frame_div = 0;
            char diag_msg[96];
            int dlen = snprintf(diag_msg, sizeof(diag_msg), "DIAG,ARMED,%lu,%lu,0,%d,%d,0,0.0\n",
                                (unsigned long)changed_px_count, (unsigned long)color_count,
                                span_x, span_y);
            uart_write_bytes(BRIDGE_UART_NUM, diag_msg, dlen);
        }
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

    init_detector_memory();
    ESP_LOGI(TAG, "Starting camera capture loop (Full Camera Speed Event Detection)...");

    while (true) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Dequeue frame from V4L2 camera driver (blocks until a frame is ready)
        if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
            frame_count++;
            
            stat_captured++;

            uint8_t *frame_ptr = g_buffers[buf.index];
            uint32_t out_len = buf.bytesused;

            if (frame_ptr != NULL && out_len > 0) {
                // --- 1. HIGH-SPEED DETERMINISTIC EVENT DETECTION (EVERY FRAME) ---
                uint16_t *pixels = (uint16_t *)frame_ptr;
                float hit_x = 0.0f, hit_y = 0.0f;
                int hit_delta = 0, hit_color = 0, hit_motion = 0;
                process_target_event_frame(pixels, 800, 800, &hit_x, &hit_y, &hit_delta, &hit_color, &hit_motion);

                #if !STANDALONE_NO_VIDEO_STREAM
                // --- 2. PACED JPEG VIDEO STREAMING OVER UART0 (COM PORT) ---
                int64_t now_stream = esp_timer_get_time();
                if (now_stream - last_transmit_time >= frame_interval_us) {
                    last_transmit_time = now_stream;

                    jpeg_encode_cfg_t enc_config = {
                        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                        .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
                        .image_quality = 35,
                        .width = 800,
                        .height = 800,
                    };
                    
                    uint32_t jpeg_encoded_size = 0;
                    esp_err_t ret = jpeg_encoder_process(g_jpeg_handle, &enc_config, frame_ptr, out_len, g_jpeg_out_buf, g_jpeg_out_size, &jpeg_encoded_size);
                    
                    if (ret == ESP_OK && jpeg_encoded_size > 0 &&
                        g_jpeg_out_buf[0] == 0xFF && g_jpeg_out_buf[1] == 0xD8) {
                        
                        stat_encoded++;
                        stat_jpeg_bytes_total += jpeg_encoded_size;

                        shot_metadata_t meta = {
                            .magic = {0xDE, 0xAD, 0xBE, 0xEF},
                            .laser_found = (hit_delta > 0) ? 1 : 0,
                            .zone = 1,
                            .score_ring = 10,
                            .is_calibrated = 1,
                            .laser_x_px = (uint16_t)hit_x,
                            .laser_y_px = (uint16_t)hit_y,
                            .laser_x_mm = 0.0f,
                            .laser_y_mm = 0.0f,
                            .laser_dist_mm = (float)hit_delta
                        };
                        
                        memcpy(g_jpeg_out_buf + jpeg_encoded_size, &meta, sizeof(meta));
                        uint32_t total_payload_len = jpeg_encoded_size + sizeof(meta);
                        send_packet(frame_count, g_jpeg_out_buf, total_payload_len);
                        stat_transmitted++;
                        stat_tx_bytes_total += (15 + total_payload_len);
                    }
                }
                #endif // !STANDALONE_NO_VIDEO_STREAM
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