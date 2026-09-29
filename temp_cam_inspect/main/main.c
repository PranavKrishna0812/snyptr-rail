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
// HIGH-SPEED DETERMINISTIC TARGET EVENT DETECTOR (HSED)
// ============================================================================
// Physical Architecture:
// 1. Camera is zoomed into the optimal zone of the pop-up target.
// 2. Optimal ROI: Ellipse centered at (X=400, Y=280) with Rx=160, Ry=150.
//    Strictly excludes the 3D-printed yellow/orange rail at the bottom (Y >= 480).
// 3. FSM Lifecycle:
//    TARGET_DOWN -> ARM -> RISING (850ms settle) -> ESTABLISH_REF (avg 3 frames)
//    -> ARMED (watch optimal ROI at full camera FPS)
//    -> HIT CONFIRMED (multi-signal voting) -> DISPATCH REVERSE (DOWN,1) -> LOCKED
// ============================================================================

#define OPTIMAL_ROI_CENTER_X   400
#define OPTIMAL_ROI_CENTER_Y   280
#define OPTIMAL_ROI_RADIUS_X   160
#define OPTIMAL_ROI_RADIUS_Y   150
#define OPTIMAL_ROI_STEP       2

#define ROI_GRID_COLS  ((2 * OPTIMAL_ROI_RADIUS_X) / OPTIMAL_ROI_STEP + 1) // 161
#define ROI_GRID_ROWS  ((2 * OPTIMAL_ROI_RADIUS_Y) / OPTIMAL_ROI_STEP + 1) // 151

typedef enum {
    DETECTOR_STATE_DOWN = 0,         // Target is lowered / retracted. Detection is LOCKED.
    DETECTOR_STATE_RISING,           // Target is swinging up. Mechanical settle window (850ms).
    DETECTOR_STATE_ESTABLISH_REF,    // Target is upright & stationary. Averaging 3 frames for reference.
    DETECTOR_STATE_ARMED,            // Stationary reference established. Continuous high-speed monitoring.
    DETECTOR_STATE_HIT_LOCKED        // Hit confirmed! Reverse command dispatched. Detection locked.
} detector_state_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} roi_pixel_t;

// Persistent reference and temporal grids in PSRAM
static roi_pixel_t *s_ref_grid = NULL;
static roi_pixel_t *s_prev_grid = NULL;
static uint16_t *s_accum_r = NULL;
static uint16_t *s_accum_g = NULL;
static uint16_t *s_accum_b = NULL;

static detector_state_t g_detector_state = DETECTOR_STATE_DOWN;
static bool s_ref_ready = false;
static bool s_byte_swap_needed = false;
static bool s_swap_calibrated = false;

static int64_t s_arm_settle_until_us = 0;
static int64_t s_hit_cooldown_until_us = 0;
static int s_ref_accum_frames = 0;
static int s_flush_frames = 0;
static int s_diag_frame_div = 0;

#define STANDALONE_NO_VIDEO_STREAM 1

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
        s_prev_grid = (roi_pixel_t *)heap_caps_calloc(1, grid_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_accum_r = (uint16_t *)heap_caps_calloc(1, accum_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_accum_g = (uint16_t *)heap_caps_calloc(1, accum_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_accum_b = (uint16_t *)heap_caps_calloc(1, accum_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        ESP_LOGI("EVENT_DETECTOR", "Detector memory allocated: %u KB in PSRAM",
                 (unsigned int)((grid_bytes * 2 + accum_bytes * 3) / 1024));
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
        if (strstr((const char *)rx_buf, "ARM") != NULL) {
            g_detector_state = DETECTOR_STATE_RISING;
            s_arm_settle_until_us = frame_start_us + 850000ULL; // 850ms mechanical settle window
            s_ref_ready = false;
            s_ref_accum_frames = 0;
            s_hit_cooldown_until_us = 0;
            ESP_LOGI("EVENT_DETECTOR", ">>> [ARM RECEIVED] Target rising. Settling for 850ms... <<<");
            return false;
        } else if (strstr((const char *)rx_buf, "DISARM") != NULL || strstr((const char *)rx_buf, "DOWN") != NULL) {
            g_detector_state = DETECTOR_STATE_DOWN;
            s_ref_ready = false;
            s_ref_accum_frames = 0;
            ESP_LOGI("EVENT_DETECTOR", ">>> [DISARM RECEIVED] Target down. Detection locked. <<<");
            return false;
        }
    }

    int64_t now_us = esp_timer_get_time();

    // 2. FSM State Management
    if (g_detector_state == DETECTOR_STATE_DOWN) {
        // Target is retracted: ZERO processing, ZERO false positives!
        return false;
    }

    if (g_detector_state == DETECTOR_STATE_HIT_LOCKED) {
        // Cooldown active after hit: prevent bouncing or double-triggering
        if (now_us > s_hit_cooldown_until_us) {
            g_detector_state = DETECTOR_STATE_DOWN;
            const char *clear_msg = "CLEAR,1\n";
            uart_write_bytes(BRIDGE_UART_NUM, clear_msg, strlen(clear_msg));
            ESP_LOGI("EVENT_DETECTOR", "Cooldown expired -> Target returned to DOWN");
        }
        return false;
    }

    if (g_detector_state == DETECTOR_STATE_RISING) {
        if (now_us < s_arm_settle_until_us) {
            // Target is still in mechanical motion/vibration: discard frames
            return false;
        }
        // Exactly 850ms elapsed: Target is now fully upright and stationary!
        s_flush_frames = 2; // Flush remaining stale frames from camera queue
        g_detector_state = DETECTOR_STATE_ESTABLISH_REF;
        s_ref_accum_frames = 0;
        memset(s_accum_r, 0, ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(uint16_t));
        memset(s_accum_g, 0, ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(uint16_t));
        memset(s_accum_b, 0, ROI_GRID_ROWS * ROI_GRID_COLS * sizeof(uint16_t));
        ESP_LOGI("EVENT_DETECTOR", "Target settled! Flushing stale frames & establishing reference...");
        return false;
    }

    // Auto-detect byte swap on the first upright frame if needed
    if (!s_swap_calibrated) {
        int disp_no_swap = 0, disp_swap = 0;
        for (int dy = -10; dy <= 10; dy += 2) {
            int row = (OPTIMAL_ROI_CENTER_Y + dy) * width;
            for (int dx = -10; dx <= 10; dx += 2) {
                uint16_t p = pixels[row + OPTIMAL_ROI_CENTER_X + dx];
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

    if (g_detector_state == DETECTOR_STATE_ESTABLISH_REF) {
        if (s_flush_frames > 0) {
            s_flush_frames--;
            return false;
        }

        // Accumulate 3 stationary frames for high-SNR reference baseline
        for (int gy = 0; gy < ROI_GRID_ROWS; gy++) {
            int y = (OPTIMAL_ROI_CENTER_Y - OPTIMAL_ROI_RADIUS_Y) + gy * OPTIMAL_ROI_STEP;
            int row_offset = y * width;
            for (int gx = 0; gx < ROI_GRID_COLS; gx++) {
                int x = (OPTIMAL_ROI_CENTER_X - OPTIMAL_ROI_RADIUS_X) + gx * OPTIMAL_ROI_STEP;
                uint16_t p = pixels[row_offset + x];
                int r, g, b;
                unpack_rgb565(p, s_byte_swap_needed, &r, &g, &b);
                int idx = gy * ROI_GRID_COLS + gx;
                s_accum_r[idx] += (uint16_t)r;
                s_accum_g[idx] += (uint16_t)g;
                s_accum_b[idx] += (uint16_t)b;
            }
        }
        s_ref_accum_frames++;

        if (s_ref_accum_frames >= 3) {
            // Compute average reference baseline and initialize prev_grid
            for (int gy = 0; gy < ROI_GRID_ROWS; gy++) {
                for (int gx = 0; gx < ROI_GRID_COLS; gx++) {
                    int idx = gy * ROI_GRID_COLS + gx;
                    s_ref_grid[idx].r = (uint8_t)(s_accum_r[idx] / 3);
                    s_ref_grid[idx].g = (uint8_t)(s_accum_g[idx] / 3);
                    s_ref_grid[idx].b = (uint8_t)(s_accum_b[idx] / 3);
                    s_prev_grid[idx] = s_ref_grid[idx];
                }
            }
            s_ref_ready = true;
            g_detector_state = DETECTOR_STATE_ARMED;
            const char *armed_msg = "ARMED,1\n";
            uart_write_bytes(BRIDGE_UART_NUM, armed_msg, strlen(armed_msg));
            ESP_LOGI("EVENT_DETECTOR", "✅ STABLE BLACK TARGET REFERENCE ESTABLISHED. SYSTEM ARMED!");
        }
        return false;
    }

    // 3. TARGET IS ARMED: Continuous High-Speed Multi-Signal Evaluation
    if (g_detector_state != DETECTOR_STATE_ARMED || !s_ref_ready) {
        return false;
    }

    uint32_t delta_ref_count = 0;
    uint32_t motion_prev_count = 0;
    uint32_t color_count = 0;
    uint32_t roi_points = 0;

    int min_x = 9999, max_x = -1;
    int min_y = 9999, max_y = -1;
    uint64_t sum_x = 0, sum_y = 0;

    // Scan optimal elliptical ROI
    // Ellipse test: (dx*dx * 22500) + (dy*dy * 25600) <= 576000000
    for (int gy = 0; gy < ROI_GRID_ROWS; gy++) {
        int y = (OPTIMAL_ROI_CENTER_Y - OPTIMAL_ROI_RADIUS_Y) + gy * OPTIMAL_ROI_STEP;
        int dy = y - OPTIMAL_ROI_CENTER_Y;
        int dy_term = dy * dy * 25600;
        int row_offset = y * width;
        int grid_row_idx = gy * ROI_GRID_COLS;

        for (int gx = 0; gx < ROI_GRID_COLS; gx++) {
            int x = (OPTIMAL_ROI_CENTER_X - OPTIMAL_ROI_RADIUS_X) + gx * OPTIMAL_ROI_STEP;
            int dx = x - OPTIMAL_ROI_CENTER_X;
            if ((dx * dx * 22500 + dy_term) > 576000000) {
                continue;
            }
            roi_points++;

            uint16_t p = pixels[row_offset + x];
            int r, g, b;
            unpack_rgb565(p, s_byte_swap_needed, &r, &g, &b);

            int idx = grid_row_idx + gx;

            // Signal 1: Reference Delta
            int ref_r = s_ref_grid[idx].r;
            int ref_g = s_ref_grid[idx].g;
            int ref_b = s_ref_grid[idx].b;
            int d_ref = abs(r - ref_r) + abs(g - ref_g) + abs(b - ref_b);

            if (d_ref >= 38) {
                delta_ref_count++;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                sum_x += (uint64_t)x;
                sum_y += (uint64_t)y;
            }

            // Signal 2: Temporal Motion Delta (Current vs Previous Frame)
            int prev_r = s_prev_grid[idx].r;
            int prev_g = s_prev_grid[idx].g;
            int prev_b = s_prev_grid[idx].b;
            int d_prev = abs(r - prev_r) + abs(g - prev_g) + abs(b - prev_b);
            if (d_prev >= 30) {
                motion_prev_count++;
            }

            // Signal 3: Orange / Yellow Color Presence
            // Nerf projectile stands out brightly against dark/black target
            if ((r + g) >= 155 && b <= 75) {
                // Yellow foam body: high red & green, low blue
                bool is_yellow = (r >= 85 && g >= 70 && r >= (b + 22) && g >= (b + 12));
                // Orange rubber tip: high red, moderate green, low blue
                bool is_orange = (r >= 95 && g >= 35 && r >= (g + 20) && r >= (b + 28));
                if (is_yellow || is_orange) {
                    color_count++;
                }
            }

            // Update previous grid for next frame
            s_prev_grid[idx].r = (uint8_t)r;
            s_prev_grid[idx].g = (uint8_t)g;
            s_prev_grid[idx].b = (uint8_t)b;
        }
    }

    int span_x = (delta_ref_count > 0) ? (max_x - min_x) : 0;
    int span_y = (delta_ref_count > 0) ? (max_y - min_y) : 0;
    float cx = (delta_ref_count > 0) ? ((float)sum_x / (float)delta_ref_count) : (float)OPTIMAL_ROI_CENTER_X;
    float cy = (delta_ref_count > 0) ? ((float)sum_y / (float)delta_ref_count) : (float)OPTIMAL_ROI_CENTER_Y;

    *out_x = cx;
    *out_y = cy;
    *out_delta = (int)delta_ref_count;
    *out_color = (int)color_count;
    *out_motion = (int)motion_prev_count;

    // Signal Checks
    bool sig_delta = (delta_ref_count >= 18);    // >= ~72 full pixels changed from baseline
    bool sig_color = (color_count >= 12);        // >= ~48 full pixels of orange/yellow
    bool sig_motion = (motion_prev_count >= 14);  // >= ~56 full pixels changed in last frame
    bool sig_spatial = (span_x <= 90 && span_y <= 90 && span_x >= 4 && span_y >= 4);

    bool hit_confirmed = false;
    int confidence = 0;

    // Decision Logic: Correlated Multi-Signal Confirmation
    if (sig_delta && sig_color && sig_spatial) {
        hit_confirmed = true;
        confidence = 100;
    } else if (sig_delta && sig_motion && sig_spatial && delta_ref_count >= 30) {
        hit_confirmed = true;
        confidence = 90;
    } else if (color_count >= 35 && sig_delta) {
        hit_confirmed = true;
        confidence = 95;
    }

    float latency_ms = (float)(esp_timer_get_time() - frame_start_us) / 1000.0f;

    if (hit_confirmed) {
        // --- 1. IMMEDIATELY DISPATCH REVERSE COMMAND OVER UART ---
        char hit_cmd[64];
        int hlen = snprintf(hit_cmd, sizeof(hit_cmd), "HIT,1,%.0f,%.0f,%lu\n",
                            cx, cy, (unsigned long)(delta_ref_count * 4));
        uart_write_bytes(BRIDGE_UART_NUM, hit_cmd, hlen);

        // 2. Dispatch detailed diagnostic telemetry
        char diag_msg[96];
        int dlen = snprintf(diag_msg, sizeof(diag_msg), "DIAG,HIT,%lu,%lu,%lu,%d,%d,%d,%.1f\n",
                            (unsigned long)delta_ref_count, (unsigned long)color_count,
                            (unsigned long)motion_prev_count, span_x, span_y, confidence, latency_ms);
        uart_write_bytes(BRIDGE_UART_NUM, diag_msg, dlen);

        // 3. Immediately lock state to prevent double-triggering
        g_detector_state = DETECTOR_STATE_HIT_LOCKED;
        s_hit_cooldown_until_us = esp_timer_get_time() + 3000000ULL;

        ESP_LOGI("EVENT_DETECTOR", "🎯 >>> HIT CONFIRMED! <<< Pos:(%.0f,%.0f) Span:%dx%d Delta:%lu Color:%lu Motion:%lu Conf:%d%% Latency:%.2fms -> SERVO REVERSED!",
                 cx, cy, span_x, span_y, (unsigned long)delta_ref_count, (unsigned long)color_count,
                 (unsigned long)motion_prev_count, confidence, latency_ms);
        return true;
    } else {
        // Periodic diagnostic telemetry every 6 frames (~100ms)
        if (++s_diag_frame_div >= 6) {
            s_diag_frame_div = 0;
            char diag_msg[96];
            int dlen = snprintf(diag_msg, sizeof(diag_msg), "DIAG,ARMED,%lu,%lu,%lu,%d,%d,0,0.0\n",
                                (unsigned long)delta_ref_count, (unsigned long)color_count,
                                (unsigned long)motion_prev_count, span_x, span_y);
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
            
            #if !STANDALONE_NO_VIDEO_STREAM
            // Pace the frame stream to target FPS only when streaming video
            int64_t now = esp_timer_get_time();
            if (now - last_transmit_time < frame_interval_us) {
                ioctl(g_video_fd, VIDIOC_QBUF, &buf);
                continue;
            }
            last_transmit_time = now;
            #endif

            stat_captured++;

            uint8_t *frame_ptr = g_buffers[buf.index];
            uint32_t out_len = buf.bytesused;

            if (frame_ptr != NULL && out_len > 0) {
                // --- HIGH-SPEED DETERMINISTIC EVENT DETECTION PIPELINE ---
                uint16_t *pixels = (uint16_t *)frame_ptr;
                float hit_x = 0.0f, hit_y = 0.0f;
                int hit_delta = 0, hit_color = 0, hit_motion = 0;
                process_target_event_frame(pixels, 800, 800, &hit_x, &hit_y, &hit_delta, &hit_color, &hit_motion);

                #if !STANDALONE_NO_VIDEO_STREAM
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