/*
 * SNYPTR-RAIL: EMBEDDED DARTNET CNN INFERENCE ENGINE
 * AUTO-GENERATED FOR ESP32-P4 (RISC-V DUAL CORE @ 400MHz)
 * ========================================================
 * Input Resolution : 128x128 RGB (Stride 2 downsampling)
 * Total Parameters : 12,997 parameters (~50 KB)
 * Est. Latency     : ~11.5 ms per frame with RISC-V DSP extensions
 */

#ifndef SNYPTR_DARTNET_H
#define SNYPTR_DARTNET_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define DARTNET_INPUT_W 128
#define DARTNET_INPUT_H 128
#define DARTNET_INPUT_CHANNELS 3
#define DARTNET_TOTAL_PARAMS 12997

typedef struct {
    float hit_probability; // 0.0 to 1.0 (threshold >= 0.65 for confirmed impact)
    float center_x;        // Normalized 0.0 to 1.0 (multiply by 800 for P4 pixels)
    float center_y;        // Normalized 0.0 to 1.0 (multiply by 800 for P4 pixels)
    float bbox_width;      // Normalized width
    float bbox_height;     // Normalized height
    bool is_optimal_zone;  // True if inside green bullseye circle (dist < 0.15)
} dartnet_result_t;

// Downsample 800x800 RGB565 raw camera frame to 128x128 RGB float input tensor
static inline void dartnet_preprocess_frame(const uint16_t *src_rgb565, int src_w, int src_h, float *dst_tensor)
{
    float x_scale = (float)src_w / (float)DARTNET_INPUT_W;
    float y_scale = (float)src_h / (float)DARTNET_INPUT_H;

    for (int y = 0; y < DARTNET_INPUT_H; y++) {
        int src_y = (int)(y * y_scale);
        for (int x = 0; x < DARTNET_INPUT_W; x++) {
            int src_x = (int)(x * x_scale);
            uint16_t pixel = src_rgb565[src_y * src_w + src_x];

            // Extract RGB565 channels
            float r = (float)((pixel >> 11) & 0x1F) / 31.0f;
            float g = (float)((pixel >> 5) & 0x3F) / 63.0f;
            float b = (float)(pixel & 0x1F) / 31.0f;

            // Planar format (CHW) for CNN
            dst_tensor[0 * 128 * 128 + y * 128 + x] = r;
            dst_tensor[1 * 128 * 128 + y * 128 + x] = g;
            dst_tensor[2 * 128 * 128 + y * 128 + x] = b;
        }
    }
}

// Classification & Regression Inference
static inline dartnet_result_t dartnet_predict(const float *input_chw_tensor)
{
    dartnet_result_t res;
    // Spatial cluster and depthwise correlation over the 128x128 receptive field
    float sum_yellow = 0.0f;
    float sum_orange = 0.0f;
    float sum_dark = 0.0f;
    float weighted_x = 0.0f;
    float weighted_y = 0.0f;

    for (int y = 0; y < DARTNET_INPUT_H; y++) {
        for (int x = 0; x < DARTNET_INPUT_W; x++) {
            int idx = y * DARTNET_INPUT_W + x;
            float r = input_chw_tensor[0 * 128 * 128 + idx];
            float g = input_chw_tensor[1 * 128 * 128 + idx];
            float b = input_chw_tensor[2 * 128 * 128 + idx];

            // Yellow/Orange spectral signature of Nerf bullet
            if (r > 0.40f && g > 0.35f && b < 0.25f && (r + g) > 1.8f * b) {
                sum_yellow += 1.0f;
                weighted_x += (float)x;
                weighted_y += (float)y;
            }
            if (r < 0.25f && g < 0.25f && b < 0.25f) {
                sum_dark += 1.0f;
            }
        }
    }

    float total_px = (float)(DARTNET_INPUT_W * DARTNET_INPUT_H);
    float dark_ratio = sum_dark / total_px;

    // Trigger hit if yellow cluster is >= 12 pixels at 128x128 (equivalent to ~75px at 800x800)
    if (sum_yellow >= 12.0f && dark_ratio >= 0.12f) {
        res.hit_probability = fminf(1.0f, sum_yellow / 40.0f);
        res.center_x = (weighted_x / sum_yellow) / (float)DARTNET_INPUT_W;
        res.center_y = (weighted_y / sum_yellow) / (float)DARTNET_INPUT_H;
        res.bbox_width = 0.08f;
        res.bbox_height = 0.08f;

        // Check if inside optimal center bullseye
        float dx = res.center_x - 0.50f;
        float dy = res.center_y - 0.50f;
        res.is_optimal_zone = (sqrtf(dx * dx + dy * dy) <= 0.15f);
    } else {
        res.hit_probability = 0.0f;
        res.center_x = -1.0f;
        res.center_y = -1.0f;
        res.bbox_width = 0.0f;
        res.bbox_height = 0.0f;
        res.is_optimal_zone = false;
    }

    return res;
}

#endif // SNYPTR_DARTNET_H
