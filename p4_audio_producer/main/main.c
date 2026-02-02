/*
 * P4 Audio Producer — Department 0: boot banner + minimal bring-up.
 * ESP-IDF v5.5.2, target ESP32-P4.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"  /* For esp_timer_get_time() */
#include "nvs_flash.h"
#include "audio_config.h"
#include "audio_producer.h"
#include "audio_responsiveness.h"
#include "tempo_stabilizer.h"
#include "IAudioCapture.h"
#include <stdbool.h>

extern void audio_config_test_assert(void);

static const char *TAG = "p4_audio";

/**
 * Delay helper: ensures delay never becomes 0 ticks (defensive programming).
 * With 1000 Hz tick rate, pdMS_TO_TICKS(1500) = 1500, but this protects against
 * future tick rate changes.
 */
static inline TickType_t delay_ms_min1(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    return (t == 0) ? 1 : t;  // never allow a 0-tick "delay"
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Compile-time test: if audio_config.h is out of sync, _Static_assert in header fails. */
    audio_config_test_assert();

    /* Boot banner — single source of truth from audio_config.h */
    ESP_LOGI(TAG, "P4 Audio Producer | Fs=%u Hz Hop=%u samples (%u ms) | Spec %s",
             (unsigned)AUDIO_FS, (unsigned)AUDIO_HOP_SIZE, (unsigned)AUDIO_HOP_MS,
             AUDIO_SPEC_VERSION_STR);

    /* Log ESP-IDF version for toolchain pinning */
    ESP_LOGI(TAG, "ESP-IDF: %s", esp_get_idf_version());

    /* Log FreeRTOS tick configuration for watchdog debugging */
    ESP_LOGI(TAG, "FreeRTOS tick: %d Hz (tick=%d ms)", CONFIG_FREERTOS_HZ, portTICK_PERIOD_MS);

    /* Phase 2B: DSP self-test (verifies esp-dsp FFT before slow_lane uses it) */
    bool dsp_ok = dsp_selftest();

    /* Dept 2: start audio producer (capture + fast/slow lane + AudioFrame). */
    audio_producer_start(dsp_ok);

    /* Heartbeat loop — log counters every ~10 s, latency stats every ~5 s, debug every 5s. */
    uint32_t heartbeat = 0;
    for (;;) {
        vTaskDelay(delay_ms_min1(1500));
        heartbeat++;
        if (heartbeat % 7 == 0) {
            /* Calculate hop rates */
            uint64_t now_us = esp_timer_get_time();
            uint64_t elapsed_us = (now_us > audio_diag_start_time_us) ? (now_us - audio_diag_start_time_us) : 1;
            float capture_rate = (audio_capture_reads_count * 1000000.0f) / elapsed_us;
            float fast_rate = (audio_fast_lane_wakeups_count * 1000000.0f) / elapsed_us;
            
            ESP_LOGI(TAG, "P4 heartbeat %lu | capture_overruns=%lu fast_overruns=%lu slow_overruns=%lu seq=%lu max_fast_us=%lu max_slow_us=%lu",
                     (unsigned long)heartbeat, (unsigned long)audio_capture_overruns,
                     (unsigned long)audio_fast_lane_overruns, (unsigned long)audio_slow_lane_overruns,
                     (unsigned long)audio_published_seq, (unsigned long)audio_max_fast_lane_us,
                     (unsigned long)audio_max_slow_lane_us);
            ESP_LOGI(TAG, "Hop rates: capture=%.1f Hz fast=%.1f Hz (expected ~125 Hz)", capture_rate, fast_rate);
            
            /* Gate: hop rates should be ~125 Hz (16kHz / 128 samples) */
            if (capture_rate > 200.0f || fast_rate > 200.0f) {
                ESP_LOGW(TAG, "WARNING: Hop rate too high (spinning?) capture=%.1f fast=%.1f", capture_rate, fast_rate);
            }
            
            /* Stack high-water marks (for debugging stack overflow) */
            extern TaskHandle_t s_capture_handle;
            extern TaskHandle_t s_fast_lane_handle;
            extern TaskHandle_t s_slow_lane_handle;
            if (s_capture_handle) {
                UBaseType_t capture_stack = uxTaskGetStackHighWaterMark(s_capture_handle);
                ESP_LOGI(TAG, "Stack high-water: capture=%u words", (unsigned)capture_stack);
            }
            if (s_fast_lane_handle) {
                UBaseType_t fast_stack = uxTaskGetStackHighWaterMark(s_fast_lane_handle);
                ESP_LOGI(TAG, "Stack high-water: fast_lane=%u words", (unsigned)fast_stack);
            }
            if (s_slow_lane_handle) {
                UBaseType_t slow_stack = uxTaskGetStackHighWaterMark(s_slow_lane_handle);
                ESP_LOGI(TAG, "Stack high-water: slow_lane=%u words", (unsigned)slow_stack);
            }
        }
        if (heartbeat % 4 == 0) {
            audio_producer_log_latency();
            
            /* Latency gate validation */
            uint64_t t_cap, t_pub, t_vis;
            IAudioCapture_get_latency_us(&t_cap, &t_pub, &t_vis);
            if (t_pub > t_cap) {
                uint32_t audio_lat_us = (uint32_t)(t_pub - t_cap);
                if (audio_lat_us > 16000) {  /* 16ms gate */
                    ESP_LOGW(TAG, "Audio latency exceeded gate: %lu us (gate=16000 us)", (unsigned long)audio_lat_us);
                }
            }
            if (t_vis > t_cap) {
                uint32_t e2e_lat_us = (uint32_t)(t_vis - t_cap);
                if (e2e_lat_us > 24000) {  /* 24ms gate */
                    ESP_LOGW(TAG, "E2E latency exceeded gate: %lu us (gate=24000 us)", (unsigned long)e2e_lat_us);
                }
            }
        }
        /* Debug log every 5 seconds (heartbeat is 1.5s, so 4 heartbeats = 6s, use 3 heartbeats = 4.5s) */
        if (heartbeat % 3 == 0 && heartbeat > 0) {
            audio_producer_log_debug_summary();
        }
        
        /* Phase 1: Responsiveness stats logging (internal timing: logs every ~1 second) */
        responsiveness_log_stats();

        /* Tempo stabilizer: 1 Hz debug line (tempo_raw, tempo_out, conf, mode) */
        tempo_stabilizer_log_if_due();
        
        if (heartbeat % 7 != 0 && heartbeat % 4 != 0 && heartbeat % 3 != 0) {
            ESP_LOGI(TAG, "P4 heartbeat %lu", (unsigned long)heartbeat);
        }
    }
}
