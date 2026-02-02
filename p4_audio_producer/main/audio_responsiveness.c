/*
 * Responsiveness measurement harness — Phase 1 implementation.
 * Measures reaction time: publish_time - capture_time of the first frame that detected the event.
 * Uses hop-start (t_capture_us - 8ms) so latency is not inflated by end-of-hop timestamp.
 * Cooldown (200 ms) prevents one drum hit from counting as multiple transients.
 */
#include "audio_responsiveness.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "responsiveness";

static responsiveness_state_t s_state = {0};

/* Comparison function for qsort */
static int uint32_cmp(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a;
    uint32_t y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

void responsiveness_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.prev_onset_strength = 0.f;
    s_state.prev_vu_peak = 0.f;
    s_state.cooldown_until_us = 0;
    s_state.last_stats_log_us = esp_timer_get_time();
    s_state.stats_interval_ms = 1000;  /* Log stats every 1 second */
    
    ESP_LOGI(TAG, "Responsiveness harness initialized (onset_threshold=%.3f, vu_threshold=%.3f, cooldown=%lu ms)",
             RESPONSIVENESS_ONSET_THRESHOLD, RESPONSIVENESS_VU_THRESHOLD,
             (unsigned long)(RESPONSIVENESS_COOLDOWN_US / 1000));
}

void responsiveness_feed_frame(const AudioFrame *f, uint64_t t_now_us)
{
    if (!f) return;
    
    /* Cooldown: ignore new transients until debounce window has passed */
    if (t_now_us < s_state.cooldown_until_us) {
        s_state.prev_onset_strength = f->onset_strength;
        s_state.prev_vu_peak = f->vu_peak;
        return;
    }
    
    /* Detect transient: onset_strength or vu_peak crosses threshold upward */
    bool onset_crossed = (f->onset_strength >= RESPONSIVENESS_ONSET_THRESHOLD) &&
                         (s_state.prev_onset_strength < RESPONSIVENESS_ONSET_THRESHOLD);
    
    bool vu_crossed = (f->vu_peak >= RESPONSIVENESS_VU_THRESHOLD) &&
                      (s_state.prev_vu_peak < RESPONSIVENESS_VU_THRESHOLD);
    
    if (onset_crossed || vu_crossed) {
        /* Reaction time = publish_time - hop_start of the frame that detected the event.
         * t_capture_us is end-of-hop, so hop_start = t_capture_us - 8ms. */
        uint64_t t_hop_start_us = f->t_capture_us - RESPONSIVENESS_HOP_US;
        uint64_t t_pub = t_now_us;
        uint64_t t_vis = t_now_us;  /* Stub: visual commit same as publish for now */
        
        if (t_pub > t_hop_start_us) {
            uint32_t onset_lat_us = (uint32_t)(t_pub - t_hop_start_us);
            uint32_t e2e_lat_us = (uint32_t)(t_vis - t_hop_start_us);
            
            /* Store immediately (one sample per transient) */
            s_state.onset_latency_us[s_state.latency_idx % RESPONSIVENESS_HISTORY_LEN] = onset_lat_us;
            s_state.e2e_latency_us[s_state.latency_idx % RESPONSIVENESS_HISTORY_LEN] = e2e_lat_us;
            s_state.latency_idx++;
            if (s_state.latency_count < RESPONSIVENESS_HISTORY_LEN) {
                s_state.latency_count++;
            }
            
            s_state.transient_count++;
            
            ESP_LOGD(TAG, "Transient: seq=%lu onset_lat=%lu us e2e_lat=%lu us (hop_start used)",
                     (unsigned long)f->seq, (unsigned long)onset_lat_us, (unsigned long)e2e_lat_us);
        }
        
        /* Debounce: ignore further transients for 200 ms */
        s_state.cooldown_until_us = t_now_us + RESPONSIVENESS_COOLDOWN_US;
    }
    
    /* Update previous values */
    s_state.prev_onset_strength = f->onset_strength;
    s_state.prev_vu_peak = f->vu_peak;
}

void responsiveness_log_stats(void)
{
    uint64_t now_us = esp_timer_get_time();
    uint64_t elapsed_ms = (now_us - s_state.last_stats_log_us) / 1000;
    
    if (elapsed_ms < s_state.stats_interval_ms) {
        return;  /* Not time to log yet */
    }
    
    s_state.last_stats_log_us = now_us;
    
    if (s_state.latency_count == 0) {
        ESP_LOGI(TAG, "Responsiveness stats: no transients detected yet");
        return;
    }
    
    /* Copy latency arrays for sorting */
    uint32_t onset_sorted[RESPONSIVENESS_HISTORY_LEN];
    uint32_t e2e_sorted[RESPONSIVENESS_HISTORY_LEN];
    unsigned n = (s_state.latency_count < RESPONSIVENESS_HISTORY_LEN) ? 
                 s_state.latency_count : RESPONSIVENESS_HISTORY_LEN;
    
    memcpy(onset_sorted, s_state.onset_latency_us, n * sizeof(uint32_t));
    memcpy(e2e_sorted, s_state.e2e_latency_us, n * sizeof(uint32_t));
    
    qsort(onset_sorted, n, sizeof(uint32_t), uint32_cmp);
    qsort(e2e_sorted, n, sizeof(uint32_t), uint32_cmp);
    
    /* Compute statistics */
    uint64_t onset_sum = 0, e2e_sum = 0;
    for (unsigned i = 0; i < n; i++) {
        onset_sum += onset_sorted[i];
        e2e_sum += e2e_sorted[i];
    }
    
    uint32_t onset_median = onset_sorted[n / 2];
    uint32_t onset_p95 = onset_sorted[(n * 95) / 100];
    uint32_t onset_avg = (uint32_t)(onset_sum / n);
    uint32_t onset_max = onset_sorted[n - 1];
    
    uint32_t e2e_median = e2e_sorted[n / 2];
    uint32_t e2e_p95 = e2e_sorted[(n * 95) / 100];
    uint32_t e2e_avg = (uint32_t)(e2e_sum / n);
    uint32_t e2e_max = e2e_sorted[n - 1];
    
    /* Log statistics */
    ESP_LOGI(TAG, "Responsiveness stats (n=%u transients):", n);
    ESP_LOGI(TAG, "  Onset latency: median=%lu avg=%lu p95=%lu max=%lu us (target ≤16000 us)",
             (unsigned long)onset_median, (unsigned long)onset_avg,
             (unsigned long)onset_p95, (unsigned long)onset_max);
    ESP_LOGI(TAG, "  E2E latency:   median=%lu avg=%lu p95=%lu max=%lu us (target ≤24000 us)",
             (unsigned long)e2e_median, (unsigned long)e2e_avg,
             (unsigned long)e2e_p95, (unsigned long)e2e_max);
    
    /* Gate validation */
    if (onset_p95 > 16000) {
        ESP_LOGW(TAG, "  ⚠️  Onset latency p95 exceeds gate: %lu us > 16000 us",
                 (unsigned long)onset_p95);
    }
    if (e2e_p95 > 24000) {
        ESP_LOGW(TAG, "  ⚠️  E2E latency p95 exceeds gate: %lu us > 24000 us",
                 (unsigned long)e2e_p95);
    }
    
    /* Reset counters for next interval */
    s_state.transient_count = 0;
}

void responsiveness_get_stats(uint32_t *transient_count_out,
                              uint32_t *onset_median_us_out,
                              uint32_t *onset_p95_us_out,
                              uint32_t *e2e_median_us_out,
                              uint32_t *e2e_p95_us_out)
{
    if (!transient_count_out || !onset_median_us_out || !onset_p95_us_out ||
        !e2e_median_us_out || !e2e_p95_us_out) {
        return;
    }
    
    *transient_count_out = s_state.transient_count;
    
    if (s_state.latency_count == 0) {
        *onset_median_us_out = 0;
        *onset_p95_us_out = 0;
        *e2e_median_us_out = 0;
        *e2e_p95_us_out = 0;
        return;
    }
    
    /* Copy and sort */
    uint32_t onset_sorted[RESPONSIVENESS_HISTORY_LEN];
    uint32_t e2e_sorted[RESPONSIVENESS_HISTORY_LEN];
    unsigned n = (s_state.latency_count < RESPONSIVENESS_HISTORY_LEN) ?
                 s_state.latency_count : RESPONSIVENESS_HISTORY_LEN;
    
    memcpy(onset_sorted, s_state.onset_latency_us, n * sizeof(uint32_t));
    memcpy(e2e_sorted, s_state.e2e_latency_us, n * sizeof(uint32_t));
    
    qsort(onset_sorted, n, sizeof(uint32_t), uint32_cmp);
    qsort(e2e_sorted, n, sizeof(uint32_t), uint32_cmp);
    
    *onset_median_us_out = onset_sorted[n / 2];
    *onset_p95_us_out = onset_sorted[(n * 95) / 100];
    *e2e_median_us_out = e2e_sorted[n / 2];
    *e2e_p95_us_out = e2e_sorted[(n * 95) / 100];
}
