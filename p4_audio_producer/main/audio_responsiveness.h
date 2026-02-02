/*
 * Responsiveness measurement harness — Phase 1: reaction-time latency metrics.
 * Measures: publish_time - capture_time of the first frame that detected the event.
 * Uses hop-start (t_capture_us - 8ms) so latency is not inflated by end-of-hop timestamp.
 * Target: ≤16 ms onset response, ≤24 ms E2E latency (gate).
 */
#ifndef P4_AUDIO_PRODUCER_AUDIO_RESPONSIVENESS_H
#define P4_AUDIO_PRODUCER_AUDIO_RESPONSIVENESS_H

#include "IAudioCapture.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Impulse test mode: detects transients and measures reaction time (first-frame latency) */
#define RESPONSIVENESS_HISTORY_LEN 64  /* Latency history for stats */
#define RESPONSIVENESS_ONSET_THRESHOLD 0.1f  /* Onset strength threshold for transient detection */
#define RESPONSIVENESS_VU_THRESHOLD 0.05f  /* VU peak threshold (backup detection) */
#define RESPONSIVENESS_HOP_US 8000u  /* 128 samples @ 16 kHz = 8 ms (t_capture_us is end-of-hop) */
#define RESPONSIVENESS_COOLDOWN_US 200000u  /* 200 ms debounce so one hit doesn't generate many transients */

/* Latency event: captures timing chain for one transient */
typedef struct {
    uint64_t t_transient_us;      /* When transient detected (onset crosses threshold) */
    uint64_t t_onset_peak_us;     /* When onset_strength reaches peak */
    uint64_t t_visual_commit_us;  /* When visual commit happens (stub: same as publish for now) */
    float onset_peak_value;       /* Peak onset strength value */
    float vu_peak_value;          /* Peak VU at transient */
    uint32_t seq;                 /* Frame sequence number */
} responsiveness_event_t;

/* Responsiveness state */
typedef struct {
    /* Transient detection state */
    float prev_onset_strength;
    float prev_vu_peak;
    uint64_t cooldown_until_us;  /* Ignore new transients until this time (debounce) */
    
    /* Latency history */
    uint32_t onset_latency_us[RESPONSIVENESS_HISTORY_LEN];
    uint32_t e2e_latency_us[RESPONSIVENESS_HISTORY_LEN];
    unsigned latency_idx;
    unsigned latency_count;
    
    /* Statistics */
    uint64_t last_stats_log_us;
    uint32_t transient_count;
    uint32_t stats_interval_ms;
} responsiveness_state_t;

/**
 * Initialize responsiveness measurement harness.
 */
void responsiveness_init(void);

/**
 * Feed frame to responsiveness tracker (called from fast_lane_task after publish).
 * Detects transients and records reaction time (first-frame latency) immediately; 200 ms cooldown debounces.
 */
void responsiveness_feed_frame(const AudioFrame *f, uint64_t t_now_us);

/**
 * Log responsiveness statistics (median, p95 latency).
 * Call periodically (e.g., every 1 second).
 */
void responsiveness_log_stats(void);

/**
 * Get current responsiveness statistics (for external monitoring).
 */
void responsiveness_get_stats(uint32_t *transient_count_out,
                              uint32_t *onset_median_us_out,
                              uint32_t *onset_p95_us_out,
                              uint32_t *e2e_median_us_out,
                              uint32_t *e2e_p95_us_out);

#ifdef __cplusplus
}
#endif

#endif /* P4_AUDIO_PRODUCER_AUDIO_RESPONSIVENESS_H */
