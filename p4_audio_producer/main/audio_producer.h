/*
 * Two-lane audio producer: capture task, fast lane, slow lane, AudioFrame publish.
 * Authority: docs/DEPARTMENTAL_AUDIO_PRODUCER_SPEC.md.
 */
#ifndef P4_AUDIO_PRODUCER_AUDIO_PRODUCER_H
#define P4_AUDIO_PRODUCER_AUDIO_PRODUCER_H

#include "audio_frame.h"
#include "audio_config.h"
#include "freertos/task.h"  /* For TaskHandle_t */
#include <stdbool.h>

void audio_producer_start(bool enable_slow_lane);

/* DSP self-test: verifies esp-dsp FFT functionality (called once at boot).
 * Returns: true if PASS, false if FAIL.
 */
bool dsp_selftest(void);

/* Latest published frame (read-only for consumers). */
void audio_producer_get_latest(AudioFrame *out);

/* Call periodically (e.g. every 5 s) to log latency stats. */
void audio_producer_log_latency(void);

/* Debug summary: logs audio pipeline state (called every 5s automatically). */
void audio_producer_log_debug_summary(void);

/* Instrumentation counters. */
extern volatile uint32_t audio_capture_overruns;
extern volatile uint32_t audio_fast_lane_overruns;
extern volatile uint32_t audio_slow_lane_overruns;
extern volatile uint32_t audio_slow_lane_stale_count;
extern volatile uint32_t audio_published_seq;
extern volatile uint64_t audio_t_capture_us;
extern volatile uint32_t audio_max_fast_lane_us;
extern volatile uint32_t audio_max_slow_lane_us;
extern volatile uint64_t audio_t_publish_us;
extern volatile uint64_t audio_t_visual_commit_us;
/* Hop rate diagnostics */
extern volatile uint32_t audio_capture_reads_count;
extern volatile uint32_t audio_fast_lane_wakeups_count;
extern volatile uint64_t audio_diag_start_time_us;
/* Task handles for stack monitoring */
extern TaskHandle_t s_capture_handle;
extern TaskHandle_t s_fast_lane_handle;
extern TaskHandle_t s_slow_lane_handle;

#endif /* P4_AUDIO_PRODUCER_AUDIO_PRODUCER_H */
