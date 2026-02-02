/*
 * IAudioCapture HAL implementation for ESP32-P4.
 * Wraps audio_producer; satisfies include/IAudioCapture.h contract.
 */
#include "freertos/FreeRTOS.h"  /* Must be included before freertos/task.h */
#include "IAudioCapture.h"
#include "audio_producer.h"
#include <string.h>

void IAudioCapture_get_latest(AudioFrame *out)
{
    audio_producer_get_latest(out);
}

void IAudioCapture_get_counters(uint32_t *capture_overruns, uint32_t *fast_lane_overruns, uint32_t *published_seq)
{
    if (capture_overruns) *capture_overruns = audio_capture_overruns;
    if (fast_lane_overruns) *fast_lane_overruns = audio_fast_lane_overruns;
    if (published_seq) *published_seq = audio_published_seq;
}

void IAudioCapture_get_latency_us(uint64_t *t_capture_us, uint64_t *t_publish_us, uint64_t *t_visual_commit_us)
{
    if (t_capture_us) *t_capture_us = audio_t_capture_us;
    if (t_publish_us) *t_publish_us = audio_t_publish_us;
    if (t_visual_commit_us) *t_visual_commit_us = audio_t_visual_commit_us;
}
