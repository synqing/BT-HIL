/*
 * IAudioCapture HAL — contract for audio producer.
 * Authority: docs/DEPARTMENTAL_AUDIO_PRODUCER_SPEC.md §2.
 * Implementations: p4_audio_producer (ESP32-P4).
 */
#ifndef EMOTISCOPE_IAUDIOCAPTURE_H
#define EMOTISCOPE_IAUDIOCAPTURE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_FRAME_VERSION  0x00010000u
#define AUDIO_FLAG_OVERFLOW          (1u << 0)
#define AUDIO_FLAG_UNDERRUN         (1u << 1)
#define AUDIO_FLAG_CLIPPED          (1u << 2)
#define AUDIO_FLAG_NOISE_FLOOR_HIGH (1u << 3)
#define AUDIO_FLAG_TEMPO_UNCERTAIN  (1u << 4)
#define AUDIO_FLAG_SLOW_LANE_STALE  (1u << 5)

typedef struct {
    uint32_t version;
    uint32_t seq;
    uint64_t t_capture_us;
    uint32_t flags;
    float quality;
    float vu_rms;
    float vu_peak;
    float noise_floor;
    float crest_factor;
    float onset_strength;
    float band_energy[8];
    float goertzel_bins[64];
    float chroma[12];
    float chroma_conf;
    float tempo_bpm;
    float tempo_conf;
    float beat_phase_0_1;
    float beat_conf;
    float centroid;
    float rolloff;
    float flatness;
} AudioFrame;

void IAudioCapture_get_latest(AudioFrame *out);

void IAudioCapture_get_counters(uint32_t *capture_overruns, uint32_t *fast_lane_overruns, uint32_t *published_seq);

void IAudioCapture_get_latency_us(uint64_t *t_capture_us, uint64_t *t_publish_us, uint64_t *t_visual_commit_us);

#ifdef __cplusplus
}
#endif

#endif /* EMOTISCOPE_IAUDIOCAPTURE_H */
