#ifndef HIL_CAPTURE_H
#define HIL_CAPTURE_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "global_defines.h"

struct hil_capture_state_t {
    // Audio DSP captures
    float spectrogram_capture[NUM_FREQS];              // Goertzel magnitudes
    float spectrogram_smooth_capture[NUM_FREQS];       // Smoothed magnitudes
    float chromagram_capture[12];                      // Pitch class energy
    
    float vu_level_capture;                            // Loudness
    float vu_max_capture;
    float vu_floor_capture;
    
    float novelty_curve_capture[NOVELTY_HISTORY_LENGTH];          // Spectral flux history
    float novelty_curve_normalized_capture[NOVELTY_HISTORY_LENGTH];
    
    float tempi_magnitude_capture[NUM_TEMPI];          // BPM detection
    float tempi_phase_capture[NUM_TEMPI];
    float tempi_beat_capture[NUM_TEMPI];

    // Raw I2S samples (Task 1.6) - captured every Nth frame to reduce data volume
    float sample_history_capture[SAMPLE_HISTORY_LENGTH];
    uint32_t sample_history_capture_counter;

    volatile uint32_t cpu_seq;
    volatile uint32_t gpu_seq;
};

extern hil_capture_state_t* hil_capture_state;
extern volatile bool hil_monitoring_active;

inline void hil_capture_cpu_write_begin() {
    if (!hil_capture_state) {
        return;
    }
    __atomic_fetch_add(&hil_capture_state->cpu_seq, 1, __ATOMIC_ACQ_REL);
}

inline void hil_capture_cpu_write_end() {
    if (!hil_capture_state) {
        return;
    }
    __atomic_fetch_add(&hil_capture_state->cpu_seq, 1, __ATOMIC_RELEASE);
}

inline void hil_capture_gpu_write_begin() {
    if (!hil_capture_state) {
        return;
    }
    __atomic_fetch_add(&hil_capture_state->gpu_seq, 1, __ATOMIC_ACQ_REL);
}

inline void hil_capture_gpu_write_end() {
    if (!hil_capture_state) {
        return;
    }
    __atomic_fetch_add(&hil_capture_state->gpu_seq, 1, __ATOMIC_RELEASE);
}

struct hil_capture_cpu_snapshot_t {
    float vu_level;
    float vu_max;
    float vu_floor;
    float novelty_norm_0_15[16];
    float spectrogram_0_15[16];
    float tempi_magnitude[NUM_TEMPI];
};

inline bool hil_capture_read_cpu_snapshot(hil_capture_cpu_snapshot_t* out) {
    if (!hil_capture_state || !out) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        uint32_t seq_a = __atomic_load_n(&hil_capture_state->cpu_seq, __ATOMIC_ACQUIRE);
        if (seq_a & 1U) {
            continue;
        }

        out->vu_level = hil_capture_state->vu_level_capture;
        out->vu_max = hil_capture_state->vu_max_capture;
        out->vu_floor = hil_capture_state->vu_floor_capture;

        memcpy(out->novelty_norm_0_15, hil_capture_state->novelty_curve_normalized_capture, sizeof(float) * 16);
        memcpy(out->spectrogram_0_15, hil_capture_state->spectrogram_capture, sizeof(float) * 16);
        memcpy(out->tempi_magnitude, hil_capture_state->tempi_magnitude_capture, sizeof(float) * NUM_TEMPI);

        uint32_t seq_b = __atomic_load_n(&hil_capture_state->cpu_seq, __ATOMIC_ACQUIRE);
        if (seq_a == seq_b && ((seq_b & 1U) == 0U)) {
            return true;
        }
    }

    return false;
}

#endif
