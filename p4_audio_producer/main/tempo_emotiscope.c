/*
 * Emotiscope-parity tempo: 1024 novelty, 96-bin Goertzel bank (48–144 BPM).
 * Authority: v1.1_build/tempo.h, global_defines.h.
 */
#include "tempo_emotiscope.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define BEAT_SHIFT_PERCENT 0.16f

typedef struct {
    float target_tempo_hz;
    float coeff;
    float sine;
    float cosine;
    float window_step;
    float phase;
    uint32_t block_size;
    float magnitude;
    float magnitude_full_scale;
} p4_tempo_bin_t;

static float s_window_lookup[4096];
static int s_window_init = 0;

static float s_novelty_curve[P4_NOVELTY_HISTORY_LENGTH];
static float s_novelty_curve_normalized[P4_NOVELTY_HISTORY_LENGTH];
static float s_vu_curve[P4_NOVELTY_HISTORY_LENGTH];
static float s_vu_curve_normalized[P4_NOVELTY_HISTORY_LENGTH];

static float s_tempi_bpm_hz[P4_NUM_TEMPI];
static p4_tempo_bin_t s_tempi[P4_NUM_TEMPI];
static float s_tempi_smooth[P4_NUM_TEMPI];
static float s_tempi_power_sum = 1e-9f;

static float s_out_bpm = 0.f;
static float s_out_conf = 0.f;
static float s_out_beat_phase_0_1 = 0.f;

static int s_novelty_filled = 0;

static void shift_left(float *arr, uint16_t len, uint16_t shift)
{
    if (shift >= len) {
        memset(arr, 0, len * sizeof(float));
        return;
    }
    memmove(arr, arr + shift, (len - shift) * sizeof(float));
    memset(arr + len - shift, 0, shift * sizeof(float));
}

static void init_window_lookup(void)
{
    if (s_window_init) return;
    s_window_init = 1;
    for (int i = 0; i < 2048; i++) {
        float sigma = 0.8f;
        float n_minus_half = (float)i - 2048.f / 2.f;
        float g = expf(-0.5f * (n_minus_half / (sigma * 2048.f / 2.f)) * (n_minus_half / (sigma * 2048.f / 2.f)));
        s_window_lookup[i] = g;
        s_window_lookup[4095 - i] = g;
    }
}

static void init_tempo_goertzel_constants(void)
{
    for (uint16_t i = 0; i < P4_NUM_TEMPI; i++) {
        float progress = (float)i / (float)P4_NUM_TEMPI;
        float tempo_bpm = (P4_TEMPO_HIGH - P4_TEMPO_LOW) * progress + (float)P4_TEMPO_LOW;
        s_tempi_bpm_hz[i] = tempo_bpm / 60.f;
    }

    for (uint16_t i = 0; i < P4_NUM_TEMPI; i++) {
        float target_hz = s_tempi_bpm_hz[i];
        float left_hz = (i == 0) ? target_hz : s_tempi_bpm_hz[i - 1];
        float right_hz = (i == P4_NUM_TEMPI - 1) ? target_hz : s_tempi_bpm_hz[i + 1];
        float dl = fabsf(left_hz - target_hz);
        float dr = fabsf(right_hz - target_hz);
        float max_dist_hz = (dl > dr) ? dl : dr;
        if (max_dist_hz < 1e-9f) max_dist_hz = 1e-9f;

        float block_f = P4_NOVELTY_LOG_HZ / (max_dist_hz * 0.5f);
        if (block_f > (float)P4_NOVELTY_HISTORY_LENGTH) block_f = (float)P4_NOVELTY_HISTORY_LENGTH;
        if (block_f < 4.f) block_f = 4.f;
        s_tempi[i].block_size = (uint32_t)(block_f + 0.5f);
        if (s_tempi[i].block_size > P4_NOVELTY_HISTORY_LENGTH) {
            s_tempi[i].block_size = P4_NOVELTY_HISTORY_LENGTH;
        }

        float k = (float)(int)(0.5f + (s_tempi[i].block_size * target_hz) / P4_NOVELTY_LOG_HZ);
        float w = (2.f * (float)M_PI * k) / (float)s_tempi[i].block_size;
        s_tempi[i].cosine = cosf(w);
        s_tempi[i].sine = sinf(w);
        s_tempi[i].coeff = 2.f * s_tempi[i].cosine;
        s_tempi[i].window_step = 4096.f / (float)s_tempi[i].block_size;
        s_tempi[i].target_tempo_hz = target_hz;
    }
}

static void normalize_novelty_curve(void)
{
    static float max_val = 1e-5f;
    max_val *= 0.99f;
    for (uint16_t i = 0; i < P4_NOVELTY_HISTORY_LENGTH; i++) {
        if (s_novelty_curve[i] > max_val) max_val = s_novelty_curve[i];
    }
    if (max_val < 1e-9f) max_val = 1e-9f;
    float scale = 1.f / max_val;
    for (uint16_t i = 0; i < P4_NOVELTY_HISTORY_LENGTH; i++) {
        s_novelty_curve_normalized[i] = s_novelty_curve[i] * scale;
    }
}

static void normalize_vu_curve(void)
{
    static float max_val = 1e-5f;
    max_val *= 0.99f;
    for (uint16_t i = 0; i < P4_NOVELTY_HISTORY_LENGTH; i++) {
        if (s_vu_curve[i] > max_val) max_val = s_vu_curve[i];
    }
    if (max_val < 1e-9f) max_val = 1e-9f;
    float scale = 1.f / max_val;
    for (uint16_t i = 0; i < P4_NOVELTY_HISTORY_LENGTH; i++) {
        s_vu_curve_normalized[i] = s_vu_curve[i] * scale;
    }
}

static float calculate_magnitude_of_tempo(uint16_t bin)
{
    uint32_t block_size = s_tempi[bin].block_size;
    if (block_size < 4 || (int)s_novelty_filled < (int)block_size) {
        return 0.f;
    }

    float q1 = 0.f, q2 = 0.f;
    float window_pos = 0.f;
    int base = (int)P4_NOVELTY_HISTORY_LENGTH - (int)block_size;

    for (uint32_t i = 0; i < block_size; i++) {
        float sn = s_novelty_curve_normalized[base + i];
        float sv = s_vu_curve_normalized[base + i];
        float sample = (sn + sv) * 0.5f;
        uint32_t wi = (uint32_t)window_pos;
        if (wi > 4095) wi = 4095;
        float q0 = s_tempi[bin].coeff * q1 - q2 + sample * s_window_lookup[wi];
        q2 = q1;
        q1 = q0;
        window_pos += s_tempi[bin].window_step;
    }

    float real = q1 - q2 * s_tempi[bin].cosine;
    float imag = q2 * s_tempi[bin].sine;
    s_tempi[bin].phase = atan2f(imag, real) + ((float)M_PI * BEAT_SHIFT_PERCENT);
    while (s_tempi[bin].phase > (float)M_PI) s_tempi[bin].phase -= 2.f * (float)M_PI;
    while (s_tempi[bin].phase < -(float)M_PI) s_tempi[bin].phase += 2.f * (float)M_PI;

    float mag_sq = q1 * q1 + q2 * q2 - q1 * q2 * s_tempi[bin].coeff;
    float mag = sqrtf(mag_sq > 0.f ? mag_sq : 0.f);
    float norm = mag / ((float)block_size * 0.5f);
    return norm < 0.f ? 0.f : norm;
}

static void calculate_tempi_magnitudes(void)
{
    float max_val = 0.02f;
    for (uint16_t i = 0; i < P4_NUM_TEMPI; i++) {
        /* Only compute bins that have enough history (early output: ~4 s for 120 BPM instead of 33 s) */
        if ((int)s_novelty_filled < (int)s_tempi[i].block_size) {
            s_tempi[i].magnitude_full_scale = 0.f;
        } else {
            s_tempi[i].magnitude_full_scale = calculate_magnitude_of_tempo(i);
        }
        if (s_tempi[i].magnitude_full_scale > max_val) {
            max_val = s_tempi[i].magnitude_full_scale;
        }
    }
    if (max_val < 0.02f) max_val = 0.02f;
    float scale = 1.f / max_val;
    for (uint16_t i = 0; i < P4_NUM_TEMPI; i++) {
        float s = s_tempi[i].magnitude_full_scale * scale;
        if (s < 0.f) s = 0.f;
        if (s > 1.f) s = 1.f;
        s_tempi[i].magnitude = s * s * s;
    }
}

void tempo_emotiscope_init(void)
{
    memset(s_novelty_curve, 0, sizeof(s_novelty_curve));
    memset(s_novelty_curve_normalized, 0, sizeof(s_novelty_curve_normalized));
    memset(s_vu_curve, 0, sizeof(s_vu_curve));
    memset(s_vu_curve_normalized, 0, sizeof(s_vu_curve_normalized));
    memset(s_tempi_smooth, 0, sizeof(s_tempi_smooth));
    s_novelty_filled = 0;
    s_tempi_power_sum = 1e-9f;
    s_out_bpm = 0.f;
    s_out_conf = 0.f;
    s_out_beat_phase_0_1 = 0.f;

    init_window_lookup();
    init_tempo_goertzel_constants();
}

void tempo_emotiscope_feed(float flux, float vu_positive_delta)
{
    shift_left(s_novelty_curve, P4_NOVELTY_HISTORY_LENGTH, 1);
    s_novelty_curve[P4_NOVELTY_HISTORY_LENGTH - 1] = log1pf(flux > 0.f ? flux : 0.f);
    if (s_novelty_filled < P4_NOVELTY_HISTORY_LENGTH) s_novelty_filled++;

    shift_left(s_vu_curve, P4_NOVELTY_HISTORY_LENGTH, 1);
    s_vu_curve[P4_NOVELTY_HISTORY_LENGTH - 1] = vu_positive_delta > 0.f ? vu_positive_delta : 0.f;

    normalize_novelty_curve();
    normalize_vu_curve();
    calculate_tempi_magnitudes();

    s_tempi_power_sum = 1e-9f;
    for (uint16_t i = 0; i < P4_NUM_TEMPI; i++) {
        s_tempi_smooth[i] = s_tempi_smooth[i] * 0.975f + s_tempi[i].magnitude * 0.025f;
        s_tempi_power_sum += s_tempi_smooth[i];
    }

    float max_contrib = 0.f;
    uint16_t top_bin = 0;
    for (uint16_t i = 0; i < P4_NUM_TEMPI; i++) {
        float c = s_tempi_smooth[i] / s_tempi_power_sum;
        if (c > max_contrib) {
            max_contrib = c;
            top_bin = i;
        }
    }

    /* Only publish BPM/conf when we have meaningful data (avoids 48 BPM / 0 conf during warmup) */
    if (max_contrib > 0.01f && s_tempi_power_sum > 0.01f) {
        s_out_bpm = (float)(P4_TEMPO_LOW + top_bin);
        s_out_conf = max_contrib;
        float ph = s_tempi[top_bin].phase;
        s_out_beat_phase_0_1 = (ph + (float)M_PI) / (2.f * (float)M_PI);
        if (s_out_beat_phase_0_1 < 0.f) s_out_beat_phase_0_1 = 0.f;
        if (s_out_beat_phase_0_1 > 1.f) s_out_beat_phase_0_1 = 1.f;
    }
}

void tempo_emotiscope_get(float *bpm_out, float *conf_out, float *beat_phase_0_1_out)
{
    if (bpm_out) *bpm_out = s_out_bpm;
    if (conf_out) *conf_out = s_out_conf;
    if (beat_phase_0_1_out) *beat_phase_0_1_out = s_out_beat_phase_0_1;
}
