/*
 * Tempo stabilizer: octave resolver + hysteresis.
 * Raw tempo -> candidates (1x, 2x, 0.5x) in [60, 180] BPM; pick closest to
 * previous output; switch only after N consecutive wins and conf >= CONF_MIN.
 */
#include "tempo_stabilizer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "tempo_stab";

typedef enum {
    MODE_1X = 0,
    MODE_2X = 1,
    MODE_HALF = 2,
    MODE_COUNT
} candidate_mode_t;

static float clamp_bpm(float bpm)
{
    if (bpm < TEMPO_STABILIZER_BPM_MIN) return TEMPO_STABILIZER_BPM_MIN;
    if (bpm > TEMPO_STABILIZER_BPM_MAX) return TEMPO_STABILIZER_BPM_MAX;
    return bpm;
}

static float s_tempo_out = 0.f;
static int s_has_prior = 0;
static int s_consecutive_count = 0;
static int s_current_candidate_mode = MODE_1X;
static float s_last_tempo_raw = 0.f;
static float s_last_conf = 0.f;
static int s_last_chosen_mode = MODE_1X;
static uint64_t s_last_log_us = 0;
#define LOG_INTERVAL_US 1000000u  /* 1 Hz */

void tempo_stabilizer_update(float tempo_raw, float conf)
{
    s_last_tempo_raw = tempo_raw;
    s_last_conf = conf;

    /* Invalid or no signal: hold (do not update tempo_out) */
    if (tempo_raw < 1.0f) {
        return;
    }

    float c0 = clamp_bpm(tempo_raw);
    float c1 = clamp_bpm(tempo_raw * 2.0f);
    float c2 = clamp_bpm(tempo_raw * 0.5f);

    float prev = s_tempo_out;
    if (!s_has_prior) {
        prev = c0;  /* so "closest" is well-defined for first time */
    }

    /* Prefer 1x when raw is in full-tempo range: don't lock on half-time (60) for 120 BPM music */
    float d0 = fabsf(c0 - prev);
    float d1 = fabsf(c1 - prev);
    float d2 = fabsf(c2 - prev);
    if (tempo_raw >= 88.0f) {
        d2 = 1e9f;  /* disqualify 0.5x when raw is already in full-tempo range */
    }
    if (tempo_raw <= 95.0f && s_tempo_out >= 150.0f) {
        d1 = 1e9f;  /* disqualify 2x when raw is low and we're already at double */
    }

    float best_bpm = c0;
    int best_mode = MODE_1X;
    float best_dist = d0;
    if (d1 < best_dist) {
        best_dist = d1;
        best_bpm = c1;
        best_mode = MODE_2X;
    }
    if (d2 < best_dist) {
        best_bpm = c2;
        best_mode = MODE_HALF;
    }

    /* Low confidence: hold tempo_out, do not flip */
    if (conf < TEMPO_STABILIZER_CONF_MIN) {
        return;
    }

    if (best_mode == s_current_candidate_mode) {
        s_consecutive_count++;
        if (s_consecutive_count >= TEMPO_STABILIZER_N_CONSECUTIVE) {
            s_tempo_out = best_bpm;
            s_has_prior = 1;
            s_last_chosen_mode = best_mode;
        }
    } else {
        s_current_candidate_mode = best_mode;
        s_consecutive_count = 1;
        /* First valid reading: set tempo_out immediately so we don't wait N ticks */
        if (!s_has_prior) {
            s_tempo_out = best_bpm;
            s_has_prior = 1;
            s_last_chosen_mode = best_mode;
        }
    }
}

float tempo_stabilizer_get_bpm(void)
{
    return s_tempo_out;
}

static const char *mode_str(int mode)
{
    switch (mode) {
        case MODE_1X:  return "1x";
        case MODE_2X:  return "2x";
        case MODE_HALF: return "0.5x";
        default: return "?";
    }
}

void tempo_stabilizer_log_if_due(void)
{
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    if (now_us - s_last_log_us < LOG_INTERVAL_US) {
        return;
    }
    s_last_log_us = now_us;

    ESP_LOGI(TAG, "tempo_raw=%.1f tempo_out=%.1f conf=%.3f mode=%s",
             s_last_tempo_raw, s_tempo_out, s_last_conf,
             mode_str(s_last_chosen_mode));
}
