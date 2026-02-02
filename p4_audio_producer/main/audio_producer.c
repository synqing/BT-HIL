/*
 * Two-lane audio producer: capture, fast lane, slow lane, publish.
 * Real I2S DMA capture with event-driven scheduling and full DSP pipeline.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "audio_producer.h"
#include "audio_config.h"
#include "ring_buffer.h"
#include "audio_i2s_config.h"

/* ESP-DSP for FFT (optional, fallback to manual if unavailable) */
#ifdef CONFIG_DSP_ENABLED
#include "dsps_fft.h"
#include "dsps_wind.h"
#endif

static const char *TAG = "audio_producer";

volatile uint32_t audio_capture_overruns = 0;
volatile uint32_t audio_fast_lane_overruns = 0;
volatile uint32_t audio_slow_lane_overruns = 0;
volatile uint32_t audio_slow_lane_stale_count = 0;
volatile uint32_t audio_published_seq = 0;
volatile uint64_t audio_t_capture_us = 0;
volatile uint32_t audio_max_fast_lane_us = 0;
volatile uint32_t audio_max_slow_lane_us = 0;
volatile uint64_t audio_t_publish_us = 0;
volatile uint64_t audio_t_visual_commit_us = 0;

/* Hop rate diagnostics */
volatile uint32_t audio_capture_reads_count = 0;
volatile uint32_t audio_fast_lane_wakeups_count = 0;
volatile uint64_t audio_diag_start_time_us = 0;

#define LATENCY_HISTORY_LEN 64
static uint32_t s_audio_latency_us[LATENCY_HISTORY_LEN];
static uint32_t s_e2e_latency_us[LATENCY_HISTORY_LEN];
static unsigned s_latency_idx = 0;

static ring_buffer_t s_ring;
static AudioFrame s_published;
static AudioFrame s_last_slow;
static SemaphoreHandle_t s_frame_mux;
static i2s_chan_handle_t s_rx_handle = NULL;
static float s_hop_buf[AUDIO_HOP_SIZE];
static int32_t s_i2s_buf[AUDIO_HOP_SIZE];

/* Event-driven scheduling: task handles for notifications (exported for main.c) */
TaskHandle_t s_fast_lane_handle = NULL;
TaskHandle_t s_slow_lane_handle = NULL;
static volatile uint32_t s_hop_counter = 0;

/* Fast lane DSP state */
#define NOISE_FLOOR_WINDOW_HOPS 125  /* 1 second at 8ms/hop */
static float s_rms_history[NOISE_FLOOR_WINDOW_HOPS];
static unsigned s_rms_history_idx = 0;

/* Slow lane DSP state */
#define FFT_SIZE 512
#define GOERTZEL_SIZE 1024
static float s_fft_window[FFT_SIZE];  /* Hann window (precomputed once) */
static float s_fft_input[FFT_SIZE * 2];  /* Complex: real, imag interleaved */
static float s_fft_mag[FFT_SIZE / 2];  /* FFT magnitude bins */
static int s_fft_window_init = 0;
static int s_fft_init_done = 0;  /* esp-dsp FFT initialization flag */

/* Slow lane working buffers (moved off stack to prevent overflow) */
static float s_slow_history_buf[GOERTZEL_SIZE];  /* For Goertzel: 1024 samples */
static float s_slow_fft_buf[FFT_SIZE];  /* For FFT512: 512 samples */

/* Goertzel state for 64 musical bins (A1-C7, semitone-spaced) */
static float s_goertzel_coeffs[64];  /* Precomputed coefficients */
static int s_goertzel_init = 0;

/* Task handles for stack monitoring (exported for main.c) */
TaskHandle_t s_capture_handle = NULL;

/* DSP self-test buffers (static to avoid stack allocation) */
#define DSP_SELFTEST_SIZE 512
static float s_dsp_selftest_sine[DSP_SELFTEST_SIZE];
static float s_dsp_selftest_windowed[DSP_SELFTEST_SIZE];
static float s_dsp_selftest_fft[DSP_SELFTEST_SIZE * 2];  /* Complex: real, imag interleaved */
static float s_dsp_selftest_mag[DSP_SELFTEST_SIZE / 2];

/**
 * DSP self-test: verifies esp-dsp FFT functionality with synthetic sine wave.
 * Generates 1000 Hz sine at Fs=16k, applies Hann window, runs FFT, verifies peak bin.
 * All buffers are static to avoid stack allocation.
 */
void dsp_selftest(void)
{
#ifdef CONFIG_DSP_ENABLED
    /* Generate synthetic sine wave: 1000 Hz at Fs=16000 Hz */
    float freq_hz = 1000.0f;
    float phase = 0.0f;
    float phase_inc = (2.0f * M_PI * freq_hz) / AUDIO_FS;
    
    for (int i = 0; i < DSP_SELFTEST_SIZE; i++) {
        s_dsp_selftest_sine[i] = sinf(phase);
        phase += phase_inc;
        if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
    }
    
    /* Apply Hann window using esp-dsp */
    esp_err_t ret = dsps_wind_hann_f32(s_dsp_selftest_sine, DSP_SELFTEST_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DSP_SELFTEST: FAIL (window error %d)", ret);
        return;
    }
    
    /* Copy windowed signal to FFT input buffer (complex format) */
    for (int i = 0; i < DSP_SELFTEST_SIZE; i++) {
        s_dsp_selftest_fft[i * 2] = s_dsp_selftest_sine[i];  /* Real */
        s_dsp_selftest_fft[i * 2 + 1] = 0.0f;  /* Imaginary */
    }
    
    /* Initialize FFT (radix-4 for 512 points) */
    ret = dsps_fft4r_init_fc32(NULL, DSP_SELFTEST_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DSP_SELFTEST: FAIL (FFT init error %d)", ret);
        return;
    }
    
    /* Run FFT */
    dsps_fft4r_fc32(s_dsp_selftest_fft, DSP_SELFTEST_SIZE);
    dsps_bit_rev4r_fc32(s_dsp_selftest_fft, DSP_SELFTEST_SIZE);
    dsps_cplx2real_fc32(s_dsp_selftest_fft, DSP_SELFTEST_SIZE);
    
    /* Compute magnitude spectrum */
    for (int i = 0; i < DSP_SELFTEST_SIZE / 2; i++) {
        float real = s_dsp_selftest_fft[i * 2];
        float imag = s_dsp_selftest_fft[i * 2 + 1];
        s_dsp_selftest_mag[i] = sqrtf(real * real + imag * imag);
    }
    
    /* Find peak bin */
    int peak_bin = 0;
    float peak_mag = 0.0f;
    for (int i = 0; i < DSP_SELFTEST_SIZE / 2; i++) {
        if (s_dsp_selftest_mag[i] > peak_mag) {
            peak_mag = s_dsp_selftest_mag[i];
            peak_bin = i;
        }
    }
    
    /* Expected bin: 1000 Hz / (16000 Hz / 512) = 32 */
    int expected_bin = (int)((freq_hz * DSP_SELFTEST_SIZE) / AUDIO_FS);
    int bin_error = abs(peak_bin - expected_bin);
    
    if (bin_error <= 1) {
        ESP_LOGI(TAG, "DSP_SELFTEST: PASS (peak bin=%d, expected=%d, error=%d)", 
                 peak_bin, expected_bin, bin_error);
    } else {
        ESP_LOGE(TAG, "DSP_SELFTEST: FAIL (peak bin=%d, expected=%d, error=%d)", 
                 peak_bin, expected_bin, bin_error);
    }
#else
    ESP_LOGW(TAG, "DSP_SELFTEST: SKIP (CONFIG_DSP_ENABLED not set)");
#endif
}

/**
 * Delay helper: ensures delay never becomes 0 ticks (which would cause starvation).
 * With 1000 Hz tick rate, pdMS_TO_TICKS(8) = 8, but this protects against
 * future tick rate changes that could reintroduce the bug.
 */
static inline TickType_t delay_ms_min1(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    return (t == 0) ? 1 : t;  // never allow a 0-tick "delay"
}

/**
 * Capture sanity check: detect clipping, DC offset, silence.
 */
static void capture_sanity_check(const float *hop, unsigned n, uint32_t *flags)
{
    float rms = 0.f, peak = 0.f, dc = 0.f;
    int clipped = 0;
    
    for (unsigned i = 0; i < n; i++) {
        float s = hop[i];
        if (s < -1.0f || s > 1.0f) clipped = 1;
        float abs_s = (s < 0) ? -s : s;
        if (abs_s > peak) peak = abs_s;
        rms += abs_s * abs_s;
        dc += s;  // signed for DC
    }
    rms = sqrtf(rms / n);
    dc = dc / n;
    
    if (clipped) *flags |= AUDIO_FLAG_CLIPPED;
    if (fabsf(dc) > 0.1f) {
        ESP_LOGW(TAG, "DC offset: %.3f", dc);
    }
    if (rms < 1e-6f) {
        ESP_LOGD(TAG, "Silence detected (RMS=%.6f)", rms);
    }
}

/**
 * Initialize Goertzel coefficients for 64 musical bins (A1-C7).
 * Frequencies: A1=55Hz, A#1=58.27Hz, ..., C7=2093Hz (semitone-spaced).
 */
static void goertzel_init_coeffs(void)
{
    if (s_goertzel_init) return;
    
    float base_freq = 55.0f;  /* A1 */
    float semitone_ratio = powf(2.0f, 1.0f / 12.0f);
    
    for (int i = 0; i < 64; i++) {
        float freq = base_freq * powf(semitone_ratio, i);
        float k = (GOERTZEL_SIZE * freq) / AUDIO_FS;
        float w = (2.0f * M_PI * k) / GOERTZEL_SIZE;
        s_goertzel_coeffs[i] = 2.0f * cosf(w);
    }
    
    s_goertzel_init = 1;
}

/**
 * Compute Goertzel magnitude for one bin.
 * Uses precomputed coefficient (2*cos(w)) where w = 2*pi*k/N.
 */
static float goertzel_magnitude(const float *samples, unsigned n, float coeff)
{
    float q0 = 0.f, q1 = 0.f, q2 = 0.f;
    
    /* Goertzel filter: q[n] = coeff * q[n-1] - q[n-2] + x[n] */
    for (unsigned i = 0; i < n; i++) {
        q0 = coeff * q1 - q2 + samples[i];
        q2 = q1;
        q1 = q0;
    }
    
    /* Magnitude: sqrt(q1^2 + q2^2 - q1*q2*coeff) */
    /* This is the standard Goertzel magnitude formula */
    return sqrtf(q1 * q1 + q2 * q2 - q1 * q2 * coeff);
}

/**
 * Initialize FFT window (Hann) using esp-dsp.
 */
static void fft_window_init(void)
{
    if (s_fft_window_init) return;
    
#ifdef CONFIG_DSP_ENABLED
    /* Use esp-dsp Hann window function */
    esp_err_t ret = dsps_wind_hann_f32(s_fft_window, FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Hann window: %d", ret);
        /* Fallback to manual calculation */
        for (int i = 0; i < FFT_SIZE; i++) {
            s_fft_window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
        }
    }
#else
    /* Manual calculation if esp-dsp not available */
    for (int i = 0; i < FFT_SIZE; i++) {
        s_fft_window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
    }
#endif
    
    s_fft_window_init = 1;
}

/**
 * Compute FFT512 using esp-dsp (Phase 3B).
 * Input: samples[FFT_SIZE] (real samples)
 * Output: out_mag[FFT_SIZE/2] (magnitude spectrum)
 */
static void compute_fft_512(const float *samples, float *out_mag)
{
#ifdef CONFIG_DSP_ENABLED
    /* Initialize esp-dsp FFT if not done yet */
    if (!s_fft_init_done) {
        esp_err_t ret = dsps_fft4r_init_fc32(NULL, FFT_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "FFT init failed: %d", ret);
            memset(out_mag, 0, (FFT_SIZE / 2) * sizeof(float));
            return;
        }
        s_fft_init_done = 1;
    }
    
    /* Ensure window is initialized */
    if (!s_fft_window_init) {
        fft_window_init();
    }
    
    /* Apply window and convert to complex format */
    for (int i = 0; i < FFT_SIZE; i++) {
        s_fft_input[i * 2] = samples[i] * s_fft_window[i];  /* Real */
        s_fft_input[i * 2 + 1] = 0.0f;  /* Imaginary */
    }
    
    /* Run FFT using esp-dsp */
    dsps_fft4r_fc32(s_fft_input, FFT_SIZE);
    dsps_bit_rev4r_fc32(s_fft_input, FFT_SIZE);
    dsps_cplx2real_fc32(s_fft_input, FFT_SIZE);
    
    /* Compute magnitude spectrum */
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = s_fft_input[i * 2];
        float imag = s_fft_input[i * 2 + 1];
        out_mag[i] = sqrtf(real * real + imag * imag);
    }
#else
    /* Fallback: zero output if esp-dsp not available */
    ESP_LOGW(TAG, "FFT: esp-dsp not available, zeroing output");
    memset(out_mag, 0, (FFT_SIZE / 2) * sizeof(float));
#endif
}

/**
 * Fast lane features: RMS, peak, noise floor, onset, band_energy[8].
 */
static void fill_fast_lane_from_hop(AudioFrame *f, const float *hop, unsigned n, uint64_t t_capture_us)
{
    /* RMS and peak */
    float sum_sq = 0.f;
    float peak = 0.f;
    for (unsigned i = 0; i < n; i++) {
        float s = hop[i];
        if (s < 0) s = -s;
        if (s > peak) peak = s;
        sum_sq += s * s;
    }
    f->vu_rms = (n > 0) ? sqrtf(sum_sq / n) : 0.f;
    f->vu_peak = peak;
    f->crest_factor = (f->vu_rms > 1e-9f) ? (f->vu_peak / f->vu_rms) : 0.f;
    
    /* Noise floor: track minimum RMS over sliding window */
    s_rms_history[s_rms_history_idx % NOISE_FLOOR_WINDOW_HOPS] = f->vu_rms;
    s_rms_history_idx++;
    float min_rms = f->vu_rms;
    unsigned window_size = (s_rms_history_idx < NOISE_FLOOR_WINDOW_HOPS) ? s_rms_history_idx : NOISE_FLOOR_WINDOW_HOPS;
    for (unsigned i = 0; i < window_size; i++) {
        if (s_rms_history[i] < min_rms) min_rms = s_rms_history[i];
    }
    f->noise_floor = min_rms;
    if (f->noise_floor > 0.01f) {  /* Threshold: adjust as needed */
        f->flags |= AUDIO_FLAG_NOISE_FLOOR_HIGH;
    }
    
    /* Onset detection: simple energy-based (much faster than DFT) */
    /* Compare current RMS to previous RMS - spike indicates onset */
    static float s_prev_rms = 0.f;
    float rms_diff = f->vu_rms - s_prev_rms;
    f->onset_strength = (rms_diff > 0.f) ? rms_diff : 0.f;  /* Half-wave rectify */
    s_prev_rms = f->vu_rms;
    
    /* TODO: Replace with proper spectral flux FFT when ESP-DSP available */
    
    /* Band energy [8]: simple frequency-domain approximation using sample differences */
    /* TODO: Replace with proper FFT-based band energy when ESP-DSP available */
    memset(f->band_energy, 0, sizeof(f->band_energy));
    /* For now, distribute energy evenly across bands based on RMS */
    /* This is a placeholder until proper FFT is available */
    float energy_per_band = f->vu_rms * f->vu_rms / 8.0f;
    for (int i = 0; i < 8; i++) {
        f->band_energy[i] = energy_per_band;
    }
    
    f->t_capture_us = t_capture_us;
}

/**
 * Capture task: reads I2S DMA, writes to ring buffer, notifies lanes.
 * Event-driven: blocks on I2S read (natural timing), notifies fast/slow lanes.
 */
static void capture_task(void *arg)
{
    (void)arg;
    uint32_t flags = 0;

    while (1) {
        uint64_t t_start_us = esp_timer_get_time();
        
        /* Block on I2S DMA read (provides natural 8ms timing) - use portMAX_DELAY for true blocking */
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_rx_handle, s_i2s_buf, sizeof(s_i2s_buf), &bytes_read, portMAX_DELAY);
        audio_capture_reads_count++;
        
        flags = 0;
        if (ret != ESP_OK || bytes_read < sizeof(s_i2s_buf)) {
            /* Underrun: fill with zeros, set flag */
            memset(s_hop_buf, 0, sizeof(s_hop_buf));
            flags |= AUDIO_FLAG_UNDERRUN;
        } else {
            /* Convert int32_t samples to float [-1.0, 1.0] */
            for (unsigned i = 0; i < AUDIO_HOP_SIZE; i++) {
                s_hop_buf[i] = (float)s_i2s_buf[i] / 32768.f;
            }
            
            /* Sanity check: clipping, DC offset, silence */
            capture_sanity_check(s_hop_buf, AUDIO_HOP_SIZE, &flags);
        }
        
        /* Write to ring buffer */
        int overflow = ring_write_hop(&s_ring, s_hop_buf, AUDIO_HOP_SIZE);
        if (overflow) {
            audio_capture_overruns++;
            flags |= AUDIO_FLAG_OVERFLOW;
        }
        
        uint64_t t_end_us = esp_timer_get_time();
        audio_t_capture_us = t_end_us;
        
        /* Event-driven: notify fast lane (every hop) using counting notifications */
        if (s_fast_lane_handle) {
            xTaskNotifyGive(s_fast_lane_handle);
        }
        
        /* Event-driven: notify slow lane (every N hops) - only if task is enabled */
        s_hop_counter++;
        if (s_hop_counter >= AUDIO_SLOW_LANE_HOP_DIV) {
            if (s_slow_lane_handle) {
                xTaskNotifyGive(s_slow_lane_handle);
            }
            s_hop_counter = 0;
        }
    }
}

/**
 * Fast lane task: processes every hop, computes fast features.
 * Event-driven: wakes on notification from capture task.
 */
static void fast_lane_task(void *arg)
{
    (void)arg;
    float hop[AUDIO_HOP_SIZE];
    uint32_t max_fast_us = 0;

    while (1) {
        /* Wait for notification from capture task using counting notifications */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        audio_fast_lane_wakeups_count++;
        
        int64_t start_us = (int64_t)esp_timer_get_time();

        ring_read_latest_hop(&s_ring, hop, AUDIO_HOP_SIZE);
        uint64_t t_cap = audio_t_capture_us;

        if (xSemaphoreTake(s_frame_mux, delay_ms_min1(5)) != pdTRUE) {
            audio_fast_lane_overruns++;
            continue;
        }
        
        AudioFrame *f = &s_published;
        f->version = AUDIO_FRAME_VERSION;
        f->seq = audio_published_seq++;
        f->t_capture_us = t_cap;
        /* Flags: check capture overruns and slow lane stale */
        f->flags = 0;
        if (audio_capture_overruns > 0) f->flags |= AUDIO_FLAG_OVERFLOW;
        if (audio_slow_lane_stale_count > 0) f->flags |= AUDIO_FLAG_SLOW_LANE_STALE;
        f->quality = 1.0f;
        
        fill_fast_lane_from_hop(f, hop, AUDIO_HOP_SIZE, t_cap);
        
        /* Copy slow lane data (updated by slow lane task) */
        memcpy(&f->goertzel_bins, &s_last_slow.goertzel_bins, sizeof(f->goertzel_bins));
        memcpy(&f->chroma, &s_last_slow.chroma, sizeof(f->chroma));
        f->chroma_conf = s_last_slow.chroma_conf;
        f->tempo_bpm = s_last_slow.tempo_bpm;
        f->tempo_conf = s_last_slow.tempo_conf;
        f->beat_phase_0_1 = s_last_slow.beat_phase_0_1;
        f->beat_conf = s_last_slow.beat_conf;
        f->centroid = s_last_slow.centroid;
        f->rolloff = s_last_slow.rolloff;
        f->flatness = s_last_slow.flatness;
        
        uint64_t t_pub = (uint64_t)esp_timer_get_time();
        audio_t_publish_us = t_pub;
        audio_t_visual_commit_us = t_pub;  /* Stub: visual consumer will update */
        
        uint32_t audio_lat_us = (uint32_t)(t_pub - t_cap);
        uint32_t e2e_lat_us = (uint32_t)(t_pub - t_cap);
        
        /* Yield to ensure IDLE task gets CPU time (prevent WDT starvation) */
        taskYIELD();
        s_audio_latency_us[s_latency_idx % LATENCY_HISTORY_LEN] = audio_lat_us;
        s_e2e_latency_us[s_latency_idx % LATENCY_HISTORY_LEN] = e2e_lat_us;
        s_latency_idx++;
        xSemaphoreGive(s_frame_mux);

        int64_t elapsed_us = (int64_t)esp_timer_get_time() - start_us;
        if (elapsed_us > (int64_t)max_fast_us) {
            max_fast_us = (uint32_t)elapsed_us;
            audio_max_fast_lane_us = max_fast_us;
        }
        if (elapsed_us > (int64_t)AUDIO_FAST_LANE_BUDGET_US) {
            audio_fast_lane_overruns++;
        }
    }
}

/**
 * Slow lane task: Phase 3B - FFT512 Hann + spectral features via esp-dsp.
 * Event-driven: wakes on notification from capture task (every AUDIO_SLOW_LANE_HOP_DIV hops).
 */
static void slow_lane_task(void *arg)
{
    (void)arg;
    uint32_t max_slow_us = 0;
    uint64_t last_process_time_us = 0;
    uint32_t expected_cadence_us = AUDIO_SLOW_LANE_HOP_DIV * AUDIO_HOP_MS * 1000;  /* 32 ms */

    /* Initialize FFT window */
    fft_window_init();

    while (1) {
        /* Wait for notification from capture task (every N hops) using counting notifications */
        uint64_t t_start_us = esp_timer_get_time();
        
        /* Check for stale detection: if notification arrives too late */
        if (last_process_time_us > 0) {
            uint64_t time_since_last = t_start_us - last_process_time_us;
            if (time_since_last > (expected_cadence_us * 2)) {
                /* Missed cadence - mark as stale */
                audio_slow_lane_stale_count++;
            }
        }
        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Read 512 samples from ring buffer history (for FFT512, 75% overlap) */
        ring_read_history(&s_ring, s_slow_fft_buf, 0, FFT_SIZE);

        /* FFT512: compute spectral features (centroid, rolloff, flatness) */
        compute_fft_512(s_slow_fft_buf, s_fft_mag);
        
        /* Spectral centroid: weighted average frequency */
        float mag_sum = 0.f;
        float weighted_sum = 0.f;
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            float freq_hz = (i * AUDIO_FS) / FFT_SIZE;
            weighted_sum += freq_hz * s_fft_mag[i];
            mag_sum += s_fft_mag[i];
        }
        s_last_slow.centroid = (mag_sum > 1e-9f) ? (weighted_sum / mag_sum) : 0.f;
        
        /* Spectral rolloff: frequency below which 85% of energy is contained */
        float energy_sum = 0.f;
        float energy_total = 0.f;
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            energy_total += s_fft_mag[i] * s_fft_mag[i];
        }
        float energy_threshold = 0.85f * energy_total;
        s_last_slow.rolloff = 0.f;
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            energy_sum += s_fft_mag[i] * s_fft_mag[i];
            if (energy_sum >= energy_threshold) {
                s_last_slow.rolloff = (i * AUDIO_FS) / FFT_SIZE;
                break;
            }
        }
        
        /* Spectral flatness: geometric_mean / arithmetic_mean */
        float geo_mean = 1.f;
        float arith_mean = 0.f;
        int non_zero_count = 0;
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            if (s_fft_mag[i] > 1e-9f) {
                geo_mean *= powf(s_fft_mag[i], 1.0f / (FFT_SIZE / 2));
                arith_mean += s_fft_mag[i];
                non_zero_count++;
            }
        }
        arith_mean /= (FFT_SIZE / 2);
        s_last_slow.flatness = (arith_mean > 1e-9f && non_zero_count > 0) ? (geo_mean / arith_mean) : 0.f;

        /* Measure processing time */
        uint64_t t_end_us = esp_timer_get_time();
        uint32_t elapsed_us = (uint32_t)(t_end_us - t_start_us);
        last_process_time_us = t_end_us;
        
        if (elapsed_us > max_slow_us) {
            max_slow_us = elapsed_us;
            audio_max_slow_lane_us = max_slow_us;
        }
        
        /* Check for overrun (if processing takes too long) */
        if (elapsed_us > expected_cadence_us) {
            audio_slow_lane_overruns++;
        }
    }
}

void audio_producer_start(void)
{
    memset(&s_published, 0, sizeof(s_published));
    memset(&s_last_slow, 0, sizeof(s_last_slow));
    memset(s_rms_history, 0, sizeof(s_rms_history));
    s_published.version = AUDIO_FRAME_VERSION;
    s_frame_mux = xSemaphoreCreateMutex();
    if (!s_frame_mux) {
        ESP_LOGE(TAG, "no mutex");
        return;
    }
    ring_init(&s_ring);

    /* Initialize I2S for real audio capture with DMA-paced configuration */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(AUDIO_I2S_PORT_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    /* Configure DMA: one frame = one hop (128 samples) to prevent backlog draining */
    chan_cfg.dma_frame_num = AUDIO_HOP_SIZE;  /* DMA frame size matches hop size */
    chan_cfg.dma_desc_num = 4;  /* Small descriptor count to prevent backlog */
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed %d", err);
        return;
    }
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_FS),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = AUDIO_I2S_MCK_GPIO,
            .bclk = AUDIO_I2S_BCK_GPIO,
            .ws = AUDIO_I2S_WS_GPIO,
            .dout = -1,
            .din = AUDIO_I2S_DI_GPIO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    err = i2s_channel_init_std_mode(s_rx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed %d", err);
        return;
    }
    err = i2s_channel_enable(s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable failed %d", err);
        return;
    }
    ESP_LOGI(TAG, "I2S configured: DMA frame=%u samples, descs=%u", (unsigned)chan_cfg.dma_frame_num, (unsigned)chan_cfg.dma_desc_num);

    /* Initialize diagnostics */
    audio_diag_start_time_us = esp_timer_get_time();
    
    /* Create tasks with handles for event-driven notifications */
    /* Pin capture to CPU0 (highest priority, mostly blocked on DMA) */
    xTaskCreatePinnedToCore(capture_task, "capture", 3072, NULL, 5, &s_capture_handle, 0);
    /* Pin fast_lane to CPU1 to reduce CPU0 load and IDLE0 starvation */
    xTaskCreatePinnedToCore(fast_lane_task, "fast_lane", 3072, NULL, 4, &s_fast_lane_handle, 1);
    
    /* Phase 3A: Re-enabled slow_lane in scheduler-only mode (no DSP yet) */
    xTaskCreatePinnedToCore(slow_lane_task, "slow_lane", 12288, NULL, 3, &s_slow_lane_handle, 1);
    
    ESP_LOGI(TAG, "producer started (real I2S capture, counting notifications, fast_lane on CPU1, slow_lane ENABLED with FFT512)");
}

void audio_producer_get_latest(AudioFrame *out)
{
    if (!out) return;
    if (xSemaphoreTake(s_frame_mux, delay_ms_min1(10)) == pdTRUE) {
        memcpy(out, &s_published, sizeof(AudioFrame));
        xSemaphoreGive(s_frame_mux);
    }
}

static int uint32_cmp(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a;
    uint32_t y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

void audio_producer_log_latency(void)
{
    unsigned n = (s_latency_idx < LATENCY_HISTORY_LEN) ? s_latency_idx : LATENCY_HISTORY_LEN;
    if (n == 0) return;
    uint32_t au[LATENCY_HISTORY_LEN];
    uint32_t e2e[LATENCY_HISTORY_LEN];
    memcpy(au, s_audio_latency_us, n * sizeof(uint32_t));
    memcpy(e2e, s_e2e_latency_us, n * sizeof(uint32_t));
    qsort(au, n, sizeof(uint32_t), uint32_cmp);
    qsort(e2e, n, sizeof(uint32_t), uint32_cmp);
    uint64_t sum_au = 0, sum_e2e = 0;
    for (unsigned i = 0; i < n; i++) {
        sum_au += au[i];
        sum_e2e += e2e[i];
    }
    uint32_t p99_au = au[(n * 99) / 100];
    uint32_t p99_e2e = e2e[(n * 99) / 100];
    
    /* Stack high-water mark monitoring */
    UBaseType_t capture_stack = s_capture_handle ? uxTaskGetStackHighWaterMark(s_capture_handle) : 0;
    UBaseType_t fast_stack = s_fast_lane_handle ? uxTaskGetStackHighWaterMark(s_fast_lane_handle) : 0;
    UBaseType_t slow_stack = s_slow_lane_handle ? uxTaskGetStackHighWaterMark(s_slow_lane_handle) : 0;
    
    /* Hop rate diagnostics */
    uint64_t elapsed_us = esp_timer_get_time() - audio_diag_start_time_us;
    uint32_t capture_rate = (elapsed_us > 0) ? (uint32_t)((audio_capture_reads_count * 1000000ULL) / elapsed_us) : 0;
    uint32_t fast_rate = (elapsed_us > 0) ? (uint32_t)((audio_fast_lane_wakeups_count * 1000000ULL) / elapsed_us) : 0;
    
    ESP_LOGI(TAG, "latency | audio max=%lu avg=%lu p99=%lu us | e2e max=%lu avg=%lu p99=%lu us | capture_overruns=%lu fast_overruns=%lu slow_overruns=%lu",
             (unsigned long)au[n - 1], (unsigned long)(sum_au / n), (unsigned long)p99_au,
             (unsigned long)e2e[n - 1], (unsigned long)(sum_e2e / n), (unsigned long)p99_e2e,
             (unsigned long)audio_capture_overruns, (unsigned long)audio_fast_lane_overruns, (unsigned long)audio_slow_lane_overruns);
    ESP_LOGI(TAG, "stack | capture=%lu fast=%lu slow=%lu words free",
             (unsigned long)capture_stack, (unsigned long)fast_stack, (unsigned long)slow_stack);
    ESP_LOGI(TAG, "hop_rate | capture=%lu/sec fast=%lu/sec (expected ~125/sec)",
             (unsigned long)capture_rate, (unsigned long)fast_rate);
    ESP_LOGI(TAG, "slow_lane | max_slow_us=%lu stale_count=%lu",
             (unsigned long)audio_max_slow_lane_us, (unsigned long)audio_slow_lane_stale_count);
}
