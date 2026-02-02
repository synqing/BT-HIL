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
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "audio_producer.h"
#include "audio_config.h"
#include "ring_buffer.h"
#include "audio_i2s_config.h"
#include "audio_responsiveness.h"
#include "tempo_stabilizer.h"
#include "tempo_emotiscope.h"

/* ESP-DSP for FFT (mandatory - build fails if not present) */
#include "esp_dsp.h"
#include "es8311.h"

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
static es8311_handle_t s_es8311 = NULL;
static float s_hop_buf[AUDIO_HOP_SIZE];
/* I2S buffer: 16-bit mono samples (128 samples * 2 bytes = 256 bytes) */
#define BYTES_PER_HOP (AUDIO_HOP_SIZE * sizeof(int16_t))
static uint8_t s_i2s_raw[BYTES_PER_HOP] __attribute__((aligned(4)));

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
#define FFT_FAST_SIZE 128
#define FFT_FAST_BINS (FFT_FAST_SIZE / 2)
static float s_fft_window[FFT_SIZE];  /* Hann window (precomputed once) */
static float s_fft_input[FFT_SIZE];  /* Packed real FFT buffer (length FFT_SIZE) */
static float s_fft_mag[FFT_SIZE / 2];  /* FFT magnitude bins */
static int s_fft_window_init = 0;
static int s_fft_init_done = 0;  /* esp-dsp FFT initialization flag */

/* Fast lane FFT state (FFT128) */
static float s_fast_fft_window[FFT_FAST_SIZE];
static float s_fast_fft_input[FFT_FAST_SIZE];
static float s_fast_fft_mag[FFT_FAST_BINS];
static float s_fast_fft_prev_mag[FFT_FAST_BINS];
static int s_fast_fft_window_init = 0;
static int s_fast_band_init = 0;
static int s_fast_band_edges[9];

/* Latest capture flags for published AudioFrame */
static volatile uint32_t s_last_capture_flags = 0;

/* Slow lane working buffers (moved off stack to prevent overflow) */
static float s_slow_history_buf[GOERTZEL_SIZE];  /* For Goertzel: 1024 samples */
static float s_slow_fft_buf[FFT_SIZE];  /* For FFT512: 512 samples */

/* Slow lane flux state (novelty is in tempo_emotiscope) */
static float s_prev_slow_mag[FFT_SIZE / 2];
static int s_prev_slow_mag_valid = 0;

/* Goertzel state for 64 musical bins (A1-C7, semitone-spaced) */
static float s_goertzel_coeffs[64];  /* Precomputed coefficients */
static float s_goertzel_freq_hz[64];  /* Precomputed frequencies (Hz) for debug output */
static int s_goertzel_init = 0;


/* Task handles for stack monitoring (exported for main.c) */
TaskHandle_t s_capture_handle = NULL;

/* DSP self-test buffers (static to avoid stack allocation) */
#define DSP_SELFTEST_SIZE 512
static float s_dsp_selftest_sine[DSP_SELFTEST_SIZE];
static float s_dsp_selftest_windowed[DSP_SELFTEST_SIZE];
static float s_dsp_selftest_fft[DSP_SELFTEST_SIZE];  /* Packed real FFT buffer (length DSP_SELFTEST_SIZE) */
static float s_dsp_selftest_mag[DSP_SELFTEST_SIZE / 2];

static esp_err_t audio_codec_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AUDIO_PA_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(AUDIO_PA_GPIO, 1);

    i2c_config_t es_i2c_cfg = {
        .sda_io_num = AUDIO_I2C_SDA_GPIO,
        .scl_io_num = AUDIO_I2C_SCL_GPIO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    esp_err_t ret = i2c_param_config(AUDIO_I2C_PORT_NUM, &es_i2c_cfg);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_driver_install(AUDIO_I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    if (!s_es8311) {
        s_es8311 = es8311_create(AUDIO_I2C_PORT_NUM, ES8311_ADDRRES_0);
    }
    if (!s_es8311) {
        return ESP_FAIL;
    }

    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = (AUDIO_FS * AUDIO_I2S_MCLK_MULTIPLE),
        .sample_frequency = AUDIO_FS,
    };
    ret = es8311_init(s_es8311, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = es8311_sample_frequency_config(s_es8311, AUDIO_FS * AUDIO_I2S_MCLK_MULTIPLE, AUDIO_FS);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = es8311_microphone_config(s_es8311, false);
    return ret;
}

/**
 * DSP self-test: verifies esp-dsp FFT functionality with synthetic sine wave.
 * Generates bin-locked sine at Fs=16k, applies Hann window, runs FFT, verifies peak bin.
 * All buffers are static to avoid stack allocation.
 * REQUIRES: esp-dsp must be linked (build fails if missing).
 * Returns: true if PASS, false if FAIL.
 */
bool dsp_selftest(void)
{
    /* Generate bin-locked sine wave to avoid frequency quantization error */
    const int target_bin = 32;
    const float phase_inc = (2.0f * M_PI * (float)target_bin) / (float)DSP_SELFTEST_SIZE;

    for (int i = 0; i < DSP_SELFTEST_SIZE; i++) {
        s_dsp_selftest_sine[i] = sinf(phase_inc * (float)i);
    }
    
    /* Generate Hann window coefficients into windowed buffer */
    dsps_wind_hann_f32(s_dsp_selftest_windowed, DSP_SELFTEST_SIZE);
    
    /* Apply window: multiply sine * hann, copy to packed real FFT buffer */
    for (int i = 0; i < DSP_SELFTEST_SIZE; i++) {
        float w = s_dsp_selftest_windowed[i];
        float x = s_dsp_selftest_sine[i] * w;
        s_dsp_selftest_fft[i] = x;
    }

    /* Initialize FFT (radix-4 for real input, N/2 complex points) */
    const int fft_points = DSP_SELFTEST_SIZE / 2;
    esp_err_t ret = dsps_fft4r_init_fc32(NULL, fft_points);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DSP_SELFTEST: FAIL (FFT init error %d)", ret);
        return false;
    }
    
    /* Run FFT */
    dsps_fft4r_fc32(s_dsp_selftest_fft, fft_points);
    dsps_bit_rev4r_fc32(s_dsp_selftest_fft, fft_points);
    /* Convert complex FFT output for real input (required for real input FFTs) */
    dsps_cplx2real_fc32(s_dsp_selftest_fft, fft_points);
    
    /* Compute magnitude spectrum from interleaved complex output */
    /* After cplx2real, data is still interleaved: data[i*2+0]=real, data[i*2+1]=imag */
    for (int i = 0; i < DSP_SELFTEST_SIZE / 2; i++) {
        float real = s_dsp_selftest_fft[i * 2];
        float imag = s_dsp_selftest_fft[i * 2 + 1];
        s_dsp_selftest_mag[i] = sqrtf(real * real + imag * imag);
    }
    
    /* Find peak bin (ignore bin 0 - DC component) */
    int peak_bin = 1;
    float peak_mag = s_dsp_selftest_mag[1];
    for (int i = 2; i < DSP_SELFTEST_SIZE / 2; i++) {
        if (s_dsp_selftest_mag[i] > peak_mag) {
            peak_mag = s_dsp_selftest_mag[i];
            peak_bin = i;
        }
    }
    
    /* Expected bin: bin-locked target */
    int expected_bin = target_bin;
    int bin_error = abs(peak_bin - expected_bin);
    
    if (bin_error <= 1) {
        ESP_LOGI(TAG, "DSP_SELFTEST: PASS (peak_bin=%d expected=%d error=%d)", 
                 peak_bin, expected_bin, bin_error);
        return true;
    } else {
        ESP_LOGE(TAG, "DSP_SELFTEST: FAIL (peak_bin=%d expected=%d error=%d)", 
                 peak_bin, expected_bin, bin_error);

        int top_bins[5] = {0, 0, 0, 0, 0};
        float top_mag[5] = {0.f, 0.f, 0.f, 0.f, 0.f};
        for (int i = 1; i < DSP_SELFTEST_SIZE / 2; i++) {
            float mag = s_dsp_selftest_mag[i];
            for (int j = 0; j < 5; j++) {
                if (mag > top_mag[j]) {
                    for (int k = 4; k > j; k--) {
                        top_mag[k] = top_mag[k - 1];
                        top_bins[k] = top_bins[k - 1];
                    }
                    top_mag[j] = mag;
                    top_bins[j] = i;
                    break;
                }
            }
        }
        ESP_LOGE(TAG,
                 "DSP_SELFTEST: top bins: %d=%.3f %d=%.3f %d=%.3f %d=%.3f %d=%.3f",
                 top_bins[0], top_mag[0],
                 top_bins[1], top_mag[1],
                 top_bins[2], top_mag[2],
                 top_bins[3], top_mag[3],
                 top_bins[4], top_mag[4]);
        return false;
    }
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
        s_goertzel_freq_hz[i] = freq;  /* Precompute frequency for debug output */
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
 * REQUIRES: esp-dsp must be linked (build fails if missing).
 */
static void fft_window_init(void)
{
    if (s_fft_window_init) return;
    
    /* Use esp-dsp Hann window function */
    dsps_wind_hann_f32(s_fft_window, FFT_SIZE);
    
    s_fft_window_init = 1;
}

static void dsp_fft_init_max(void)
{
    if (s_fft_init_done) return;

    const int fft_points = FFT_SIZE / 2;
    esp_err_t ret = dsps_fft4r_init_fc32(NULL, fft_points);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FFT init failed: %d", ret);
        return;
    }
    s_fft_init_done = 1;
}

static void fast_fft_window_init(void)
{
    if (s_fast_fft_window_init) return;

    dsps_wind_hann_f32(s_fast_fft_window, FFT_FAST_SIZE);
    s_fast_fft_window_init = 1;
}

static void fast_band_init(void)
{
    if (s_fast_band_init) return;

    const float edges_hz[9] = {0.f, 125.f, 250.f, 500.f, 1000.f, 2000.f, 3000.f, 4000.f, 8000.f};
    for (int i = 0; i < 9; i++) {
        float bin_f = (edges_hz[i] * (float)FFT_FAST_SIZE) / (float)AUDIO_FS;
        int bin = (int)lroundf(bin_f);
        if (bin < 0) bin = 0;
        if (bin > FFT_FAST_BINS) bin = FFT_FAST_BINS;
        s_fast_band_edges[i] = bin;
    }
    for (int i = 1; i < 9; i++) {
        if (s_fast_band_edges[i] < s_fast_band_edges[i - 1]) {
            s_fast_band_edges[i] = s_fast_band_edges[i - 1];
        }
    }
    s_fast_band_edges[0] = (s_fast_band_edges[0] < 1) ? 1 : s_fast_band_edges[0];
    s_fast_band_edges[8] = FFT_FAST_BINS;
    s_fast_band_init = 1;
}

/**
 * Compute FFT512 using esp-dsp (Phase 3B).
 * Input: samples[FFT_SIZE] (real samples)
 * Output: out_mag[FFT_SIZE/2] (magnitude spectrum)
 * REQUIRES: esp-dsp must be linked (build fails if missing).
 */
static void compute_fft_512(const float *samples, float *out_mag)
{
    /* Initialize esp-dsp FFT if not done yet */
    const int fft_points = FFT_SIZE / 2;
    if (!s_fft_init_done) {
        dsp_fft_init_max();
        if (!s_fft_init_done) {
            memset(out_mag, 0, (FFT_SIZE / 2) * sizeof(float));
            return;
        }
    }
    
    /* Ensure window is initialized */
    if (!s_fft_window_init) {
        fft_window_init();
    }
    
    /* Apply window and pack real samples for FFT */
    for (int i = 0; i < FFT_SIZE; i++) {
        s_fft_input[i] = samples[i] * s_fft_window[i];
    }

    /* Run FFT using esp-dsp */
    dsps_fft4r_fc32(s_fft_input, fft_points);
    dsps_bit_rev4r_fc32(s_fft_input, fft_points);
    /* Convert complex FFT output for real input (required for real input FFTs) */
    dsps_cplx2real_fc32(s_fft_input, fft_points);
    
    /* Compute magnitude spectrum from interleaved complex output */
    /* After cplx2real, data is still interleaved: data[i*2+0]=real, data[i*2+1]=imag */
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = s_fft_input[i * 2];
        float imag = s_fft_input[i * 2 + 1];
        out_mag[i] = sqrtf(real * real + imag * imag);
    }
}

static void compute_fft_128(const float *samples, float *out_mag)
{
    const int fft_points = FFT_FAST_SIZE / 2;
    if (!s_fft_init_done) {
        dsp_fft_init_max();
        if (!s_fft_init_done) {
            memset(out_mag, 0, FFT_FAST_BINS * sizeof(float));
            return;
        }
    }

    if (!s_fast_fft_window_init) {
        fast_fft_window_init();
    }

    for (int i = 0; i < FFT_FAST_SIZE; i++) {
        s_fast_fft_input[i] = samples[i] * s_fast_fft_window[i];
    }

    dsps_fft4r_fc32(s_fast_fft_input, fft_points);
    dsps_bit_rev4r_fc32(s_fast_fft_input, fft_points);
    dsps_cplx2real_fc32(s_fast_fft_input, fft_points);

    for (int i = 0; i < FFT_FAST_BINS; i++) {
        float real = s_fast_fft_input[i * 2];
        float imag = s_fast_fft_input[i * 2 + 1];
        out_mag[i] = sqrtf(real * real + imag * imag);
    }
}

static float compute_slow_flux(const float *mag, int n)
{
    float flux = 0.f;
    if (!s_prev_slow_mag_valid) {
        memcpy(s_prev_slow_mag, mag, n * sizeof(float));
        s_prev_slow_mag_valid = 1;
        return 0.f;
    }
    for (int i = 0; i < n; i++) {
        float diff = mag[i] - s_prev_slow_mag[i];
        if (diff > 0.f) {
            flux += diff;
        }
        s_prev_slow_mag[i] = mag[i];
    }
    return flux;
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
    
    /* Onset detection: spectral flux from FFT128 magnitude */
    compute_fft_128(hop, s_fast_fft_mag);
    float mag_sum = 0.f;
    float flux = 0.f;
    for (int i = 0; i < FFT_FAST_BINS; i++) {
        float mag = s_fast_fft_mag[i];
        float diff = mag - s_fast_fft_prev_mag[i];
        if (diff > 0.f) {
            flux += diff;
        }
        s_fast_fft_prev_mag[i] = mag;
        mag_sum += mag;
    }
    f->onset_strength = (mag_sum > 1e-9f) ? (flux / mag_sum) : 0.f;

    /* Band energy [8]: FFT-based energy in coarse bands */
    if (!s_fast_band_init) {
        fast_band_init();
    }
    memset(f->band_energy, 0, sizeof(f->band_energy));
    for (int b = 0; b < 8; b++) {
        int start = s_fast_band_edges[b];
        int end = s_fast_band_edges[b + 1];
        if (end <= start) {
            end = start + 1;
        }
        float band_sum = 0.f;
        for (int i = start; i < end && i < FFT_FAST_BINS; i++) {
            float mag = s_fast_fft_mag[i];
            band_sum += mag * mag;
        }
        f->band_energy[b] = band_sum;
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
        /* Explicit read size: 128 samples * 2 bytes (16-bit mono) = 256 bytes */
        esp_err_t ret = i2s_channel_read(s_rx_handle, s_i2s_raw, BYTES_PER_HOP, &bytes_read, portMAX_DELAY);
        audio_capture_reads_count++;
        
        /* Diagnostic: log first read to verify cadence */
        static bool first_read_logged = false;
        if (!first_read_logged) {
            unsigned samples_read = (unsigned)(bytes_read / sizeof(int16_t));
            ESP_LOGI(TAG, "I2S first read: requested=%u bytes, received=%u bytes, samples=%u (expected=%u)",
                     (unsigned)BYTES_PER_HOP, (unsigned)bytes_read, samples_read, (unsigned)AUDIO_HOP_SIZE);
            first_read_logged = true;
        }
        
        flags = 0;
        if (ret != ESP_OK || bytes_read < BYTES_PER_HOP) {
            /* Underrun: fill with zeros, set flag */
            memset(s_hop_buf, 0, sizeof(s_hop_buf));
            flags |= AUDIO_FLAG_UNDERRUN;
        } else {
            /* Convert int16_t samples to float [-1.0, 1.0] (I2S configured for 16-bit mono) */
            const int16_t *samples = (const int16_t *)s_i2s_raw;
            unsigned samples_read = (unsigned)(bytes_read / sizeof(int16_t));
            if (samples_read > AUDIO_HOP_SIZE) {
                samples_read = AUDIO_HOP_SIZE;  /* Clamp to expected size */
            }
            for (unsigned i = 0; i < samples_read; i++) {
                s_hop_buf[i] = (float)samples[i] / 32768.f;
            }
            /* Zero-pad if short read */
            if (samples_read < AUDIO_HOP_SIZE) {
                memset(&s_hop_buf[samples_read], 0, (AUDIO_HOP_SIZE - samples_read) * sizeof(float));
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

        s_last_capture_flags = flags;
        
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
        /* Flags: include latest capture flags + slow lane stale */
        f->flags = s_last_capture_flags;
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
        if (s_last_slow.tempo_conf < 0.2f) {
            f->flags |= AUDIO_FLAG_TEMPO_UNCERTAIN;
        }
        
        uint64_t t_pub = (uint64_t)esp_timer_get_time();
        audio_t_publish_us = t_pub;
        audio_t_visual_commit_us = t_pub;  /* Stub: visual consumer will update */
        
        uint32_t audio_lat_us = (uint32_t)(t_pub - t_cap);
        uint32_t e2e_lat_us = (uint32_t)(t_pub - t_cap);
        
        /* Phase 1: Feed frame to responsiveness harness for latency tracking */
        responsiveness_feed_frame(f, t_pub);
        
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
    goertzel_init_coeffs();

    while (1) {
        /* Wait for notification from capture task (every N hops) using counting notifications */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        /* Start timer AFTER wait returns - measure compute time only */
        uint64_t t_start_us = esp_timer_get_time();
        
        /* Check for stale detection: if notification arrives too late */
        if (last_process_time_us > 0) {
            uint64_t time_since_last = t_start_us - last_process_time_us;
            if (time_since_last > (expected_cadence_us * 2)) {
                /* Missed cadence - mark as stale */
                audio_slow_lane_stale_count++;
            }
        }

        /* Read 512 samples from ring buffer history (for FFT512, 75% overlap) */
        ring_read_history(&s_ring, s_slow_fft_buf, 0, FFT_SIZE);

        /* FFT512: compute spectral features (centroid, rolloff, flatness) */
        compute_fft_512(s_slow_fft_buf, s_fft_mag);
        float slow_flux = compute_slow_flux(s_fft_mag, FFT_SIZE / 2);
        
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

        /* Goertzel1024: compute 64 musical bins (A1-C7) */
        ring_read_history(&s_ring, s_slow_history_buf, 0, GOERTZEL_SIZE);
        for (int i = 0; i < 64; i++) {
            s_last_slow.goertzel_bins[i] = goertzel_magnitude(s_slow_history_buf, GOERTZEL_SIZE, s_goertzel_coeffs[i]);
        }

        /* Chromagram: aggregate Goertzel bins into 12 pitch classes */
        float chroma_sum = 0.f;
        float chroma_max = 0.f;
        memset(s_last_slow.chroma, 0, sizeof(s_last_slow.chroma));
        for (int i = 0; i < 64; i++) {
            int pitch_class = (9 + i) % 12;  /* A=9 when C=0 */
            float mag = s_last_slow.goertzel_bins[i];
            s_last_slow.chroma[pitch_class] += mag;
        }
        for (int i = 0; i < 12; i++) {
            chroma_sum += s_last_slow.chroma[i];
            if (s_last_slow.chroma[i] > chroma_max) {
                chroma_max = s_last_slow.chroma[i];
            }
        }
        if (chroma_sum > 1e-9f) {
            for (int i = 0; i < 12; i++) {
                s_last_slow.chroma[i] /= chroma_sum;
            }
            s_last_slow.chroma_conf = chroma_max / chroma_sum;
        } else {
            s_last_slow.chroma_conf = 0.f;
        }

        /* Tempo/beat: Emotiscope-parity (1024 novelty, 96-bin Goertzel 48–144 BPM) */
        tempo_emotiscope_feed(slow_flux, 0.f);
        tempo_emotiscope_get(&s_last_slow.tempo_bpm, &s_last_slow.tempo_conf,
                             &s_last_slow.beat_phase_0_1);
        s_last_slow.beat_conf = s_last_slow.tempo_conf;

        /* Stabilize: raw -> tempo_out (octave resolver + hysteresis); publish stabilized BPM */
        tempo_stabilizer_update(s_last_slow.tempo_bpm, s_last_slow.tempo_conf);
        s_last_slow.tempo_bpm = tempo_stabilizer_get_bpm();

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

void audio_producer_start(bool enable_slow_lane)
{
    memset(&s_published, 0, sizeof(s_published));
    memset(&s_last_slow, 0, sizeof(s_last_slow));
    memset(s_rms_history, 0, sizeof(s_rms_history));
    s_published.version = AUDIO_FRAME_VERSION;
    tempo_emotiscope_init();
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
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = AUDIO_I2S_MCK_GPIO,
            .bclk = AUDIO_I2S_BCK_GPIO,
            .ws = AUDIO_I2S_WS_GPIO,
            .dout = AUDIO_I2S_DO_GPIO,
            .din = AUDIO_I2S_DI_GPIO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = AUDIO_I2S_MCLK_MULTIPLE;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
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
    ESP_LOGI(TAG, "I2S sample rate: %u Hz (from AUDIO_FS)", (unsigned)AUDIO_FS);
    ESP_LOGI(TAG, "I2S data width: 16-bit, slot mode: MONO, bytes per sample: %u", (unsigned)sizeof(int16_t));
    ESP_LOGI(TAG, "Expected bytes per hop: %u (samples=%u * bytes_per_sample=%u)", 
             (unsigned)BYTES_PER_HOP, (unsigned)AUDIO_HOP_SIZE, (unsigned)sizeof(int16_t));
    err = audio_codec_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "es8311 init failed %d", err);
    }

    /* Initialize diagnostics */
    audio_diag_start_time_us = esp_timer_get_time();

    /* Initialize responsiveness harness (Phase 1) */
    responsiveness_init();

    /* Initialize DSP tables once before tasks start (avoid first-use race between lanes) */
    dsp_fft_init_max();
    
    /* Create tasks with handles for event-driven notifications */
    /* Pin capture to CPU0 (highest priority, mostly blocked on DMA) */
    xTaskCreatePinnedToCore(capture_task, "capture", 3072, NULL, 5, &s_capture_handle, 0);
    /* Pin fast_lane to CPU1 to reduce CPU0 load and IDLE0 starvation */
    xTaskCreatePinnedToCore(fast_lane_task, "fast_lane", 3072, NULL, 4, &s_fast_lane_handle, 1);
    
    /* Phase 3B: slow_lane with FFT512 - only if DSP self-test passed */
    if (enable_slow_lane) {
        xTaskCreatePinnedToCore(slow_lane_task, "slow_lane", 12288, NULL, 3, &s_slow_lane_handle, 1);
        ESP_LOGI(TAG, "producer started (real I2S capture, counting notifications, fast_lane on CPU1, slow_lane ENABLED with FFT512)");
    } else {
        ESP_LOGW(TAG, "producer started (real I2S capture, counting notifications, fast_lane on CPU1, slow_lane DISABLED (DSP self-test failed))");
    }
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

void audio_producer_log_debug_summary(void)
{
    uint64_t now_us = esp_timer_get_time();
    
    /* Copy frame quickly and release lock */
    AudioFrame frame;
    audio_producer_get_latest(&frame);
    
    /* Calculate age_ms for staleness detection */
    uint32_t age_ms = (now_us > frame.t_capture_us) ? (uint32_t)((now_us - frame.t_capture_us) / 1000) : 0;
    
    /* Find top band */
    int top_band_idx = 0;
    float top_band_energy = frame.band_energy[0];
    for (int i = 1; i < 8; i++) {
        if (frame.band_energy[i] > top_band_energy) {
            top_band_energy = frame.band_energy[i];
            top_band_idx = i;
        }
    }
    
    /* Find top Goertzel and top chroma */
    int top_goertzel_idx = 0;
    float top_goertzel_mag = frame.goertzel_bins[0];
    for (int i = 1; i < 64; i++) {
        if (frame.goertzel_bins[i] > top_goertzel_mag) {
            top_goertzel_mag = frame.goertzel_bins[i];
            top_goertzel_idx = i;
        }
    }
    
    int top_chroma_idx = 0;
    float top_chroma_mag = frame.chroma[0];
    for (int i = 1; i < 12; i++) {
        if (frame.chroma[i] > top_chroma_mag) {
            top_chroma_mag = frame.chroma[i];
            top_chroma_idx = i;
        }
    }
    
    float top_goertzel_freq = s_goertzel_freq_hz[top_goertzel_idx];
    int underrun = (frame.flags & AUDIO_FLAG_UNDERRUN) ? 1 : 0;
    int clip = (frame.flags & AUDIO_FLAG_CLIPPED) ? 1 : 0;
    
    /* ANSI red for tempo so it stands out: \033[31m ... \033[0m */
    ESP_LOGI(TAG, "debug | rms=%.4f peak=%.4f noise=%.4f onset=%.4f top_band=%d=%.4f centroid=%.1fHz rolloff=%.1fHz flatness=%.4f top_goertzel=%d=%.1fHz=%.4f top_chroma=%d=%.4f \033[31mtempo=%.1fBPM\033[0m conf=%.3f beat=%.3f underrun=%d clip=%d age_ms=%lu",
             frame.vu_rms, frame.vu_peak, frame.noise_floor, frame.onset_strength,
             top_band_idx, top_band_energy,
             frame.centroid, frame.rolloff, frame.flatness,
             top_goertzel_idx, top_goertzel_freq, top_goertzel_mag,
             top_chroma_idx, top_chroma_mag,
             frame.tempo_bpm, frame.tempo_conf, frame.beat_phase_0_1,
             underrun, clip, (unsigned long)age_ms);
}
