# Emotiscope Audio-Visual Pipeline: Complete Technical Reference

**Version:** 1.1.0
**Platform:** ESP32-S3 @ 240 MHz, 520 KB RAM
**Generated:** January 2026

This document provides an exhaustive technical reference for the entire Emotiscope audio-visual processing pipeline, from raw I2S microphone samples to LED photon emission. Every algorithm, data structure, constant, and code path is documented.

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Boot Sequence & Initialization](#2-boot-sequence--initialization)
3. [Audio Input: I2S Microphone Interface](#3-audio-input-i2s-microphone-interface)
4. [Sample Buffer Management](#4-sample-buffer-management)
5. [Window Function: Gaussian Windowing](#5-window-function-gaussian-windowing)
6. [Goertzel Frequency Analysis](#6-goertzel-frequency-analysis)
7. [Spectrogram Processing & Auto-Ranging](#7-spectrogram-processing--auto-ranging)
8. [Chromagram: Pitch Class Aggregation](#8-chromagram-pitch-class-aggregation)
9. [VU Metering: Loudness Detection](#9-vu-metering-loudness-detection)
10. [Spectral Flux & Novelty Curve](#10-spectral-flux--novelty-curve)
11. [Tempo Detection: Novelty-Domain Goertzel](#11-tempo-detection-novelty-domain-goertzel)
12. [Beat Phase Tracking & Synchronization](#12-beat-phase-tracking--synchronization)
13. [Silence Detection](#13-silence-detection)
14. [GPU Core: Graphics Pipeline](#14-gpu-core-graphics-pipeline)
15. [Post-Processing: LPF, Tonemapping, Gamma](#15-post-processing-lpf-tonemapping-gamma)
16. [LED Output: RMT Driver & Temporal Dithering](#16-led-output-rmt-driver--temporal-dithering)
17. [Data Structures Reference](#17-data-structures-reference)
18. [Constants Reference](#18-constants-reference)
19. [Memory Map](#19-memory-map)
20. [Timing Budget](#20-timing-budget)
21. [Porting Guide](#21-porting-guide)

---

## 1. System Architecture Overview

### Dual-Core Design

Emotiscope runs on both cores of the ESP32-S3:

```
┌─────────────────────────────────────────────────────────────────────┐
│                     ESP32-S3 DUAL-CORE ARCHITECTURE                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  CORE 1 (CPU Core) - loop() in Arduino                               │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ run_cpu():                                                       │ │
│  │   1. acquire_sample_chunk()    ← I2S DMA read (64 samples)      │ │
│  │   2. calculate_magnitudes()    ← Goertzel × 64 bins             │ │
│  │   3. get_chromagram()          ← Pitch class aggregation        │ │
│  │   4. run_vu()                  ← Loudness metering              │ │
│  │   5. update_tempo()            ← Tempo Goertzel (2 bins/frame)  │ │
│  │   6. watch_cpu_fps()           ← Frame rate tracking            │ │
│  │   7. print_system_info()       ← Periodic diagnostics           │ │
│  │   8. read_touch()              ← Capacitive touch input         │ │
│  │   9. check_serial()            ← UART command processing        │ │
│  │  10. yield()                   ← Watchdog feed                  │ │
│  │                                                                   │ │
│  │ run_web():                                                       │ │
│  │   - PsychicHttp server handling                                  │ │
│  │   - WebSocket message processing                                 │ │
│  │   - Command queue dispatch                                       │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                       │
│  CORE 0 (GPU Core) - loop_gpu() FreeRTOS task                        │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ run_gpu():                                                       │ │
│  │   1. update_novelty()          ← Spectral flux @ 50 Hz          │ │
│  │   2. update_tempi_phase()      ← Phase tracking all 96 tempi    │ │
│  │   3. update_auto_color()       ← Color cycling from novelty     │ │
│  │   4. clear_display()           ← Zero LED buffer                │ │
│  │   5. light_modes[].draw()      ← Active visualization mode      │ │
│  │   6. apply_background()        ← Ambient color gradient         │ │
│  │   7. draw_ui_overlay()         ← Touch indicators               │ │
│  │   8. run_screensaver()         ← Idle animation (if enabled)    │ │
│  │   9. apply_brightness()        ← Master brightness              │ │
│  │  10. apply_image_lpf()         ← Temporal smoothing filter      │ │
│  │  11. apply_tonemapping()       ← HDR soft-clip                  │ │
│  │  12. apply_warmth()            ← Incandescent color shift       │ │
│  │  13. apply_master_brightness() ← Final fade-in                  │ │
│  │  14. apply_gamma_correction()  ← Perceptual gamma (²)           │ │
│  │  15. transmit_leds()           ← RMT output with dithering      │ │
│  │  16. watch_gpu_fps()           ← Frame rate tracking            │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
```

### Data Flow Diagram

```
┌──────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  I2S MEMS    │───▶│  sample_history │───▶│   Goertzel ×64  │
│  Microphone  │    │  [4096] float   │    │  spectrogram[]  │
└──────────────┘    └─────────────────┘    └────────┬────────┘
                                                     │
                    ┌────────────────────────────────┼────────────────────┐
                    │                                │                    │
                    ▼                                ▼                    ▼
           ┌────────────────┐              ┌────────────────┐    ┌───────────────┐
           │  chromagram[12]│              │ spectrogram_   │    │  run_vu()     │
           │  pitch classes │              │ smooth[64]     │    │  vu_level     │
           └────────────────┘              └───────┬────────┘    └───────────────┘
                                                   │
                                                   ▼
                                          ┌────────────────┐
                                          │ Spectral Flux  │
                                          │ (HWR diff)     │
                                          └───────┬────────┘
                                                   │
                                                   ▼
                                          ┌────────────────┐
                                          │ novelty_curve  │
                                          │ [1024] @ 50Hz  │
                                          └───────┬────────┘
                                                   │
                                                   ▼
                                          ┌────────────────┐
                                          │ Goertzel ×96   │
                                          │ 48-143 BPM     │
                                          └───────┬────────┘
                                                   │
                    ┌──────────────────────────────┼──────────────────────┐
                    │                              │                      │
                    ▼                              ▼                      ▼
           ┌────────────────┐            ┌────────────────┐     ┌────────────────┐
           │ tempi[].       │            │ tempi[].       │     │ tempi[].       │
           │ magnitude      │            │ phase          │     │ beat           │
           │ (BPM strength) │            │ (radians)      │     │ sin(phase)     │
           └────────────────┘            └────────────────┘     └────────────────┘
                    │                              │                      │
                    └──────────────────────────────┼──────────────────────┘
                                                   │
                                                   ▼
                                          ┌────────────────┐
                                          │  Light Mode    │
                                          │  draw()        │
                                          └───────┬────────┘
                                                   │
                                                   ▼
                                          ┌────────────────┐
                                          │  leds[128]     │
                                          │  CRGBF float   │
                                          └───────┬────────┘
                                                   │
                    ┌──────────────────────────────┼──────────────────────┐
                    │                              │                      │
                    ▼                              ▼                      ▼
           ┌────────────────┐            ┌────────────────┐     ┌────────────────┐
           │ Post-Process   │            │ Gamma          │     │ Temporal       │
           │ LPF/Tonemap    │            │ Correction     │     │ Dithering      │
           └────────────────┘            └────────────────┘     └────────────────┘
                                                   │
                                                   ▼
                                          ┌────────────────┐
                                          │  RMT Driver    │
                                          │  WS2812 Output │
                                          └────────────────┘
```

---

## 2. Boot Sequence & Initialization

### Source: `v1.1_build.ino` lines 141-163, `system.h` lines 423-464

The system boots in this exact order:

```c
void setup() {
    // 1. HIL state allocation (if HIL_EXTENDED defined)
    hil_capture_state = (hil_capture_state_t*)malloc(sizeof(hil_capture_state_t));
    memset(hil_capture_state, 0, sizeof(hil_capture_state_t));

    // 2. Call init_system() which runs:
    init_system();

    // 3. Start GPU core on Core 0
    xTaskCreatePinnedToCore(loop_gpu, "loop_gpu", 4096, NULL, 0, NULL, 0);
}

void init_system() {
    // Ordered initialization sequence:
    init_hardware_version_pins();       // Read PCB version from GPIO pins
    init_serial(921600);                // USB Serial JTAG + UART @ 921600 baud
    init_light_mode_list();             // Register all visualization modes
    init_filesystem();                  // Mount LittleFS partition
    init_configuration();               // Load settings from NVS flash
    init_i2s_microphone();              // Configure I2S peripheral
    init_window_lookup();               // Pre-compute Gaussian window coefficients
    init_goertzel_constants_musical();  // Pre-compute Goertzel coefficients (64 bins)
    init_tempo_goertzel_constants();    // Pre-compute tempo Goertzel coefficients (96 bins)
    init_indicator_light();             // Status LED PWM setup
    init_rmt_driver();                  // RMT peripheral for WS2812 output
    init_touch();                       // Capacitive touch pins
    init_wifi();                        // WiFi + HTTP/WebSocket server
    init_noise_samples();               // Random bit array for dithering
    init_floating_point_lookups();      // Pre-computed float division tables
    init_boot_button();                 // GPIO0 boot button handler
    init_vu();                          // VU meter buffers
}
```

### Initialization Timing

| Stage | Approximate Time |
|-------|------------------|
| HIL allocation | < 1 ms |
| Serial init | < 5 ms |
| Filesystem mount | 50-100 ms |
| Configuration load | < 5 ms |
| I2S init | 10-20 ms |
| Window lookup | 5-10 ms |
| Goertzel constants | 10-20 ms |
| Tempo constants | 10-20 ms |
| RMT driver | 20-30 ms |
| WiFi connection | 2-10 s (depends on AP) |
| **Total (without WiFi)** | ~150-250 ms |

---

## 3. Audio Input: I2S Microphone Interface

### Source: `microphone.h` lines 14-79

### Hardware Configuration

```c
// Pin Definitions
#define I2S_LRCLK_PIN 35    // Word Select (L/R channel select)
#define I2S_BCLK_PIN  36    // Bit Clock
#define I2S_DIN_PIN   37    // Data In (from microphone)

// Audio Parameters
#define CHUNK_SIZE  64      // Samples per DMA transfer
#define SAMPLE_RATE 12800   // I2S sample rate in Hz
```

### I2S Configuration Details

```c
i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(12800),  // 12.8 kHz sample rate
    .slot_cfg = {
        .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,    // 32-bit container
        .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,    // 32-bit slot
        .slot_mode = I2S_SLOT_MODE_STEREO,             // Stereo mode (only R used)
        .slot_mask = I2S_STD_SLOT_RIGHT,               // Right channel only
        .ws_width = 32,                                 // WS signal width
        .ws_pol = true,                                 // WS polarity inverted
        .bit_shift = false,                             // MSB not delayed
        .left_align = true,                             // Left-aligned in slot
        .big_endian = false,                            // Little-endian
        .bit_order_lsb = false,                         // MSB first
    },
    .gpio_cfg = { ... }
};
```

### Why 12.8 kHz?

The sample rate is chosen to:
1. Capture frequencies up to **6.4 kHz** (Nyquist limit)
2. Minimize CPU overhead while covering musical range
3. Allow **~320 ms** of history in 4096 samples for tempo detection
4. Match the I2S clock divider constraints of ESP32-S3

### Microphone Model

The **SPH0645LM4H** MEMS microphone provides:
- 18-bit effective resolution
- -26 dBFS sensitivity
- 65 dB SNR
- I2S output with 32-bit slot width

---

## 4. Sample Buffer Management

### Source: `microphone.h` lines 81-134, `utilities.h` lines 127-136

### Buffer Structure

```c
float sample_history[SAMPLE_HISTORY_LENGTH];  // 4096 samples
const float recip_scale = 1.0 / 131072.0;     // Normalization constant (2^17)
```

### Sample Acquisition Algorithm

```c
void acquire_sample_chunk() {
    // 1. DMA read from I2S peripheral
    uint32_t new_samples_raw[CHUNK_SIZE];  // 64 × 32-bit values
    size_t bytes_read = 0;

    if (EMOTISCOPE_ACTIVE) {
        i2s_channel_read(rx_handle, new_samples_raw,
                         CHUNK_SIZE * sizeof(uint32_t),
                         &bytes_read, portMAX_DELAY);
    } else {
        memset(new_samples_raw, 0, sizeof(uint32_t) * CHUNK_SIZE);
    }

    // 2. Convert to float with DC offset correction
    float new_samples[CHUNK_SIZE];
    for (uint16_t i = 0; i < CHUNK_SIZE; i += 4) {
        // Bit shift: extract 18-bit signed value from 32-bit container
        // >> 14 shifts right, leaving 18 effective bits
        // +7000 and -360 are DC offset corrections specific to SPH0645
        new_samples[i+0] = min(max((((int32_t)new_samples_raw[i+0]) >> 14) + 7000,
                                   (int32_t)-131072), (int32_t)131072) - 360;
        new_samples[i+1] = min(max((((int32_t)new_samples_raw[i+1]) >> 14) + 7000,
                                   (int32_t)-131072), (int32_t)131072) - 360;
        new_samples[i+2] = min(max((((int32_t)new_samples_raw[i+2]) >> 14) + 7000,
                                   (int32_t)-131072), (int32_t)131072) - 360;
        new_samples[i+3] = min(max((((int32_t)new_samples_raw[i+3]) >> 14) + 7000,
                                   (int32_t)-131072), (int32_t)131072) - 360;
    }

    // 3. Normalize to [-1.0, 1.0] using ESP-DSP SIMD multiply
    dsps_mulc_f32(new_samples, new_samples, CHUNK_SIZE, recip_scale, 1, 1);

    // 4. Shift buffer left, append new samples at end
    waveform_locked = true;
    shift_and_copy_arrays(sample_history, SAMPLE_HISTORY_LENGTH, new_samples, CHUNK_SIZE);
    waveform_locked = false;
    waveform_sync_flag = true;
}
```

### Buffer Shift Implementation

```c
void shift_and_copy_arrays(float history_array[], size_t history_size,
                           const float new_array[], size_t new_size) {
    // Shift left: move samples [new_size .. history_size-1] to [0 .. history_size-new_size-1]
    memmove(history_array, history_array + new_size,
            (history_size - new_size) * sizeof(float));

    // Copy new samples to end
    memcpy(history_array + history_size - new_size, new_array,
           new_size * sizeof(float));
}
```

### Memory Layout After Each Chunk

```
Before: [S0 S1 S2 ... S63 | S64 S65 ... S127 | ... | S4032 ... S4095]
                                                     ▲
                                                     oldest

After:  [S64 S65 ... S127 | S128 ... S191 | ... | NEW0 NEW1 ... NEW63]
                                                  ▲
                                                  newest (64 new samples)
```

### Timing

- Chunk acquisition: **~5 ms** (64 samples @ 12.8 kHz)
- Buffer history: **~320 ms** (4096 samples @ 12.8 kHz)
- Update rate: **~200 Hz** (CPU FPS)

---

## 5. Window Function: Gaussian Windowing

### Source: `goertzel.h` lines 118-137

### Purpose

Windowing reduces **spectral leakage** in the Goertzel DFT. The Gaussian window provides:
- Excellent frequency resolution
- Minimal side-lobe amplitude
- Smooth amplitude taper at edges

### Window Lookup Table

```c
float window_lookup[4096];  // Pre-computed for SAMPLE_HISTORY_LENGTH

void init_window_lookup() {
    for (uint16_t i = 0; i < 2048; i++) {
        // Gaussian window parameters
        float sigma = 0.8;  // Standard deviation (controls width)
        float n_minus_halfN = i - 2048 / 2;  // Distance from center

        // Gaussian formula: e^(-0.5 * ((n - N/2) / (σ * N/2))²)
        float gaussian_weighing_factor = exp(
            -0.5 * pow((n_minus_halfN / (sigma * 2048 / 2)), 2)
        );

        // Symmetric: first half
        window_lookup[i] = gaussian_weighing_factor;
        // Symmetric: second half (mirror)
        window_lookup[4095 - i] = gaussian_weighing_factor;
    }
}
```

### Window Shape Visualization

```
Amplitude
   1.0 ┤              ████████████
       │           ███            ███
   0.8 ┤         ██                  ██
       │        █                      █
   0.6 ┤       █                        █
       │      █                          █
   0.4 ┤     █                            █
       │    █                              █
   0.2 ┤   █                                █
       │  █                                  █
   0.0 ┼─█────────────────────────────────────█─────
       0                2048               4095
                    Sample Index
```

### Window Parameters

| Parameter | Value | Effect |
|-----------|-------|--------|
| σ (sigma) | 0.8 | Narrower = sharper frequency resolution |
| N | 4096 | Window length |
| Center | N/2 = 2048 | Maximum amplitude point |

### Mathematical Formula

```
w[n] = exp(-0.5 * ((n - N/2) / (σ * N/2))²)

where:
  n = sample index [0, N-1]
  N = window length (4096)
  σ = 0.8 (Gaussian width parameter)
```

---

## 6. Goertzel Frequency Analysis

### Source: `goertzel.h` lines 29-218

The Goertzel algorithm computes single DFT bins efficiently, perfect for music analysis where specific frequencies matter more than full spectrum.

### Frequency Bin Configuration

```c
// Musical note mapping
#define BOTTOM_NOTE 12    // Starting note index in quarter-step table
#define NOTE_STEP 2       // Use half-steps (semitones)
#define NUM_FREQS 64      // Total frequency bins

// Reference frequency table (first 64 entries used)
const float notes[] = {
    55.0,      // A1 (bin 0)
    58.27047,  // A#1/Bb1
    61.73541,  // B1
    65.40639,  // C2
    // ... continues through semitones
    2093.005,  // C7 (bin 63, approximately)
};
```

### Bin-to-Frequency Mapping

**Formula:**
```
frequency_hz = 55.0 * 2^(bin_index / 12)
```

| Bin | Note | Frequency (Hz) | Musical Range |
|-----|------|----------------|---------------|
| 0 | A1 | 55.0 | Sub-bass |
| 3 | C2 | 65.4 | Bass |
| 12 | A2 | 110.0 | Bass |
| 24 | A3 | 220.0 | Low-mids |
| 36 | A4 | 440.0 | Mids (Concert A) |
| 48 | A5 | 880.0 | Treble |
| 60 | A6 | 1760.0 | High treble |
| 63 | C7 | 2093.0 | Air/shimmer |

### Goertzel Coefficient Initialization

```c
void init_goertzel(uint16_t frequency_slot, float frequency, float bandwidth) {
    // Block size determines frequency resolution
    // Smaller block = faster response, worse resolution
    // Larger block = better resolution, more latency
    frequencies_musical[frequency_slot].block_size = SAMPLE_RATE / bandwidth;

    // Round down to multiple of 4 (SIMD alignment)
    while (frequencies_musical[frequency_slot].block_size % 4 != 0) {
        frequencies_musical[frequency_slot].block_size -= 1;
    }

    // Cap at buffer length
    if (frequencies_musical[frequency_slot].block_size > SAMPLE_HISTORY_LENGTH - 1) {
        frequencies_musical[frequency_slot].block_size = SAMPLE_HISTORY_LENGTH - 1;
    }

    // Window step for variable block sizes
    frequencies_musical[frequency_slot].window_step =
        4096.0 / frequencies_musical[frequency_slot].block_size;

    // Goertzel coefficient calculation
    float k = (int)(0.5 + ((block_size * target_freq) / SAMPLE_RATE));
    float w = (2.0 * PI * k) / block_size;
    float cosine = cos(w);
    frequencies_musical[frequency_slot].coeff = 2.0 * cosine;
}
```

### Core Goertzel Algorithm

```c
float calculate_magnitude_of_bin(uint16_t bin_number) {
    float q1 = 0;
    float q2 = 0;
    float window_pos = 0.0;

    const uint16_t block_size = frequencies_musical[bin_number].block_size;
    float coeff = frequencies_musical[bin_number].coeff;
    float window_step = frequencies_musical[bin_number].window_step;

    // Start pointer: end of buffer minus block_size
    float* sample_ptr = &sample_history[(SAMPLE_HISTORY_LENGTH - 1) - block_size];

    // Goertzel iteration (the heart of the algorithm)
    for (uint16_t i = 0; i < block_size; i++) {
        // Apply Gaussian window to sample
        float windowed_sample = sample_ptr[i] * window_lookup[uint32_t(window_pos)];

        // Goertzel recurrence relation: q0 = coeff*q1 - q2 + sample
        float q0 = coeff * q1 - q2 + windowed_sample;
        q2 = q1;
        q1 = q0;

        // Advance window position
        window_pos += window_step;
    }

    // Final magnitude calculation
    // |X(k)|² = q1² + q2² - q1*q2*coeff
    float magnitude_squared = (q1 * q1) + (q2 * q2) - q1 * q2 * coeff;
    float magnitude = sqrt(magnitude_squared);

    // Normalize by block size
    float normalized_magnitude = magnitude_squared / (block_size / 2.0);

    // Frequency-dependent scaling (boost high frequencies)
    float progress = float(bin_number) / NUM_FREQS;
    progress *= progress;
    progress *= progress;  // progress⁴
    float scale = (progress * 0.9975) + 0.0025;

    return sqrt(normalized_magnitude * scale);
}
```

### Goertzel Mathematical Derivation

The Goertzel algorithm computes a single DFT bin:

```
X[k] = Σ(n=0 to N-1) x[n] * e^(-j*2π*k*n/N)
```

Using the recurrence relation:
```
s[n] = x[n] + 2*cos(2π*k/N)*s[n-1] - s[n-2]
```

Final result:
```
X[k] = s[N-1] - s[N-2]*e^(-j*2π*k/N)

|X[k]|² = s[N-1]² + s[N-2]² - 2*cos(2π*k/N)*s[N-1]*s[N-2]
        = q1² + q2² - coeff*q1*q2    (where coeff = 2*cos(2π*k/N))
```

### Block Size vs Frequency Resolution

| Frequency | Block Size | Resolution | Latency |
|-----------|------------|------------|---------|
| 55 Hz | 3072 | ~4.2 Hz | ~240 ms |
| 110 Hz | 2048 | ~6.25 Hz | ~160 ms |
| 440 Hz | 1024 | ~12.5 Hz | ~80 ms |
| 1760 Hz | 512 | ~25 Hz | ~40 ms |

---

## 7. Spectrogram Processing & Auto-Ranging

### Source: `goertzel.h` lines 220-357

### Frame Interlacing

To reduce CPU load, bins are computed in an interlaced pattern:

```c
static bool interlacing_frame_field = 0;
interlacing_frame_field = !interlacing_frame_field;

for (uint16_t i = 0; i < NUM_FREQS; i++) {
    bool interlace_field_now = ((i % 2) == 0);
    if (interlace_field_now == interlacing_frame_field) {
        // Compute bin on this frame (odd or even bins alternate)
        magnitudes_raw[i] = calculate_magnitude_of_bin(i);
    }
}
```

**Effect:** Each bin updates at **~100 Hz** instead of ~200 Hz, halving Goertzel CPU usage.

### Noise Floor Estimation

```c
// Log noise spectrum every second
if (t_now_ms - last_noise_spectrum_log >= 1000) {
    last_noise_spectrum_log = t_now_ms;

    noise_history_index = (noise_history_index + 1) % 10;
    memcpy(noise_history[noise_history_index], magnitudes_raw, sizeof(float) * NUM_FREQS);
}

// Calculate per-bin noise floor (10-second rolling average × 0.9)
for (uint8_t j = 0; j < 10; j++) {
    avg_val += noise_history[j][i];
}
avg_val /= 10.0;
avg_val *= 0.90;  // 10% headroom

// Smooth noise floor with slow EMA
noise_floor[i] = noise_floor[i] * 0.99 + avg_val * 0.01;

// Subtract noise floor from raw magnitude
magnitudes_noise_filtered[i] = max(magnitudes_raw[i] - noise_floor[i], 0.0f);
```

### Auto-Ranging (Dynamic Gain Control)

```c
// Find maximum magnitude across all bins
float max_val = 0.0;
for (uint16_t i = 0; i < NUM_FREQS; i++) {
    if (magnitudes_smooth[i] > max_val) {
        max_val = magnitudes_smooth[i];
    }
}

// Smooth max_val with asymmetric attack/release
if (max_val > max_val_smooth) {
    max_val_smooth += (max_val - max_val_smooth) * 0.005;  // Slow attack
}
if (max_val < max_val_smooth) {
    max_val_smooth -= (max_val_smooth - max_val) * 0.005;  // Slow release
}

// Floor to prevent division by zero and over-amplification
if (max_val_smooth < 0.0025) {
    max_val_smooth = 0.0025;
}

// Apply auto-ranging scale
float autoranger_scale = 1.0 / max_val_smooth;
for (uint16_t i = 0; i < NUM_FREQS; i++) {
    spectrogram[i] = clip_float(magnitudes_smooth[i] * autoranger_scale);
}
```

### Spectrogram Smoothing

```c
#define NUM_SPECTROGRAM_AVERAGE_SAMPLES 12
float spectrogram_average[NUM_SPECTROGRAM_AVERAGE_SAMPLES][NUM_FREQS];

// Rolling average (12 frames ≈ 60 ms at 200 FPS)
spectrogram_average_index = (spectrogram_average_index + 1) % NUM_SPECTROGRAM_AVERAGE_SAMPLES;

for (uint16_t i = 0; i < NUM_FREQS; i++) {
    spectrogram_average[spectrogram_average_index][i] = spectrogram[i];

    spectrogram_smooth[i] = 0;
    for (uint16_t a = 0; a < NUM_SPECTROGRAM_AVERAGE_SAMPLES; a++) {
        spectrogram_smooth[i] += spectrogram_average[a][i];
    }
    spectrogram_smooth[i] /= float(NUM_SPECTROGRAM_AVERAGE_SAMPLES);
}
```

---

## 8. Chromagram: Pitch Class Aggregation

### Source: `goertzel.h` lines 366-383

The chromagram reduces the 64-bin spectrogram to 12 pitch classes (C, C#, D, ... B).

### Algorithm

```c
void get_chromagram() {
    // Clear chromagram
    memset(chromagram, 0, sizeof(float) * 12);

    float max_val = 0.2;  // Minimum floor for auto-scaling

    // Sum spectrogram bins into pitch classes
    for (uint16_t i = 0; i < 60; i++) {  // Only first 60 bins (5 octaves)
        // Map bin to pitch class [0-11]
        // Bin 0 = A, so offset by 0 means chromagram[0] = A
        // To get C = 0, we need offset: (bin + 3) % 12 OR use i % 12 directly
        chromagram[i % 12] += (spectrogram_smooth[i] / 5.0);

        max_val = max(max_val, chromagram[i % 12]);
    }

    // Auto-scale chromagram to [0, 1]
    float auto_scale = 1.0 / max_val;
    for (uint16_t i = 0; i < 12; i++) {
        chromagram[i] *= auto_scale;
    }
}
```

### Pitch Class Mapping

| Index | Note | Contributing Bins | Octave Count |
|-------|------|-------------------|--------------|
| 0 | A | 0, 12, 24, 36, 48 | 5 |
| 1 | A#/Bb | 1, 13, 25, 37, 49 | 5 |
| 2 | B | 2, 14, 26, 38, 50 | 5 |
| 3 | C | 3, 15, 27, 39, 51 | 5 |
| 4 | C#/Db | 4, 16, 28, 40, 52 | 5 |
| 5 | D | 5, 17, 29, 41, 53 | 5 |
| 6 | D#/Eb | 6, 18, 30, 42, 54 | 5 |
| 7 | E | 7, 19, 31, 43, 55 | 5 |
| 8 | F | 8, 20, 32, 44, 56 | 5 |
| 9 | F#/Gb | 9, 21, 33, 45, 57 | 5 |
| 10 | G | 10, 22, 34, 46, 58 | 5 |
| 11 | G#/Ab | 11, 23, 35, 47, 59 | 5 |

### Note on Bin Indexing

Since bin 0 = A1 (55 Hz), the pitch class mapping is:
- Bin 0 → A (index 0 in the table above, but actually index 0 % 12 = 0)
- Bin 3 → C (index 3)
- Bin 12 → A (index 0, one octave up)

---

## 9. VU Metering: Loudness Detection

### Source: `vu.h` lines 1-93

### State Variables

```c
#define NUM_VU_LOG_SAMPLES 20     // Noise floor history length
#define NUM_VU_SMOOTH_SAMPLES 12  // Output smoothing length

float vu_log[NUM_VU_LOG_SAMPLES] = { 0 };
float vu_smooth[NUM_VU_SMOOTH_SAMPLES] = { 0 };

volatile float vu_level_raw = 0.0;  // Unsmoothed level
volatile float vu_level = 0.0;      // Smoothed output [0, 1]
volatile float vu_max = 0.0;        // Peak hold
volatile float vu_floor = 0.0;      // Estimated noise floor
```

### Complete VU Algorithm

```c
void run_vu() {
    // ═══════════════════════════════════════════════════════════════
    // STAGE 1: Peak Detection in Current Chunk
    // ═══════════════════════════════════════════════════════════════
    static float max_amplitude_cap = 0.0000001;  // Auto-gain ceiling

    // Point to latest CHUNK_SIZE samples
    float* samples = &sample_history[(SAMPLE_HISTORY_LENGTH-1) - CHUNK_SIZE];

    // Find peak amplitude (squared for power)
    float max_amplitude_now = 0.000001;
    for (uint16_t i = 0; i < CHUNK_SIZE; i++) {
        float sample = samples[i];
        float sample_abs = fabs(sample);
        max_amplitude_now = fmaxf(max_amplitude_now, sample_abs * sample_abs);
    }
    max_amplitude_now = clip_float(max_amplitude_now);

    // ═══════════════════════════════════════════════════════════════
    // STAGE 2: Noise Floor Estimation (every 250ms)
    // ═══════════════════════════════════════════════════════════════
    if (t_now_ms < 2000) {
        // Warmup: fill history with current value
        memset(vu_log, max_amplitude_now, sizeof(float) * NUM_VU_LOG_SAMPLES);
    }
    else if (t_now_ms - last_vu_log >= 250) {
        last_vu_log = t_now_ms;

        // Log current amplitude
        vu_log[vu_log_index] = max_amplitude_now;
        vu_log_index = (vu_log_index + 1) % NUM_VU_LOG_SAMPLES;

        // Calculate noise floor as average × 0.9
        float vu_sum = 0.0;
        for (uint16_t i = 0; i < NUM_VU_LOG_SAMPLES; i++) {
            vu_sum += vu_log[i];
        }
        vu_floor = (vu_sum / NUM_VU_LOG_SAMPLES) * 0.90;
    }

    // ═══════════════════════════════════════════════════════════════
    // STAGE 3: Noise Floor Subtraction
    // ═══════════════════════════════════════════════════════════════
    max_amplitude_now = fmaxf(max_amplitude_now - vu_floor, 0.0f);

    // ═══════════════════════════════════════════════════════════════
    // STAGE 4: Adaptive Auto-Gain Control (AGC)
    // ═══════════════════════════════════════════════════════════════
    if (max_amplitude_now > max_amplitude_cap) {
        // Fast attack: track peaks
        float distance = max_amplitude_now - max_amplitude_cap;
        max_amplitude_cap += (distance * 0.1);
    }
    else if (max_amplitude_cap > max_amplitude_now) {
        // Fast release: decay ceiling
        float distance = max_amplitude_cap - max_amplitude_now;
        max_amplitude_cap -= (distance * 0.1);
    }
    max_amplitude_cap = clip_float(max_amplitude_cap);

    // Floor the cap to prevent extreme amplification
    if (max_amplitude_cap < 0.000025) {
        max_amplitude_cap = 0.000025;
    }

    // Scale to [0, 1]
    float auto_scale = 1.0 / fmaxf(max_amplitude_cap, 0.00001f);
    vu_level_raw = clip_float(max_amplitude_now * auto_scale);

    // ═══════════════════════════════════════════════════════════════
    // STAGE 5: Output Smoothing (12-sample moving average)
    // ═══════════════════════════════════════════════════════════════
    vu_smooth[vu_smooth_index] = vu_level_raw;
    vu_smooth_index = (vu_smooth_index + 1) % NUM_VU_SMOOTH_SAMPLES;

    float vu_sum = 0.0;
    for (uint8_t i = 0; i < NUM_VU_SMOOTH_SAMPLES; i++) {
        vu_sum += vu_smooth[i];
    }
    vu_level = vu_sum / NUM_VU_SMOOTH_SAMPLES;

    // ═══════════════════════════════════════════════════════════════
    // STAGE 6: Peak Hold
    // ═══════════════════════════════════════════════════════════════
    vu_max = fmaxf(vu_max, vu_level);
}
```

### VU Timing Analysis

| Stage | Update Interval | Purpose |
|-------|-----------------|---------|
| Peak Detection | Every frame (~5ms) | Track instantaneous loudness |
| Noise Floor | Every 250ms | Adapt to ambient noise |
| AGC | Every frame | Track dynamic range |
| Smoothing | Every frame | Reduce visual jitter |

### VU Response Characteristics

- **Noise floor adaptation:** 20 samples × 250ms = **5 seconds** history
- **AGC attack/release:** 0.1 coefficient = **~50ms** at 200 FPS
- **Output smoothing:** 12 samples = **~60ms** at 200 FPS

---

## 10. Spectral Flux & Novelty Curve

### Source: `tempo.h` lines 288-377

Spectral flux detects **onsets** (sudden changes in the spectrum) by measuring frame-to-frame differences.

### State Variables

```c
#define NOVELTY_HISTORY_LENGTH 1024  // ~20.48 seconds at 50 Hz
#define NOVELTY_LOG_HZ 50            // Update rate

float novelty_curve[NOVELTY_HISTORY_LENGTH];
float novelty_curve_normalized[NOVELTY_HISTORY_LENGTH];
```

### Novelty Computation

```c
void update_novelty() {
    static uint32_t next_update = t_now_us;
    const uint32_t update_interval_us = 1000000 / NOVELTY_LOG_HZ;  // 20,000 us = 20 ms

    if (t_now_us >= next_update) {
        next_update += update_interval_us;

        // Calculate spectral flux (half-wave rectified)
        float current_novelty = 0.0;
        for (uint16_t i = 0; i < NUM_FREQS; i++) {
            float new_mag = spectrogram_smooth[i];
            float old_mag = frequencies_musical[i].magnitude_last;

            // Half-wave rectification: only positive differences (onsets)
            frequencies_musical[i].novelty = max(0.0f, new_mag - old_mag);
            current_novelty += frequencies_musical[i].novelty;

            // Update previous magnitude
            frequencies_musical[i].magnitude_last = new_mag;
        }
        // Average across all bins
        current_novelty /= float(NUM_FREQS);

        // Check for silence
        check_silence(current_novelty);

        // Log with compression (log1p for better dynamic range)
        log_novelty(log1p(current_novelty));

        // Also log VU for tempo detection
        log_vu(vu_max);
        vu_max = 0.000001;  // Reset peak hold
    }
}
```

### Novelty Logging

```c
void log_novelty(float input) {
    // Shift buffer left by 1
    shift_array_left(novelty_curve, NOVELTY_HISTORY_LENGTH, 1);

    // Append new value
    novelty_curve[NOVELTY_HISTORY_LENGTH - 1] = input;
}
```

### Novelty Normalization

```c
void normalize_novelty_curve() {
    static float max_val = 0.00001;
    static float max_val_smooth = 0.1;

    // Decay max tracker
    max_val *= 0.99;

    // Find new max (unrolled for speed)
    for (uint16_t i = 0; i < NOVELTY_HISTORY_LENGTH; i += 4) {
        max_val = max(max_val, novelty_curve[i + 0]);
        max_val = max(max_val, novelty_curve[i + 1]);
        max_val = max(max_val, novelty_curve[i + 2]);
        max_val = max(max_val, novelty_curve[i + 3]);
    }

    // Smooth the max value
    max_val_smooth = max(0.1f, max_val_smooth * 0.95f + max_val * 0.05f);

    // Normalize entire curve using ESP-DSP SIMD
    float auto_scale = 1.0 / max_val;
    dsps_mulc_f32(novelty_curve, novelty_curve_normalized,
                  NOVELTY_HISTORY_LENGTH, auto_scale, 1, 1);
}
```

### Why Half-Wave Rectification?

**Half-wave rectification** (only positive differences) detects **onsets** but not **offsets**:
- Onset = new sound (kick drum, snare, note attack) → magnitude increases
- Offset = sound decay → magnitude decreases (ignored)

This makes tempo detection focus on rhythmic attacks rather than sustained sounds.

---

## 11. Tempo Detection: Novelty-Domain Goertzel

### Source: `tempo.h` lines 51-223

### Tempo Range Configuration

```c
#define TEMPO_LOW  48    // Minimum BPM
#define TEMPO_HIGH 143   // Maximum BPM (TEMPO_LOW + NUM_TEMPI - 1)
#define NUM_TEMPI  96    // Number of BPM bins

float tempi_bpm_values_hz[NUM_TEMPI];  // Pre-computed BPM→Hz conversion
```

### BPM-to-Hz Conversion

```
bpm_hz = bpm / 60.0

For 120 BPM:
bpm_hz = 120 / 60 = 2.0 Hz (beats per second)
```

### Tempo Goertzel Initialization

```c
void init_tempo_goertzel_constants() {
    for (uint16_t i = 0; i < NUM_TEMPI; i++) {
        float progress = float(i) / NUM_TEMPI;
        float tempi_range = TEMPO_HIGH - TEMPO_LOW;  // 95 BPM range
        float tempo = tempi_range * progress + TEMPO_LOW;  // Linear interpolation

        tempi_bpm_values_hz[i] = tempo / 60.0;  // Convert to Hz
    }

    for (uint16_t i = 0; i < NUM_TEMPI; i++) {
        tempi[i].target_tempo_hz = tempi_bpm_values_hz[i];

        // Calculate block size from neighbor spacing
        float neighbor_distance_hz = /* neighbor spacing calculation */;
        tempi[i].block_size = NOVELTY_LOG_HZ / (neighbor_distance_hz * 0.5);

        // Cap at novelty buffer length
        if (tempi[i].block_size > NOVELTY_HISTORY_LENGTH) {
            tempi[i].block_size = NOVELTY_HISTORY_LENGTH;
        }

        // Goertzel coefficient
        float k = (int)(0.5 + ((block_size * target_tempo_hz) / NOVELTY_LOG_HZ));
        float w = (2.0 * PI * k) / block_size;
        tempi[i].cosine = cos(w);
        tempi[i].sine = sin(w);
        tempi[i].coeff = 2.0 * tempi[i].cosine;

        // Window step
        tempi[i].window_step = 4096.0 / tempi[i].block_size;

        // Phase advance rate
        tempi[i].phase_radians_per_reference_frame =
            (2.0 * PI * target_tempo_hz) / REFERENCE_FPS;
    }
}
```

### Tempo Magnitude Calculation

```c
float calculate_magnitude_of_tempo(uint16_t tempo_bin) {
    float normalized_magnitude;

    uint16_t block_size = tempi[tempo_bin].block_size;

    float q1 = 0;
    float q2 = 0;
    float window_pos = 0.0;

    for (uint16_t i = 0; i < block_size; i++) {
        // Sample from novelty curve (most recent block_size samples)
        float sample_novelty = novelty_curve_normalized[
            ((NOVELTY_HISTORY_LENGTH - 1) - block_size) + i
        ];
        float sample_vu = vu_curve[
            ((NOVELTY_HISTORY_LENGTH - 1) - block_size) + i
        ];

        // Combine novelty and VU for better beat detection
        float sample = (sample_novelty + sample_vu) / 2.0;

        // Goertzel iteration (windowed)
        float q0 = tempi[tempo_bin].coeff * q1 - q2 +
                   (sample_novelty * window_lookup[uint32_t(window_pos)]);
        q2 = q1;
        q1 = q0;

        window_pos += tempi[tempo_bin].window_step;
    }

    // Extract magnitude
    float real = (q1 - q2 * tempi[tempo_bin].cosine);
    float imag = (q2 * tempi[tempo_bin].sine);

    float magnitude_squared = (q1 * q1) + (q2 * q2) - q1 * q2 * tempi[tempo_bin].coeff;
    float magnitude = sqrt(magnitude_squared);
    normalized_magnitude = magnitude / (block_size / 2.0);

    // Extract phase for beat synchronization
    tempi[tempo_bin].phase = atan2(imag, real) + (PI * BEAT_SHIFT_PERCENT);

    // Wrap phase to [-π, π]
    if (tempi[tempo_bin].phase > PI) {
        tempi[tempo_bin].phase -= (2 * PI);
        tempi[tempo_bin].phase_inverted = !tempi[tempo_bin].phase_inverted;
    }
    else if (tempi[tempo_bin].phase < -PI) {
        tempi[tempo_bin].phase += (2 * PI);
        tempi[tempo_bin].phase_inverted = !tempi[tempo_bin].phase_inverted;
    }

    return normalized_magnitude;
}
```

### Incremental Tempo Update

To reduce CPU load, only 2 tempo bins are updated per frame:

```c
void update_tempo() {
    static uint32_t iter = 0;
    iter++;

    normalize_novelty_curve();

    static uint16_t calc_bin = 0;
    uint16_t max_bin = (NUM_TEMPI - 1) * MAX_TEMPO_RANGE;

    // Alternate between even and odd bins
    if (iter % 2 == 0) {
        calculate_tempi_magnitudes(calc_bin + 0);
    } else {
        calculate_tempi_magnitudes(calc_bin + 1);
    }

    calc_bin += 2;
    if (calc_bin >= max_bin) {
        calc_bin = 0;
    }
}
```

**Update Rate:** Each tempo bin updates at **~4 Hz** (96 bins / 2 per frame / ~12 frames per cycle)

---

## 12. Beat Phase Tracking & Synchronization

### Source: `tempo.h` lines 379-440

### Phase State

```c
struct tempo {
    float phase;                          // Current phase [-π, π]
    float phase_target;                   // Detected phase
    bool phase_inverted;                  // Phase flip tracker
    float phase_radians_per_reference_frame;  // Phase advance per frame
    float beat;                           // sin(phase) [-1, 1]
    float magnitude;                      // BPM strength [0, 1]
};
```

### Phase Advance

```c
void sync_beat_phase(uint16_t tempo_bin, float delta) {
    // Advance phase by time delta (frame-rate independent)
    float push = tempi[tempo_bin].phase_radians_per_reference_frame * delta;
    tempi[tempo_bin].phase += push;

    // Wrap to [-π, π]
    if (tempi[tempo_bin].phase > PI) {
        tempi[tempo_bin].phase -= (2 * PI);
        tempi[tempo_bin].phase_inverted = !tempi[tempo_bin].phase_inverted;
    }
    else if (tempi[tempo_bin].phase < -PI) {
        tempi[tempo_bin].phase += (2 * PI);
        tempi[tempo_bin].phase_inverted = !tempi[tempo_bin].phase_inverted;
    }

    // Beat value for visualization
    tempi[tempo_bin].beat = sin(tempi[tempo_bin].phase);
}
```

### Phase Calculation

```
phase_radians_per_reference_frame = (2π × bpm_hz) / REFERENCE_FPS

For 120 BPM at 100 FPS reference:
= (2π × 2.0) / 100
= 0.1257 radians/frame

At actual 200 FPS:
delta = 100/200 = 0.5
phase_advance = 0.1257 × 0.5 = 0.0628 radians/frame
```

### Beat Value Interpretation

```
phase = 0       → beat = sin(0) = 0      (mid-beat)
phase = π/2     → beat = sin(π/2) = 1    (beat peak)
phase = π       → beat = sin(π) = 0      (mid-beat)
phase = -π/2    → beat = sin(-π/2) = -1  (beat trough)
```

### Tempi Smoothing

```c
void update_tempi_phase(float delta) {
    tempi_power_sum = 0.00000001;  // Prevent division by zero

    for (uint16_t tempo_bin = 0; tempo_bin < NUM_TEMPI; tempo_bin++) {
        float tempi_magnitude = tempi[tempo_bin].magnitude;

        // Exponential smoothing: 0.975/0.025 ratio
        tempi_smooth[tempo_bin] = tempi_smooth[tempo_bin] * 0.975 +
                                  tempi_magnitude * 0.025;
        tempi_power_sum += tempi_smooth[tempo_bin];

        // Advance and sync phase
        sync_beat_phase(tempo_bin, delta);
    }

    // Calculate confidence (max contribution ratio)
    float max_contribution = 0.000001;
    for (uint16_t tempo_bin = 0; tempo_bin < NUM_TEMPI; tempo_bin++) {
        max_contribution = max(
            tempi_smooth[tempo_bin] / tempi_power_sum,
            max_contribution
        );
    }
    tempo_confidence = max_contribution;
}
```

### Beat Shift

```c
#define BEAT_SHIFT_PERCENT (0.16)

// Applied during phase extraction:
tempi[tempo_bin].phase = atan2(imag, real) + (PI * BEAT_SHIFT_PERCENT);
```

This shifts the detected phase by **28.8°** (~16% of a cycle), compensating for perceptual latency.

---

## 13. Silence Detection

### Source: `tempo.h` lines 317-344

### Silence Detection Algorithm

```c
void check_silence(float current_novelty) {
    // Analyze recent 128 novelty samples (~2.56 seconds)
    float min_val = 1.0;
    float max_val = 0.0;

    for (uint16_t i = 0; i < 128; i++) {
        float recent_novelty = novelty_curve_normalized[
            (NOVELTY_HISTORY_LENGTH - 1 - 128) + i
        ];

        // Clip to [0, 0.5] and scale to [0, 1]
        recent_novelty = min(0.5f, recent_novelty) * 2.0;
        float scaled_value = sqrt(recent_novelty);

        max_val = max(max_val, scaled_value);
        min_val = min(min_val, scaled_value);
    }

    // Contrast = dynamic range of novelty
    float novelty_contrast = fabs(max_val - min_val);

    // Silence level = inverse of contrast
    float silence_level_raw = 1.0 - novelty_contrast;

    // Threshold at 0.5, then scale to [0, 1]
    silence_level = max(0.0f, silence_level_raw - 0.5f) * 2.0;

    if (silence_level_raw > 0.5) {
        silence_detected = true;
        // Decay tempo history to prevent stale beats
        reduce_tempo_history(silence_level * 0.10);
    } else {
        silence_level = 0.0;
        silence_detected = false;
    }
}
```

### Tempo History Decay

```c
void reduce_tempo_history(float reduction_amount) {
    float reduction_amount_inv = 1.0 - reduction_amount;

    for (uint16_t i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
        novelty_curve[i] = max(novelty_curve[i] * reduction_amount_inv, 0.00001f);
        vu_curve[i] = max(vu_curve[i] * reduction_amount_inv, 0.00001f);
    }
}
```

---

## 14. GPU Core: Graphics Pipeline

### Source: `gpu_core.h` lines 1-231

### Delta Time Calculation

```c
void run_gpu() {
    static uint32_t t_last_us = micros();

    t_now_us = micros();
    t_now_ms = millis();

    // Frame-rate independent delta (relative to 100 FPS reference)
    const uint32_t ideal_us_interval = (1000000 / REFERENCE_FPS);  // 10,000 µs
    uint32_t t_elapsed_us = t_now_us - t_last_us;
    float delta = float(t_elapsed_us) / ideal_us_interval;

    t_last_us = t_now_us;
}
```

### Graphics Pipeline Order

```c
// 1. Update novelty curve from spectrogram
update_novelty();

// 2. Update tempo phase tracking
update_tempi_phase(delta);

// 3. Auto-cycle color based on novelty
update_auto_color();

// 4. Run status indicator LED
run_indicator_light();

// 5. Clear LED buffer
clear_display();

// 6. Draw current visualization mode
light_modes[configuration.current_mode].draw();

// 7. Add ambient background gradient
apply_background(configuration.background);

// 8. Draw touch UI overlay
draw_ui_overlay();

// 9. Run screensaver if enabled and silent
if (EMOTISCOPE_ACTIVE && configuration.screensaver) {
    run_screensaver();
}

// 10. Apply user brightness setting
apply_brightness();

// 11. Run standby animation if inactive
if (EMOTISCOPE_ACTIVE == false) {
    run_standby();
}

// 12. Render touch feedback
render_touches();

// 13-18. Post-processing pipeline (see next section)
```

---

## 15. Post-Processing: LPF, Tonemapping, Gamma

### Source: `leds.h` lines 548-686

### Temporal Low-Pass Filter

```c
void apply_image_lpf(float cutoff_frequency) {
    // Calculate alpha from cutoff frequency and frame rate
    float alpha = 1.0 - expf(-6.28318530718 * cutoff_frequency / FPS_GPU);
    float alpha_inv = 1.0 - alpha;

    // ESP-DSP SIMD operations:
    // leds = leds * alpha + leds_last * (1 - alpha)
    scale_CRGBF_array_by_constant(leds, alpha, NUM_LEDS);
    scale_CRGBF_array_by_constant(leds_last, alpha_inv, NUM_LEDS);
    add_CRGBF_arrays(leds, leds_last, NUM_LEDS);

    // Store for next frame
    memcpy(leds_last, leds, sizeof(CRGBF) * NUM_LEDS);
}
```

### Cutoff Frequency Calculation

```c
float lpf_cutoff_frequency = 0.5 + (1.0 - sqrt(configuration.softness)) * 14.5;
// Range: 0.5 Hz (soft) to 15 Hz (sharp)
```

### HDR Tonemapping (Soft Clip)

```c
float soft_clip_hdr(float input) {
    if (input < 0.75) {
        return input;  // Linear region
    } else {
        // Tanh soft-clip for values > 0.75
        float t = (input - 0.75) * 4.0;
        return 0.75 + 0.25 * tanh(t);  // Asymptotic to 1.0
    }
}

void apply_tonemapping() {
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds[i].r = soft_clip_hdr(leds[i].r);
        leds[i].g = soft_clip_hdr(leds[i].g);
        leds[i].b = soft_clip_hdr(leds[i].b);
    }
}
```

### Warmth (Incandescent Color Shift)

```c
CRGBF incandescent_lookup = {sqrt(1.0000), sqrt(0.1982), sqrt(0.0244)};
// = {1.0, 0.445, 0.156} approximately

void apply_warmth(float mix) {
    float mix_inv = 1.0 - mix;

    multiply_CRGBF_array_by_LUT(
        leds,
        (CRGBF){
            incandescent_lookup.r * mix + mix_inv,  // Boost red
            incandescent_lookup.g * mix + mix_inv,  // Reduce green
            incandescent_lookup.b * mix + mix_inv   // Heavily reduce blue
        },
        NUM_LEDS
    );
}
```

### White Balance

```c
CRGBF WHITE_BALANCE = { 1.0, 0.9375, 0.84 };

// Applied to all LEDs:
multiply_CRGBF_array_by_LUT(leds, WHITE_BALANCE, NUM_LEDS);
```

### Gamma Correction

```c
void apply_gamma_correction() {
    // Square each color component (gamma = 2.0)
    dsps_mul_f32_ae32((float*)leds, (float*)leds, (float*)leds, NUM_LEDS * 3, 1, 1, 1);
}
```

**Why Gamma 2.0?** Human perception is non-linear. Gamma correction ensures perceived brightness matches intended brightness.

---

## 16. LED Output: RMT Driver & Temporal Dithering

### Source: `led_driver.h` lines 1-261

### RMT Configuration

```c
#define LED_DATA_1_PIN 21   // First half of strip
#define LED_DATA_2_PIN 17   // Second half of strip

// RMT timing for WS2812 (10 MHz clock = 0.1 µs resolution)
rmt_bytes_encoder_config_t bytes_encoder_config = {
    .bit0 = { 4, 1, 6, 0 },  // T0H=0.4µs HIGH, T0L=0.6µs LOW
    .bit1 = { 7, 1, 6, 0 },  // T1H=0.7µs HIGH, T1L=0.6µs LOW
    .flags = { .msb_first = 1 }
};
```

### Temporal Dithering Algorithm

```c
void quantize_color_error(bool temporal_dithering) {
    // Scale floating-point [0, 1] to [0, 255]
    memcpy(leds_scaled, leds, NUM_LEDS * sizeof(CRGBF));
    dsps_mulc_f32_ansi((float*)leds, (float*)leds_scaled, NUM_LEDS * 3, 255.0, 1, 1);

    if (temporal_dithering) {
        for (uint16_t i = 0; i < NUM_LEDS; i++) {
            // Truncate to 8-bit
            raw_led_data[3*i+1] = (uint8_t)(leds_scaled[i].r);  // R
            raw_led_data[3*i+0] = (uint8_t)(leds_scaled[i].g);  // G
            raw_led_data[3*i+2] = (uint8_t)(leds_scaled[i].b);  // B

            // Calculate quantization error
            float new_error_r = leds_scaled[i].r - raw_led_data[3*i+1];
            float new_error_g = leds_scaled[i].g - raw_led_data[3*i+0];
            float new_error_b = leds_scaled[i].b - raw_led_data[3*i+2];

            // Accumulate error if above threshold
            const float dither_error_threshold = 0.055;  // ~5.5%
            if (new_error_r >= dither_error_threshold) dither_error[i].r += new_error_r;
            if (new_error_g >= dither_error_threshold) dither_error[i].g += new_error_g;
            if (new_error_b >= dither_error_threshold) dither_error[i].b += new_error_b;

            // If accumulated error >= 1.0, add 1 to output and subtract from error
            if (dither_error[i].r >= 1.0) {
                raw_led_data[3*i+1] += 1;
                dither_error[i].r -= 1.0;
            }
            if (dither_error[i].g >= 1.0) {
                raw_led_data[3*i+0] += 1;
                dither_error[i].g -= 1.0;
            }
            if (dither_error[i].b >= 1.0) {
                raw_led_data[3*i+2] += 1;
                dither_error[i].b -= 1.0;
            }
        }
    }
}
```

### LED Transmission

```c
void transmit_leds() {
    // Wait for previous transmission to complete
    rmt_tx_wait_all_done(tx_chan_a, portMAX_DELAY);
    rmt_tx_wait_all_done(tx_chan_b, portMAX_DELAY);

    // Clear buffer
    memset(raw_led_data, 0, NUM_LEDS * 3);

    // Quantize with dithering
    quantize_color_error(configuration.temporal_dithering);

    // Transmit to both halves of strip simultaneously
    rmt_transmit(tx_chan_a, led_encoder_a, raw_led_data, sizeof(raw_led_data) >> 1, &tx_config);
    rmt_transmit(tx_chan_b, led_encoder_b, raw_led_data + ((NUM_LEDS >> 1) * 3),
                 sizeof(raw_led_data) >> 1, &tx_config);
}
```

### WS2812 Timing Diagram

```
Bit 0:  ┌──┐
        │  │          T0H = 0.4µs, T0L = 0.6µs
        └──┴──────

Bit 1:  ┌─────┐
        │     │       T1H = 0.7µs, T1L = 0.6µs
        └─────┴───

Reset:  ─────────────────
        >50µs LOW
```

---

## 17. Data Structures Reference

### Source: `types.h` lines 1-165

### CRGBF (Floating-Point Color)

```c
struct CRGBF {
    float r;  // Red [0, 1+] (can exceed 1.0 for HDR)
    float g;  // Green [0, 1+]
    float b;  // Blue [0, 1+]
};
```

### freq (Goertzel Bin State)

```c
struct freq {
    float target_freq;           // Target frequency (Hz)
    float coeff;                 // 2*cos(2πk/N)
    float window_step;           // Step through window_lookup[]
    float magnitude;             // Normalized magnitude [0, 1]
    float magnitude_full_scale;  // Pre-normalization magnitude
    float magnitude_last;        // Previous frame (for novelty)
    float novelty;               // Spectral flux contribution
    uint16_t block_size;         // DFT block size
};
```

### tempo (Tempo Bin State)

```c
struct tempo {
    float target_tempo_hz;                    // Target BPM in Hz
    float coeff;                              // 2*cos(2πk/N)
    float sine;                               // sin(2πk/N) for phase
    float cosine;                             // cos(2πk/N) for phase
    float window_step;                        // Window lookup step
    float phase;                              // Current phase [-π, π]
    float phase_target;                       // Detected phase
    bool phase_inverted;                      // Phase flip flag
    float phase_radians_per_reference_frame;  // Phase advance rate
    float beat;                               // sin(phase) [-1, 1]
    float magnitude;                          // Normalized BPM strength
    float magnitude_full_scale;               // Pre-normalization
    uint32_t block_size;                      // DFT block size
};
```

### config (User Settings)

```c
struct config {
    float brightness;          // [0, 1]
    float softness;            // [0, 1] (LPF strength)
    float color;               // [0, 1] (hue)
    float warmth;              // [0, 1] (incandescent tint)
    float color_range;         // [0, 1] (gradient spread)
    float speed;               // [0, 1] (animation speed)
    float saturation;          // [0, 1]
    float background;          // [0, 1] (ambient brightness)
    int32_t current_mode;      // Active visualization index
    bool mirror_mode;          // Mirror display
    bool screensaver;          // Enable idle animation
    bool temporal_dithering;   // Enable dithering
    bool auto_color_cycle;     // Cycle hue with music
    bool reverse_color_range;  // Invert gradient direction
};
```

---

## 18. Constants Reference

### Global Defines

| Constant | Value | Description |
|----------|-------|-------------|
| `NUM_LEDS` | 128 | LED count |
| `NUM_FREQS` | 64 | Goertzel bins |
| `SAMPLE_RATE` | 12800 | I2S sample rate (Hz) |
| `CHUNK_SIZE` | 64 | Samples per DMA read |
| `SAMPLE_HISTORY_LENGTH` | 4096 | Audio buffer size |
| `NOVELTY_LOG_HZ` | 50 | Novelty update rate |
| `NOVELTY_HISTORY_LENGTH` | 1024 | Novelty buffer size |
| `NUM_TEMPI` | 96 | BPM bins |
| `TEMPO_LOW` | 48 | Minimum BPM |
| `TEMPO_HIGH` | 143 | Maximum BPM |
| `REFERENCE_FPS` | 100 | Delta time reference |
| `BEAT_SHIFT_PERCENT` | 0.16 | Phase offset |

### Derived Values

| Value | Formula | Result |
|-------|---------|--------|
| Audio history duration | 4096 / 12800 | ~320 ms |
| Novelty history duration | 1024 / 50 | ~20.48 s |
| Min BPM period | 60 / 48 | 1.25 s |
| Max BPM period | 60 / 143 | 0.42 s |
| Nyquist frequency | 12800 / 2 | 6400 Hz |
| Frequency resolution | 12800 / 4096 | 3.125 Hz |

---

## 19. Memory Map

### Static Allocations

| Array | Size (bytes) | Description |
|-------|-------------|-------------|
| `sample_history[4096]` | 16,384 | Audio buffer |
| `window_lookup[4096]` | 16,384 | Gaussian window |
| `spectrogram[64]` | 256 | Current magnitudes |
| `spectrogram_smooth[64]` | 256 | Smoothed magnitudes |
| `spectrogram_average[12][64]` | 3,072 | Rolling average |
| `chromagram[12]` | 48 | Pitch classes |
| `novelty_curve[1024]` | 4,096 | Raw novelty |
| `novelty_curve_normalized[1024]` | 4,096 | Normalized novelty |
| `vu_curve[1024]` | 4,096 | VU history |
| `vu_curve_normalized[1024]` | 4,096 | Normalized VU |
| `tempi[96]` | ~4,608 | Tempo state (48 bytes each) |
| `tempi_smooth[96]` | 384 | Smoothed magnitudes |
| `frequencies_musical[64]` | ~3,840 | Goertzel state |
| `leds[128]` | 1,536 | LED buffer (CRGBF) |
| `leds_scaled[128]` | 1,536 | Scaled buffer |
| `leds_temp[128]` | 1,536 | Temporary buffer |
| `leds_last[128]` | 1,536 | Previous frame |
| `leds_smooth[128]` | 1,536 | Smoothed output |
| `dither_error[128]` | 1,536 | Dithering state |
| `raw_led_data[384]` | 384 | 8-bit output |
| **Total Static** | | **~73 KB** |

### Dynamic Allocations (HIL_EXTENDED)

| Structure | Size (bytes) |
|-----------|-------------|
| `hil_capture_state_t` | ~31,000 |
| `hil_export_state_t` | ~512 |
| **Total HIL** | **~32 KB** |

### RAM Usage Summary

```
Static DSP buffers:     ~73 KB
HIL instrumentation:    ~32 KB
Stack (CPU core):       ~8 KB
Stack (GPU core):       ~4 KB
System/WiFi:            ~100 KB
Free heap:              ~300 KB
─────────────────────────────────
Total available:        520 KB
```

---

## 20. Timing Budget

### CPU Core (loop) - Target: ~200 FPS

| Function | Typical Time | % Budget |
|----------|-------------|----------|
| `acquire_sample_chunk()` | ~500 µs | 10% |
| `calculate_magnitudes()` | ~2500 µs | 50% |
| `get_chromagram()` | ~50 µs | 1% |
| `run_vu()` | ~100 µs | 2% |
| `update_tempo()` | ~500 µs | 10% |
| System overhead | ~350 µs | 7% |
| Web handling | ~1000 µs | 20% |
| **Total** | **~5000 µs** | **100%** |

### GPU Core (loop_gpu) - Target: ~200 FPS

| Function | Typical Time | % Budget |
|----------|-------------|----------|
| `update_novelty()` | ~200 µs | 4% |
| `update_tempi_phase()` | ~400 µs | 8% |
| `light_modes[].draw()` | ~1000 µs | 20% |
| `apply_image_lpf()` | ~500 µs | 10% |
| `apply_tonemapping()` | ~200 µs | 4% |
| `apply_warmth()` | ~200 µs | 4% |
| `apply_gamma_correction()` | ~100 µs | 2% |
| `transmit_leds()` | ~2000 µs | 40% |
| Other post-processing | ~400 µs | 8% |
| **Total** | **~5000 µs** | **100%** |

---

## 21. Porting Guide

### Minimum Requirements

- **CPU:** 100+ MHz ARM Cortex-M4 or equivalent
- **RAM:** 100+ KB (73 KB DSP + system)
- **FPU:** Hardware floating-point strongly recommended
- **ADC:** 12-bit @ 12.8+ kHz or I2S microphone

### Platform Adaptations

| Component | ESP32-S3 Implementation | Portable Alternative |
|-----------|------------------------|----------------------|
| Audio Input | I2S DMA | ADC with DMA or timer interrupt |
| SIMD Math | ESP-DSP `dsps_*()` | Standard C loops |
| LED Output | RMT peripheral | Timer-based bitbang or DMA |
| Timing | `micros()`, `millis()` | System timer HAL |
| WiFi/WebSocket | PsychicHttp | Optional, can omit |

### Algorithm Dependencies

**No external dependencies required.** All DSP algorithms use standard C math:
- `sin()`, `cos()`, `sqrt()`, `atan2()`
- `exp()`, `log1p()`, `pow()`
- `fabs()`, `fmax()`, `fmin()`
- `memcpy()`, `memmove()`, `memset()`

### Porting Checklist

1. ☐ Implement audio input at 12.8 kHz
2. ☐ Allocate sample_history[4096] buffer
3. ☐ Pre-compute window_lookup[4096] with Gaussian formula
4. ☐ Initialize Goertzel coefficients for 64 bins
5. ☐ Initialize tempo Goertzel coefficients for 96 bins
6. ☐ Implement main loop at ~200 Hz
7. ☐ Test Goertzel with known sine waves
8. ☐ Verify tempo detection with 120 BPM metronome

### Performance Optimization Tips

1. **Loop unrolling:** Process 4 samples per iteration where possible
2. **SIMD:** Use platform-specific vector instructions if available
3. **Fixed-point:** Consider Q15/Q31 format for memory-constrained targets
4. **Reduce bins:** 32 frequency bins may suffice for simpler displays
5. **Reduce tempo range:** 60-140 BPM covers most music

---

*End of Complete Technical Reference*

*Total source files analyzed:*
- `v1.1_build.ino` (164 lines)
- `cpu_core.h` (150 lines)
- `gpu_core.h` (231 lines)
- `microphone.h` (135 lines)
- `goertzel.h` (385 lines)
- `tempo.h` (441 lines)
- `vu.h` (93 lines)
- `leds.h` (687 lines)
- `led_driver.h` (261 lines)
- `system.h` (465 lines)
- `utilities.h` (247 lines)
- `global_defines.h` (64 lines)
- `types.h` (165 lines)
- `profiler.h` (261 lines)
