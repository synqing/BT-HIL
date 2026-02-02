# Emotiscope DSP Algorithm Reference

**Purpose:** Complete technical reference for porting Emotiscope's DSP algorithms to other platforms.

**Source Files:**
- `v1.1_build/microphone.h` - I2S audio capture
- `v1.1_build/goertzel.h` - Frequency analysis
- `v1.1_build/vu.h` - Loudness metering
- `v1.1_build/tempo.h` - Beat detection
- `v1.1_build/cpu_core.h` - Pipeline orchestration

---

## 1. Audio Input Pipeline

### Source: `microphone.h`

#### I2S Configuration
```
Sample Rate:     12,800 Hz
Bit Depth:       32-bit (18-bit effective after shift)
Chunk Size:      64 samples (~5ms per chunk)
Buffer Length:   4,096 samples (~320ms history)
```

#### Raw Sample Processing
```c
// 1. Read 32-bit I2S data
i2s_channel_read(rx_handle, new_samples_raw, CHUNK_SIZE * sizeof(uint32_t), ...);

// 2. Shift and clip to 18-bit signed range
sample = ((int32_t)raw >> 14) + 7000;  // DC offset correction
sample = clamp(sample, -131072, 131072) - 360;  // Final DC trim

// 3. Normalize to [-1.0, 1.0]
sample_float = sample / 131072.0;
```

#### Buffer Management
```c
// Shift history left, append new chunk at end
shift_and_copy_arrays(sample_history, 4096, new_samples, 64);
```

★ Insight ─────────────────────────────────────
- The +7000 and -360 values are hardware-specific DC offset corrections for the SPH0645 microphone
- 18-bit effective range gives ~108 dB dynamic range
- Chunk size of 64 samples at 12.8kHz = 5ms latency per update
─────────────────────────────────────────────────

---

## 2. Goertzel Frequency Analysis

### Source: `goertzel.h`

The Goertzel algorithm efficiently computes single DFT bins without full FFT overhead.

#### Frequency Bin Layout
```
Bins:        64 (semitone-spaced)
Base Note:   A1 = 55 Hz (bin 0)
Top Note:    C7 ≈ 2093 Hz (bin 63)
Formula:     freq_hz = 55.0 * pow(2.0, bin_index / 12.0)
```

#### Bin-to-Frequency Table
| Bin | Note | Frequency (Hz) |
|-----|------|----------------|
| 0   | A1   | 55.0           |
| 12  | A2   | 110.0          |
| 24  | A3   | 220.0          |
| 36  | A4   | 440.0          |
| 48  | A5   | 880.0          |
| 60  | A6   | 1760.0         |
| 63  | C7   | 2093.0         |

#### Window Function
Gaussian window with sigma = 0.8:
```c
float gaussian_window[SAMPLE_HISTORY_LENGTH];
const float GAUSSIAN_SIGMA = 0.8;

void build_gaussian_window() {
    float center = (SAMPLE_HISTORY_LENGTH - 1) / 2.0;
    float sum = 0.0;

    for (int i = 0; i < SAMPLE_HISTORY_LENGTH; i++) {
        float x = (i - center) / (GAUSSIAN_SIGMA * center);
        gaussian_window[i] = exp(-0.5 * x * x);
        sum += gaussian_window[i];
    }

    // Normalize
    for (int i = 0; i < SAMPLE_HISTORY_LENGTH; i++) {
        gaussian_window[i] /= sum;
    }
}
```

#### Goertzel Algorithm
```c
float calculate_magnitude_of_bin(uint8_t bin_index) {
    float target_freq = 55.0 * pow(2.0, bin_index / 12.0);
    float k = target_freq * SAMPLE_HISTORY_LENGTH / SAMPLE_RATE;
    float w = (2.0 * PI * k) / SAMPLE_HISTORY_LENGTH;
    float coeff = 2.0 * cos(w);

    float s0 = 0, s1 = 0, s2 = 0;

    for (int i = 0; i < SAMPLE_HISTORY_LENGTH; i++) {
        float windowed_sample = sample_history[i] * gaussian_window[i];
        s0 = windowed_sample + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    // Magnitude calculation
    float real = s1 - s2 * cos(w);
    float imag = s2 * sin(w);
    float magnitude = sqrt(real * real + imag * imag);

    return magnitude;
}
```

#### Spectrogram Smoothing
```c
// Exponential moving average (time-domain smoothing)
spectrogram_smooth[i] = spectrogram_smooth[i] * 0.7 + spectrogram[i] * 0.3;
```

★ Insight ─────────────────────────────────────
- Goertzel is O(N) per bin vs FFT's O(N log N) for all bins - better when you need specific frequencies
- The Gaussian window reduces spectral leakage, critical for music analysis
- Semitone spacing matches Western music's 12-TET scale perfectly
─────────────────────────────────────────────────

---

## 3. Chromagram (Pitch Class Energy)

### Source: `goertzel.h`

The chromagram collapses the spectrogram into 12 pitch classes (C, C#, D, ... B).

#### Algorithm
```c
void get_chromagram() {
    // Clear chromagram
    memset(chromagram, 0, sizeof(float) * 12);

    // Sum spectrogram bins into pitch classes
    for (int bin = 0; bin < NUM_FREQS; bin++) {
        // Map semitone bins to pitch class (0-11)
        // Bin 0 = A1, bin 3 = C2, etc.
        int pitch_class = (bin + 9) % 12;  // A=9 offset to get C=0
        chromagram[pitch_class] += spectrogram[bin];
    }
}
```

#### Pitch Class Mapping
| Index | Note | Bins Contributing |
|-------|------|-------------------|
| 0     | C    | 3, 15, 27, 39, 51, 63 |
| 1     | C#   | 4, 16, 28, 40, 52 |
| 2     | D    | 5, 17, 29, 41, 53 |
| 3     | D#   | 6, 18, 30, 42, 54 |
| 4     | E    | 7, 19, 31, 43, 55 |
| 5     | F    | 8, 20, 32, 44, 56 |
| 6     | F#   | 9, 21, 33, 45, 57 |
| 7     | G    | 10, 22, 34, 46, 58 |
| 8     | G#   | 11, 23, 35, 47, 59 |
| 9     | A    | 0, 12, 24, 36, 48, 60 |
| 10    | A#   | 1, 13, 25, 37, 49, 61 |
| 11    | B    | 2, 14, 26, 38, 50, 62 |

---

## 4. VU Meter (Loudness)

### Source: `vu.h`

Three-stage loudness computation with adaptive noise floor.

#### Stage 1: Peak Detection
```c
float max_amplitude_now = 0.0;
for (int i = 0; i < CHUNK_SIZE; i++) {
    float sample_abs = fabs(sample_history[end - CHUNK_SIZE + i]);
    max_amplitude_now = fmax(max_amplitude_now, sample_abs * sample_abs);
}
```

#### Stage 2: Noise Floor Tracking
```c
// Log amplitude every 250ms (20 samples over 5 seconds)
#define NUM_VU_LOG_SAMPLES 20

if (elapsed_ms >= 250) {
    vu_log[vu_log_index] = max_amplitude_now;
    vu_log_index = (vu_log_index + 1) % NUM_VU_LOG_SAMPLES;

    // Average = noise floor estimate
    float sum = 0;
    for (int i = 0; i < NUM_VU_LOG_SAMPLES; i++) {
        sum += vu_log[i];
    }
    vu_floor = (sum / NUM_VU_LOG_SAMPLES) * 0.90;  // 10% below average
}

// Remove noise floor
max_amplitude_now = fmax(max_amplitude_now - vu_floor, 0.0);
```

#### Stage 3: Auto-Scaling (AGC)
```c
// Adaptive ceiling tracker
static float max_amplitude_cap = 0.000001;

if (max_amplitude_now > max_amplitude_cap) {
    max_amplitude_cap += (max_amplitude_now - max_amplitude_cap) * 0.1;  // Fast attack
} else {
    max_amplitude_cap -= (max_amplitude_cap - max_amplitude_now) * 0.1;  // Fast release
}

// Floor the cap to prevent division issues
max_amplitude_cap = fmax(max_amplitude_cap, 0.000025);

// Scale to 0-1 range
vu_level_raw = max_amplitude_now / max_amplitude_cap;
```

#### Stage 4: Smoothing
```c
#define NUM_VU_SMOOTH_SAMPLES 12

vu_smooth[vu_smooth_index] = vu_level_raw;
vu_smooth_index = (vu_smooth_index + 1) % NUM_VU_SMOOTH_SAMPLES;

float sum = 0;
for (int i = 0; i < NUM_VU_SMOOTH_SAMPLES; i++) {
    sum += vu_smooth[i];
}
vu_level = sum / NUM_VU_SMOOTH_SAMPLES;  // Final output

// Peak hold
vu_max = fmax(vu_max, vu_level);
```

#### Output Variables
| Variable | Description |
|----------|-------------|
| `vu_level` | Smoothed loudness (0.0-1.0) |
| `vu_max` | Peak value since reset |
| `vu_floor` | Estimated noise floor |

★ Insight ─────────────────────────────────────
- The 0.9 multiplier on noise floor provides headroom above ambient noise
- AGC with 0.1 attack/release gives ~100ms response time at 200 FPS
- 12-sample smoothing = ~60ms at 200 FPS, reducing visual jitter
─────────────────────────────────────────────────

---

## 5. Spectral Flux (Novelty Curve)

### Source: `tempo.h`

Spectral flux measures frame-to-frame changes in the spectrogram - key for onset detection.

#### Spectral Flux Calculation
```c
float calculate_spectral_flux() {
    static float last_spectrogram[NUM_FREQS] = {0};
    float flux = 0.0;

    for (int i = 0; i < NUM_FREQS; i++) {
        // Half-wave rectification: only positive changes (onsets)
        float diff = spectrogram[i] - last_spectrogram[i];
        if (diff > 0) {
            flux += diff;
        }
        last_spectrogram[i] = spectrogram[i];
    }

    return flux;
}
```

#### Novelty Curve Buffer
```c
#define NOVELTY_HISTORY_LENGTH 1024
#define NOVELTY_LOG_HZ 50  // Update rate

float novelty_curve[NOVELTY_HISTORY_LENGTH];  // ~20.48 seconds
float novelty_curve_normalized[NOVELTY_HISTORY_LENGTH];

void update_novelty() {
    // Shift left, add new value
    shift_and_copy_arrays(novelty_curve, NOVELTY_HISTORY_LENGTH, &flux, 1);

    // Find max for normalization
    float max_novelty = 0.0001;
    for (int i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
        max_novelty = fmax(max_novelty, novelty_curve[i]);
    }

    // Normalize
    for (int i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
        novelty_curve_normalized[i] = novelty_curve[i] / max_novelty;
    }
}
```

★ Insight ─────────────────────────────────────
- Half-wave rectification detects onsets only, not offsets
- 50 Hz update rate with 1024 samples = 20.48 seconds of beat history
- This is enough for tempo detection down to ~3 BPM (not practical, but gives margin)
─────────────────────────────────────────────────

---

## 6. Tempo Detection (Novelty-Domain Goertzel)

### Source: `tempo.h`

Uses the same Goertzel algorithm on the novelty curve to find periodic beat patterns.

#### Tempo Range
```
BPM Range:   48-143 BPM (96 bins)
TEMPO_LOW:   48
TEMPO_HIGH:  143
NUM_TEMPI:   96
```

#### Tempo Goertzel
```c
float calculate_magnitude_of_tempo(uint8_t tempo_index) {
    float target_bpm = TEMPO_LOW + tempo_index;  // 48 + index
    float target_hz = target_bpm / 60.0;  // Convert BPM to Hz

    float k = target_hz * NOVELTY_HISTORY_LENGTH / NOVELTY_LOG_HZ;
    float w = (2.0 * PI * k) / NOVELTY_HISTORY_LENGTH;
    float coeff = 2.0 * cos(w);

    float s0 = 0, s1 = 0, s2 = 0;

    for (int i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
        float windowed = novelty_curve_normalized[i] * tempo_goertzel_window[i];
        s0 = windowed + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    // Return magnitude
    float real = s1 - s2 * cos(w);
    float imag = s2 * sin(w);
    return sqrt(real * real + imag * imag);
}
```

#### Phase Extraction
```c
float calculate_phase_of_tempo(uint8_t tempo_index) {
    // Same Goertzel computation...

    // Extract phase angle
    float real = s1 - s2 * cos(w);
    float imag = s2 * sin(w);
    return atan2(imag, real);  // Returns -PI to PI
}
```

#### Beat Synchronization
```c
void sync_beat_phase() {
    for (int i = 0; i < NUM_TEMPI; i++) {
        // Advance phase by BPM rate
        float bpm = TEMPO_LOW + i;
        float hz = bpm / 60.0;
        float phase_increment = (2.0 * PI * hz) / NOVELTY_LOG_HZ;

        tempi[i].phase += phase_increment;

        // Wrap to [-PI, PI]
        while (tempi[i].phase > PI) tempi[i].phase -= 2.0 * PI;
        while (tempi[i].phase < -PI) tempi[i].phase += 2.0 * PI;

        // Soft-sync towards detected phase
        float detected_phase = calculate_phase_of_tempo(i);
        float phase_error = detected_phase - tempi[i].phase;

        // Wrap error
        while (phase_error > PI) phase_error -= 2.0 * PI;
        while (phase_error < -PI) phase_error += 2.0 * PI;

        // Weighted by magnitude (strong tempos sync faster)
        tempi[i].phase += phase_error * tempi[i].magnitude * 0.1;

        // Beat value for visualization
        tempi[i].beat = sin(tempi[i].phase);  // -1 to 1
    }
}
```

#### Finding Dominant BPM
```c
int find_top_bpm() {
    float max_magnitude = 0;
    int top_index = 0;

    for (int i = 0; i < NUM_TEMPI; i++) {
        if (tempi[i].magnitude > max_magnitude) {
            max_magnitude = tempi[i].magnitude;
            top_index = i;
        }
    }

    return TEMPO_LOW + top_index;  // BPM value
}
```

★ Insight ─────────────────────────────────────
- Phase tracking creates persistent beat sync even during quiet passages
- The soft-sync (error * magnitude * 0.1) prevents sudden phase jumps
- sin(phase) gives smooth -1 to 1 beat value perfect for LED brightness modulation
─────────────────────────────────────────────────

---

## 7. DSP Pipeline Execution Order

### Source: `cpu_core.h` (loop_cpu)

```c
void loop_cpu() {
    while (true) {
        // 1. Acquire new audio samples (64 samples, ~5ms)
        acquire_sample_chunk();

        // 2. Compute Goertzel magnitudes (64 bins)
        calculate_magnitudes();

        // 3. Aggregate into chromagram (12 pitch classes)
        get_chromagram();

        // 4. Update VU meter
        run_vu();

        // 5. Update tempo detection
        update_tempo();

        // 6. HIL capture (copy all state atomically)
        #ifdef HIL_EXTENDED
        hil_capture_cpu_write_begin();
        // ... copy spectrogram, chromagram, VU, tempi to capture state
        hil_capture_cpu_write_end();
        #endif

        // Yield to other tasks
        vTaskDelay(1);
    }
}
```

---

## 8. Key Constants Reference

### Audio
| Constant | Value | Description |
|----------|-------|-------------|
| `SAMPLE_RATE` | 12800 | I2S sample rate (Hz) |
| `CHUNK_SIZE` | 64 | Samples per acquisition |
| `SAMPLE_HISTORY_LENGTH` | 4096 | Total buffer (~320ms) |

### Goertzel
| Constant | Value | Description |
|----------|-------|-------------|
| `NUM_FREQS` | 64 | Number of frequency bins |
| `GAUSSIAN_SIGMA` | 0.8 | Window function width |
| `BASE_FREQ` | 55.0 | A1 reference (Hz) |

### Novelty
| Constant | Value | Description |
|----------|-------|-------------|
| `NOVELTY_HISTORY_LENGTH` | 1024 | Novelty buffer size |
| `NOVELTY_LOG_HZ` | 50 | Update rate (Hz) |

### Tempo
| Constant | Value | Description |
|----------|-------|-------------|
| `TEMPO_LOW` | 48 | Minimum BPM |
| `TEMPO_HIGH` | 143 | Maximum BPM |
| `NUM_TEMPI` | 96 | BPM bins (48-143) |

### VU
| Constant | Value | Description |
|----------|-------|-------------|
| `NUM_VU_LOG_SAMPLES` | 20 | Noise floor history |
| `NUM_VU_SMOOTH_SAMPLES` | 12 | Output smoothing |

---

## 9. Porting Notes

### Memory Requirements
```
sample_history:           4096 * 4 = 16,384 bytes
spectrogram:              64 * 4   = 256 bytes
spectrogram_smooth:       64 * 4   = 256 bytes
chromagram:               12 * 4   = 48 bytes
novelty_curve:            1024 * 4 = 4,096 bytes
novelty_curve_normalized: 1024 * 4 = 4,096 bytes
tempi (mag/phase/beat):   96 * 12  = 1,152 bytes
gaussian_window:          4096 * 4 = 16,384 bytes
tempo_goertzel_window:    1024 * 4 = 4,096 bytes
──────────────────────────────────────────────
Total:                    ~46 KB static RAM
```

### CPU Budget (per frame at 200 FPS)
| Stage | Typical Cycles | Notes |
|-------|---------------|-------|
| I2S Read | ~10,000 | DMA-assisted |
| Goertzel × 64 | ~800,000 | Dominant cost |
| VU | ~5,000 | Lightweight |
| Spectral Flux | ~3,000 | Simple loop |
| Tempo × 96 | ~400,000 | Second largest |
| **Total** | ~1.2M | ~5ms at 240MHz |

### Platform Adaptation
1. **Audio Input**: Replace I2S with platform ADC/codec
2. **Timing**: Use platform timer for 50Hz novelty updates
3. **Math**: Standard C math library (no SIMD required)
4. **Threads**: Single-threaded possible (merge CPU/GPU loops)

---

## 10. Quick Reference Formulas

### Frequency from Bin
```
freq_hz = 55.0 * pow(2.0, bin_index / 12.0)
```

### Bin from Frequency
```
bin_index = 12.0 * log2(freq_hz / 55.0)
```

### BPM from Tempo Index
```
bpm = TEMPO_LOW + tempo_index = 48 + tempo_index
```

### Tempo Index from BPM
```
tempo_index = bpm - TEMPO_LOW = bpm - 48
```

### Phase to Beat Value
```
beat = sin(phase)  // Range: -1.0 to 1.0
```

### Goertzel Coefficient
```
k = target_freq * buffer_length / sample_rate
w = 2 * PI * k / buffer_length
coeff = 2 * cos(w)
```

---

*Generated from Emotiscope v1.1 HIL instrumented firmware, January 2026*
