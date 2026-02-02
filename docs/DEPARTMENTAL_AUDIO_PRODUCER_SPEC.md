# Departmental Audio Producer Spec

**Version:** 1.1.0  
**Last Updated:** 2026-01-30  
**Status:** Locked (Critical fixes applied)  
**Platform Scope:** Platform-agnostic pipeline contract (platform-specific capture implementations)

---

## Document Purpose

This specification defines the **departmental audio pipeline architecture** as a **platform-agnostic contract**. It establishes a **two-lane producer model** (fast lane for hair-trigger responsiveness, slow lane for stability/frequency resolution) with explicit data contracts, department responsibilities, and failure containment rules.

**Platform-specific implementations** (ESP32-S3, ESP32-P4, etc.) must adhere to this contract but may use platform-optimized capture drivers (e.g., ESP-IDF std mode I2S for ESP32-P4).

**This spec is the single source of truth** for audio pipeline architecture. All implementation plans, code, and tests must reference this document.

---

## 0. Non-Negotiable Architecture Rule

### Two-Lane Pipeline (STFT Tradeoff)

**Core principle:** Short-time Fourier transform (STFT) physics dictate a fundamental tradeoff:

- **Short windows** = excellent time resolution, poor frequency resolution
- **Long windows** = excellent frequency resolution, poor time resolution

**Architecture decision:** Implement **two parallel lanes** that serve different needs:

- **Fast Lane:** Per-hop (8 ms) features for "hair trigger" responsiveness
- **Slow Lane:** Longer-window features for stability, frequency resolution, and musical structure (tempo/key/texture)

**Visual consumers** may subscribe to either lane or both, but the producer **never blends** them—each lane maintains independent update cadences and quality metrics.

**Reference:** [Short-time Fourier transform - Wikipedia](https://en.wikipedia.org/wiki/Short-time_Fourier_transform)

---

## 1. Capture Contract (Single Source of Truth)

### 1.1 Capture Parameters (LOCKED)

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Sample Rate (`Fs`)** | `16000 Hz` | Fixed, no runtime changes |
| **Hop Size (`Hop`)** | `128 samples` | Equivalent to **8 ms** per hop |
| **I2S Driver** | Platform-specific (e.g., ESP-IDF **std mode** for ESP32-P4) | Legacy I2S drivers cannot coexist and may be removed |
| **DMA Buffer Strategy** | Fixed-size chunks, zero-copy where possible | **Protect capture continuity** — never block I2S DMA |

**Locked constants file:** `audio_config.h` (or equivalent) is the **single source of truth** for these values. Planning docs reference it; tests assert it.

**Reference:** [ESP-IDF I2S Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html)

### 1.2 Producer Invariants

**Sequencing:**
- Every **captured hop** receives a **monotonically increasing sequence number** (`seq`)
- Sequence numbers **may skip** (gaps indicate dropped **published frames**, not dropped samples)
- Sequence numbers **never repeat** (duplicates indicate capture corruption)
- **Gap signaling:** If frames are dropped, set `AUDIO_FLAG_OVERFLOW` and publish next available frame with correct `seq` (consumers detect gaps via `seq` jumps)

**Timestamps:**
- Every published `AudioFrame` carries:
  - `t_capture_us`: Timestamp of the **last sample** in the hop (microseconds since boot)
  - `seq`: Sequence number for ordering/validation

**Failure Detection:**
- Capture task sets flags on:
  - **OVERFLOW**: Ring buffer overflow (analysis frames dropped, samples preserved in DMA)
  - **UNDERRUN**: DMA buffer underrun (gaps in I2S stream)
  - **CLIPPED**: Input samples exceeded ±1.0 range

**Drop Policy (Bulletproof):**
- **Never block I2S DMA** — capture task has highest priority
- **Protect samples:** Ring buffer sized to prevent overflow under spec load (≥2× slow lane window)
- **Drop work, not samples:** If ring buffer overflows, drop **published frames** (analysis work), not samples from DMA
- **Gap signaling:** Dropped frames result in `seq` gaps; consumers detect via sequence number jumps

---

## 2. Feature Bus: AudioFrame Schema

### 2.1 Schema Versioning

All `AudioFrame` structures include a `version` field. Consumers must check version before reading fields. Backward-compatible additions increment minor version; breaking changes increment major version.

### 2.2 AudioFrame Structure

```c
typedef struct {
    // ============================================
    // HEADER (always present)
    // ============================================
    uint32_t version;           // Schema version (e.g., 0x00010000 = v1.0)
    uint32_t seq;               // Monotonically increasing sequence number
    uint64_t t_capture_us;      // Timestamp of last sample in hop (us since boot)
    uint32_t flags;             // Bitfield: OVERFLOW, UNDERRUN, CLIPPED, etc.
    float quality;              // Overall health/confidence (0.0 = bad, 1.0 = perfect)
    
    // ============================================
    // FAST LANE (updated every hop, 8 ms)
    // ============================================
    
    // Energy & Dynamics (Dept D)
    float vu_rms;               // RMS amplitude over hop
    float vu_peak;              // Peak amplitude over hop
    float noise_floor;          // Estimated noise floor
    float crest_factor;         // Peak/RMS ratio (optional, may be 0.0 if not computed)
    
    // Rhythm & Time - Fast (Dept A)
    float onset_strength;       // Half-wave rectified spectral flux (FFT-based)
    
    // Timbre & Texture - Fast (Dept C)
    float band_energy[8];       // Coarse musical bands (e.g., sub-bass, bass, low-mid, mid, upper-mid, presence, treble, air)
    
    // ============================================
    // SLOW LANE (updated at 25-50 Hz, may repeat last value on intermediate hops)
    // ============================================
    
    // Pitch & Harmony (Dept B)
    float goertzel_bins[64];    // Musical note bins (A1-C7, semitone-spaced) - computed over longer window, published every hop
    float chroma[12];           // Pitch class energy (C, C#, D, ..., B)
    float chroma_conf;          // Chroma confidence (0.0-1.0)
    
    // Rhythm & Time (Dept A)
    float tempo_bpm;            // Estimated tempo (BPM)
    float tempo_conf;           // Tempo confidence (0.0-1.0)
    float beat_phase_0_1;       // Beat phase (0.0 = downbeat, 1.0 = next downbeat)
    float beat_conf;            // Beat phase confidence (0.0-1.0)
    
    // Timbre & Texture (Dept C)
    float centroid;             // Spectral centroid (brightness, Hz)
    float rolloff;              // Spectral rolloff (energy distribution, Hz)
    float flatness;             // Spectral flatness (tonal vs noisy, 0.0-1.0)
    
    // Optional future additions (set to 0.0 if not computed):
    // float hp_ratio;          // Harmonic/percussive ratio (HPSS-style, 0.0-1.0)
    
} AudioFrame;
```

### 2.3 Quality Flags (flags bitfield)

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | `AUDIO_FLAG_OVERFLOW` | DMA buffer overflow detected |
| 1 | `AUDIO_FLAG_UNDERRUN` | DMA buffer underrun detected |
| 2 | `AUDIO_FLAG_CLIPPED` | Input samples exceeded ±1.0 range |
| 3 | `AUDIO_FLAG_NOISE_FLOOR_HIGH` | Noise floor above threshold (low SNR) |
| 4 | `AUDIO_FLAG_TEMPO_UNCERTAIN` | Tempo estimation confidence below threshold |
| 5 | `AUDIO_FLAG_SLOW_LANE_STALE` | Slow lane data is stale (reused from previous hop) |
| 6-31 | *Reserved* | For future use |

### 2.4 Consumer Contract

**Rule:** Visual consumers read **only** the `AudioFrame` struct (or a versioned superset). No reaching into internal producer buffers, no direct access to intermediate DSP arrays.

**Lane Selection:**
- Consumers may subscribe to **fast lane only** (for maximum responsiveness)
- Consumers may subscribe to **slow lane only** (for stable musical features)
- Consumers may subscribe to **both lanes** (for hybrid behavior)

**Staleness Handling:**
- If `AUDIO_FLAG_SLOW_LANE_STALE` is set, slow lane fields contain **last known good values** (not zeros)
- Consumers should check `flags` before trusting slow lane data for critical decisions

---

## 3. Department Responsibilities

### Department A: Rhythm & Time

**Outputs:**
- **Fast:** `onset_strength` (every hop, 8 ms)
- **Slow:** `tempo_bpm`, `tempo_conf`, `beat_phase_0_1`, `beat_conf` (25-50 Hz)

**Tools:**
- **Fast:** FFT-based spectral flux (half-wave rectified magnitude differences)
- **Slow:** Goertzel-style analysis on novelty curve history (tempo bins + phase tracking)

**Cadence:**
- Fast: Every hop (8 ms)
- Slow: Every 4 hops (32 ms) or configurable 25-50 Hz

**Notes:**
- Tempo estimation benefits from longer history—keep it off the hop critical path
- Onset strength uses FFT magnitudes, not Goertzel bins (broader frequency coverage)

**Reference:** [Spectral flux - Wikipedia](https://en.wikipedia.org/wiki/Spectral_flux)

### Department B: Pitch & Harmony

**Outputs:**
- **Slow:** `goertzel_bins[64]` (computed over longer window, published every hop), `chroma[12]`, `chroma_conf` (25-50 Hz)

**Tools:**
- **Slow:** Goertzel bank (64 bins, A1-C7, semitone-spaced) computed over **512-2048 sample sliding window** (32-128 ms), latest value published every hop
- **Slow:** Chromagram aggregation from Goertzel bins + optional high-res FFT for tonal refinement

**Cadence:**
- Goertzel bins: Computed every 4 hops (32 ms), latest value published every hop (8 ms)
- Chroma: Every 4 hops (32 ms) or configurable 25-50 Hz

**Notes:**
- **8 ms hop insufficient for semitone resolution** — Goertzel bins computed over longer sliding window (512-2048 samples) for frequency resolution, but **latest value published every hop** for fast updates
- Chroma stability improves when not forced into 8 ms-only thinking (STFT tradeoff)
- Goertzel is efficient for a small set of musically chosen bins

**Reference:** [Goertzel algorithm - Wikipedia](https://en.wikipedia.org/wiki/Goertzel_algorithm), [STMicroelectronics Goertzel Design Tip](https://www.st.com/resource/en/design_tip/dt0089-the-goertzel-algorithm-to-compute-individual-terms-of-the-discrete-fourier-transform-dft-stmicroelectronics.pdf)

### Department C: Timbre & Texture

**Outputs:**
- **Fast:** `band_energy[8]` (every hop, 8 ms)
- **Slow:** `centroid`, `rolloff`, `flatness` (25-50 Hz)

**Tools:**
- **Fast:** Coarse band energy aggregation (from FFT or Goertzel)
- **Slow:** FFT-based spectral shape analysis (centroid, rolloff, flatness)

**Cadence:**
- Fast: Every hop (8 ms)
- Slow: Every 4 hops (32 ms) or configurable 25-50 Hz

**Notes:**
- Spectral shape metrics require longer windows for frequency resolution
- Band energies can be computed quickly from existing FFT/Goertzel outputs

### Department D: Energy & Dynamics

**Outputs:**
- **Fast:** `vu_rms`, `vu_peak`, `noise_floor`, `crest_factor` (every hop, 8 ms)

**Tools:**
- Time-domain statistics (RMS, peak) + short FFT for noise floor estimation

**Cadence:**
- Every hop (8 ms) — always-on, highest priority

**Notes:**
- Implement attack/decay smoothing here—**don't** let other departments re-smooth
- Noise floor estimation uses low-frequency FFT bins or time-domain statistics

---

## 4. Scheduling Plan (Bulletproof Producer)

### 4.1 Task Architecture

**Three tasks/threads:**

1. **Capture Task (Highest Priority)**
   - Reads I2S DMA chunk → writes into lock-free ring/history buffer
   - Increments `seq` for each captured hop
   - Sets flags on overflow/underrun
   - **Never blocks** — if ring buffer is full, **drop oldest analysis frames** (not samples), set OVERFLOW flag, continue capturing

2. **Fast Lane Task (Hard Real-Time Budget)**
   - Consumes exactly **one hop** each cycle
   - Produces fast lane features (Dept D, Dept A-fast, Dept B-fast, Dept C-fast)
   - Publishes `AudioFrame` with fast lane populated
   - **Budget:** Must complete within **<4000 µs** (50% of 8 ms hop; tune as needed)
   - **Failure mode:** If overrun, increment `FAST_OVERRUN` counter and **skip nonessential work** (never block capture)

3. **Slow Lane Task (Best-Effort)**
   - Runs every **N hops** (e.g., 4 hops = 32 ms cadence)
   - Reads from history buffer; runs FFT/chroma/tempo updates
   - Publishes updated slow lane fields into the next `AudioFrame`(s)
   - **Failure mode:** If falls behind, **drop work** and reuse last slow outputs (set `AUDIO_FLAG_SLOW_LANE_STALE`)

### 4.2 Critical Containment Rules

**Fast Lane Overrun:**
- If fast lane exceeds budget → **skip nonessential work** (e.g., skip `crest_factor` if needed)
- **Never block capture task** — capture has highest priority
- Increment `FAST_OVERRUN` counter for monitoring
- Treat persistent overruns as **Phase gate failure** (architecture review required)

**Slow Lane Stall:**
- If slow lane falls behind → **drop work**, reuse last slow outputs
- Set `AUDIO_FLAG_SLOW_LANE_STALE` flag
- **Never block fast lane** — slow lane is best-effort

**Capture Failure:**
- If capture task detects overflow/underrun → set flags, continue
- **Never stop producing frames** — consumers need continuous stream (even if degraded)

### 4.3 History Buffer Requirements

**Size:** Must hold at least **2× slow lane window size** (e.g., if slow lane uses 512-sample FFT, buffer must hold ≥1024 samples)

**Overflow Prevention:**
- Ring buffer sized to prevent overflow under spec load (≥2× slow lane window + fast lane consumption rate)
- If overflow occurs despite sizing, **drop published frames** (analysis work), not samples from DMA
- Gap signaling via `seq` jumps + `AUDIO_FLAG_OVERFLOW` flag

**Access Pattern:**
- Capture task: **write-only** (append, never blocks)
- Fast lane task: **read-only** (latest hop)
- Slow lane task: **read-only** (historical window)

**Thread Safety:** Lock-free ring buffer (atomic read/write pointers)

---

## 5. DSP Tool Guidance (ESP32-P4)

### 5.1 FFT Usage

**Tool:** Platform-specific DSP library (e.g., ESP-DSP for ESP32-P4) — optimized implementations with SIMD acceleration where available.

**Slow Lane FFT Parameters (LOCKED):**

| Parameter | Value | Notes |
|-----------|-------|-------|
| **FFT Size** | `512 samples` | Fixed for spectral shape analysis |
| **Window Function** | `Hann window` | Applied before FFT |
| **Overlap** | `75%` (128-sample hop, 512-sample window) | 4× overlap for smooth updates |
| **Update Cadence** | `32 ms` (every 4 hops) | Matches slow lane cadence |

**Use Cases:**
- **Broad FFT (512 point):** Spectral shape (centroid, rolloff, flatness), onset strength (spectral flux)
- **Narrow FFT (optional, band-specific):** Higher resolution per band for timbre analysis

**SIMD Acceleration:**
- Platform-specific (e.g., ESP32-P4 includes **PIE SIMD-style instruction extensions** for DSP/AI workloads)
- FFT-heavy features benefit from compute acceleration where available

**Reference:** 
- [esp-dsp GitHub](https://github.com/espressif/esp-dsp)
- [ESP32-P4 PIE Introduction](https://developer.espressif.com/blog/2024/12/pie-introduction/)

### 5.2 Goertzel Usage

**Tool:** Custom Goertzel implementation (efficient for small bin sets)

**Slow Lane Goertzel Parameters (LOCKED):**

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Window Size** | `1024 samples` (64 ms) | Sliding window for frequency resolution |
| **Bins** | `64 bins` | A1-C7, semitone-spaced |
| **Update Cadence** | `32 ms` (every 4 hops) | Computed every 4 hops, latest value published every hop |
| **Publish Cadence** | `8 ms` (every hop) | Latest computed value published for fast updates |

**Use Cases:**
- **Musical bins (64 bins):** A1-C7, semitone-spaced (Dept B-slow, published fast)
- **Tempo bins (96 bins):** Novelty curve analysis for tempo/beat (Dept A-slow)

**Why Goertzel:**
- **Cost-effective** for a small set of musically chosen frequencies
- **Lower latency** than full FFT when only specific bins are needed
- **Musical alignment** — bins match note frequencies exactly
- **Sliding window** provides frequency resolution while publishing latest value every hop maintains responsiveness

**Reference:** [Goertzel algorithm - Wikipedia](https://en.wikipedia.org/wiki/Goertzel_algorithm), [STMicroelectronics Goertzel Design Tip](https://www.st.com/resource/en/design_tip/dt0089-the-goertzel-algorithm-to-compute-individual-terms-of-the-discrete-fourier-transform-dft-stmicroelectronics.pdf)

### 5.3 Tool Selection Matrix

| Feature | Tool | Department | Lane |
|---------|------|------------|------|
| Onset strength | FFT (spectral flux) | A | Fast |
| Tempo bins | Goertzel (novelty curve) | A | Slow |
| Musical bins | Goertzel (64 bins, 1024-sample window) | B | Slow (published fast) |
| Chromagram | Goertzel aggregation | B | Slow |
| Band energies | FFT or Goertzel | C | Fast |
| Spectral shape | FFT (512-point, Hann window) | C | Slow |
| RMS/Peak | Time-domain stats | D | Fast |
| Noise floor | FFT (low-frequency bins) | D | Fast |

---

## 6. Acceptance Checks (No Excuses Gates)

### 6.1 Latency Budget Checks

**Target:** ≤16 ms end-to-end latency (1-2 hops)  
**Gate:** ≤24 ms end-to-end latency (hard limit)

**Measurement (Complete Chain):**
- **Audio latency:** Capture timestamp (`t_capture_us`) to `AudioFrame` publish timestamp
- **Visual latency:** `AudioFrame` publish timestamp to visual commit timestamp (LED buffer write complete)
- **End-to-end latency:** Capture timestamp (`t_capture_us`) to visual commit timestamp
- Must pass over **72-hour continuous run** with **99.9% of frames** meeting target

**Note:** For "audio → LED" truth, measure complete chain including visual commit. Audio-only latency is insufficient.

### 6.2 Hop Budget Checks

**Fast Lane Budget:**
- `max_fast_lane_us < 4000` (50% of 8 ms hop; tune as needed)
- Must pass over **72-hour continuous run**

**Capture Overruns:**
- `ring_buffer_overflows == 0` over **72-hour run** (zero tolerance for properly sized buffers)
- If overflow occurs, **published frame drops** are acceptable (samples preserved in DMA), but must be logged and analyzed

**Fast Lane Overruns:**
- `fast_lane_overruns < 0.1%` of hops over **72-hour run** (acceptable for nonessential work skipping)

### 6.3 Correctness Checks

**Goertzel Bins:**
- **Golden-tone tests:** Known sine inputs (e.g., 440 Hz) → verify correct bin magnitude
- **Frequency sweep:** 55 Hz to 2093 Hz → verify bin responses match expected musical notes

**FFT Sanity:**
- **Known sine sweeps:** Verify FFT bins match expected frequencies
- **White noise:** Verify flatness metric responds correctly

**Onset Detection:**
- **Percussive clips:** Known percussive audio → verify stable onset peaks
- **Regression tests:** Maintain test suite of known-good audio clips

**Reference:** [A Tutorial on Onset Detection in Music Signals](https://www.iro.umontreal.ca/~pift6080/H09/documents/papers/bello_onset_tutorial.pdf)

### 6.4 Quality Metrics

**Signal Quality:**
- `quality` field must reflect actual SNR (not just flags)
- Low `quality` (<0.5) should correlate with `AUDIO_FLAG_NOISE_FLOOR_HIGH`

**Tempo Confidence:**
- `tempo_conf` must reflect actual tempo estimation stability
- Low `tempo_conf` (<0.3) should set `AUDIO_FLAG_TEMPO_UNCERTAIN`

---

## 7. Spec Drift Guardrails

### 7.1 Single Source of Truth

**Locked Constants File:** `audio_config.h` (or equivalent)

**Contains:**
- `FS = 16000`
- `HOP_SIZE = 128`
- `HOP_MS = 8`
- All other capture parameters

**Rule:** Planning docs reference this file; tests assert it; code includes it.

### 7.2 Version Control

**Schema Versioning:**
- `AudioFrame.version` field tracks schema changes
- Backward-compatible additions → increment minor version (e.g., 1.0 → 1.1)
- Breaking changes → increment major version (e.g., 1.0 → 2.0)

**Spec Versioning:**
- This document uses semantic versioning (MAJOR.MINOR.PATCH)
- Changes to locked constants → increment major version
- Clarifications/additions → increment minor version

### 7.3 Validation Checklist

Before marking implementation as "complete," verify:

- [ ] All locked constants match `audio_config.h`
- [ ] `AudioFrame` schema matches this spec exactly
- [ ] Fast lane completes within budget (<4000 µs)
- [ ] Slow lane updates at correct cadence (25-50 Hz)
- [ ] Capture task never blocks
- [ ] Fast lane never blocks capture
- [ ] Slow lane never blocks fast lane
- [ ] Quality flags are set correctly
- [ ] Golden-tone tests pass
- [ ] FFT sanity tests pass
- [ ] Onset detection regression tests pass
- [ ] End-to-end latency meets target (≤16 ms, gate ≤24 ms) including visual commit timestamp
- [ ] Ring buffer sized to prevent overflow under spec load
- [ ] Goertzel bins computed over 1024-sample sliding window
- [ ] FFT uses 512-point Hann window with 75% overlap

---

## 8. Outputs Summary (Beyond v1.1)

### 8.1 Core Outputs (Already in v1.1)

| Output | Size | Department | Lane |
|--------|------|------------|------|
| `goertzel_bins[64]` | 64 × float | B | Slow (published fast) |
| `chromagram[12]` | 12 × float | B | Slow |
| `vu_level`, `vu_max`, `vu_floor` | 3 × float | D | Fast |
| `novelty_curve[1024]` | 1024 × float | A | Slow (internal) |
| `tempi[96]` (magnitudes + phase) | 96 × struct | A | Slow |

### 8.2 FFT-Derived Additions (Recommended)

| Output | Size | Department | Lane | Notes |
|--------|------|------------|------|-------|
| `spectral_centroid` | 1 × float | C | Slow | Brightness (Hz) |
| `spectral_rolloff` | 1 × float | C | Slow | Energy distribution (Hz) |
| `spectral_flatness` | 1 × float | C | Slow | Tonal vs noisy (0.0-1.0) |
| `band_energy[8]` | 8 × float | C | Fast | Coarse musical bands |
| `onset_strength` | 1 × float | A | Fast | FFT flux (separate from Goertzel novelty) |
| `vu_rms`, `vu_peak` | 2 × float | D | Fast | Fast amplitude stats |
| `crest_factor` | 1 × float | D | Fast | Dynamic range (peak/RMS) |
| `noise_floor` | 1 × float | D | Fast | For gating low-energy conditions |

**Cost:** These additions are **cheap on SIMD** and give visual systems a much richer feature space.

---

## 9. References

### External References

- [Short-time Fourier transform - Wikipedia](https://en.wikipedia.org/wiki/Short-time_Fourier_transform)
- [ESP-IDF I2S Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html)
- [Spectral flux - Wikipedia](https://en.wikipedia.org/wiki/Spectral_flux)
- [Goertzel algorithm - Wikipedia](https://en.wikipedia.org/wiki/Goertzel_algorithm)
- [esp-dsp GitHub Repository](https://github.com/espressif/esp-dsp)
- [ESP32-P4 PIE Introduction](https://developer.espressif.com/blog/2024/12/pie-introduction/)
- [A Tutorial on Onset Detection in Music Signals](https://www.iro.umontreal.ca/~pift6080/H09/documents/papers/bello_onset_tutorial.pdf)

### Internal References

- `CLAUDE.md` - Emotiscope.HIL project guidance
- `v1.1_build/planning/01-comprehensive-hil-monitoring/prd.md` - HIL monitoring PRD
- `docs/EMOTISCOPE_AUDIO_PIPELINE_COMPLETE_REFERENCE.md` - Audio pipeline reference

---

## 10. Change Log

| Version | Date | Changes |
|---------|------|---------|
| 1.1.0 | 2026-01-30 | Fixed drop policy contradiction, moved Goertzel bins to slow lane with sliding window, made platform-agnostic, locked slow-lane window sizes, completed latency measurement definition |
| 1.0.0 | 2026-01-30 | Initial locked spec |

---

**End of Specification**
