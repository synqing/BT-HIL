# Product Requirements Document: Comprehensive HIL Monitoring for Emotiscope v1.1

> **Traceability Note:** This PRD uses section numbers (§1-§10) that are referenced by the Technical Blueprint and Task List. All user answers have been mapped to specific sections.

## §1. Overview & Vision

Implement a comprehensive Hardware-in-the-Loop (HIL) monitoring system for Emotiscope v1.1 to enable complete algorithmic reverse-engineering and validation of the audio and visual processing pipelines. This instrumentation will provide granular visibility into every DSP stage, intermediate signal output, and rendering algorithm to inform the development of the next-generation K1-Lightwave system through empirical comparison against Lightwave-Ledstrip and Tab5.DSP reference implementations.

**Key Outcomes:**
- **Algorithm Reverse-Engineering**: Capture complete intermediate signal data from all 64 Goertzel frequency bins, spectral flux novelty curve, 96 tempo/beat detection bins, and chromagram outputs for deep analysis
- **Timing Characterization**: Microsecond-precision timing breakdowns at every audio DSP stage (I2S capture, Goertzel analysis, VU meter, spectral flux, tempo detection) and visual rendering stage (12+ light mode algorithms, post-processing pipeline)
- **Multi-Format Data Export**: Simultaneous data capture in CSV (spreadsheet analysis), JSON (structured metadata), binary dumps (signal processing tools), WebSocket streaming (real-time monitoring), and serial logs (long-duration testing via external capture)
- **Benchmarking Infrastructure**: Enable empirical algorithm validation through timestamp-aligned comparison of v1.1 outputs against Lightwave-Ledstrip and Tab5.DSP implementations processing identical audio inputs
- **Zero Performance Constraints**: Maximum data extraction without regard for CPU overhead, memory usage, or frame rate degradation - prioritize complete instrumentation over real-time performance

## §2. Problem Statement

The Emotiscope v1.1 audio processing pipeline contains sophisticated DSP algorithms (Goertzel-based frequency analysis, spectral flux novelty detection, novelty-domain tempo tracking) whose implementation details and parameter choices are not fully documented. To develop the K1-Lightwave next-generation system, we need:

**Current Pain Points:**
- **Black Box Algorithms**: Cannot observe intermediate DSP outputs between processing stages, making it impossible to understand how frequency magnitudes transform into beat detection
- **No Empirical Validation**: Unable to compare v1.1's algorithm behavior against reference implementations (Lightwave-Ledstrip, Tab5.DSP) using identical test signals
- **Limited Timing Visibility**: Existing profiler.h and hil_monitor.h capture aggregate timing but lack per-frame granularity and don't log intermediate signal buffers
- **Inefficient Debugging**: When algorithm behavior is unexpected, no mechanism exists to capture the full signal processing chain for offline analysis
- **Visual Pipeline Opacity**: 12+ light mode algorithms (Spectrum, Octave, Bloom, Hype, etc.) and multi-stage post-processing pipeline (LPF, tonemapping, gamma, warmth) lack per-algorithm timing and intermediate buffer capture

## §3. Target Users

| User Type | Needs | Key Actions |
|-----------|-------|-------------|
| **Embedded Systems Engineer (Primary)** | Reverse-engineer v1.1 DSP algorithms through complete intermediate signal capture and timing analysis | Collect multi-hour test recordings with full signal dumps, analyze timing breakdowns per DSP stage, compare algorithm outputs against reference implementations |
| **K1-Lightwave System Architect** | Empirical data to inform design decisions: which algorithms to port, which to replace, where performance bottlenecks exist | Run benchmark tests with controlled audio inputs, compare v1.1 vs Lightwave-Ledstrip vs Tab5.DSP outputs, identify algorithmic differences through signal-level comparison |
| **Algorithm Developer** | Understand how Goertzel frequency analysis, spectral flux, and tempo detection parameters were tuned in v1.1 | Export intermediate signal arrays (spectrogram[64], novelty_curve[1024], tempi[96]) for offline analysis in Python/MATLAB, visualize signal transformations through processing stages |

## §4. Core Requirements

### §4.1 Functional Requirements

| ID | Requirement | Priority | Source |
|----|-------------|----------|--------|
| FR-1 | Capture complete intermediate signal arrays from audio pipeline: sample_history[4096], spectrogram[64], spectrogram_smooth[64], chromagram[12], novelty_curve[1024], tempi[96].magnitude, tempi[96].phase on every frame | Critical | Q1: Audio pipeline deep-dive is primary objective |
| FR-2 | Log microsecond-precision timing for each audio DSP stage: I2S capture, Goertzel magnitude calculation (per bin and aggregate), chromagram computation, VU meter processing, spectral flux calculation, tempo magnitude calculation, tempo phase synchronization | Critical | Q4: Timing breakdowns absolutely critical |
| FR-3 | Implement per-algorithm timing instrumentation for all 12+ light modes: wrap each light_modes[].draw() function with profile_function() to capture execution time per frame | High | Q1: Visual pipeline algorithmic rendering logic follows audio |
| FR-4 | Capture intermediate visual rendering buffers: leds[] CRGBF array after each post-processing stage (apply_image_lpf, apply_tonemapping, apply_warmth, apply_gamma_correction, apply_master_brightness) | High | Q1: Multiple layers of custom LED manipulation |
| FR-5 | Export captured data in CSV format: timestamp-aligned rows with columns for each metric/signal, compatible with Excel/Python pandas for offline analysis | Critical | Q3: Complete data capture in all formats |
| FR-6 | Export captured data in JSON format: structured snapshots with metadata (sample_rate, block_sizes, algorithm parameters, hardware version) for configuration comparison | Critical | Q3: Complete data capture in all formats |
| FR-7 | Export captured data as binary signal dumps: raw intermediate arrays (float32 format) for signal processing tools (MATLAB, NumPy, SciPy) | Critical | Q3: Complete data capture in all formats |
| FR-8 | Stream metrics via WebSocket: extend existing broadcast() function to support binary frames for efficient real-time array transmission to connected clients | High | Q3: Real-time streaming, Q4: Real-time signal visualization valuable |
| FR-9 | Implement serial logging: continuous data streaming for multi-hour test recordings via Serial at 2Mbaud (captured by external tool) and LittleFS snapshots | High | Q3: Log files for long-duration testing |
| FR-10 | Add trigger-based data capture: API to start/stop detailed logging on demand, selective metric capture modes (timing-only vs full signal dumps) | Medium | Q4: Trigger data captures nice to have |
| FR-11 | Implement metric selection controls: configure which arrays to log (e.g., spectrogram only, tempo only, all) to manage data volume | Medium | Q4: Select which metrics to display nice to have |
| FR-12 | Create real-time signal visualization dashboard: Chart.js-based waveform/spectrogram displays for spectrogram[64], novelty_curve[1024], tempi[96] arrays | Low | Q4: Real-time visualization valuable but secondary to data logging |

### §4.2 Non-Functional Requirements

| ID | Category | Requirement | Source |
|----|----------|-------------|--------|
| NFR-1 | Performance | NO performance constraints - instrument comprehensively regardless of CPU overhead, memory usage, or frame rate impact. Maximum data extraction is the sole priority. | Q2: Don't care about device suffering from performance degradation |
| NFR-2 | Instrumentation Overhead | Acceptable to reduce audio pipeline from ~200 FPS to <50 FPS if necessary to capture all intermediate signals every frame. Real-time responsiveness is NOT a requirement for monitoring mode. | Q2: Only care about extracting metrics |
| NFR-3 | Data Completeness | Every DSP frame must log complete timing breakdowns and optionally full intermediate signal arrays. Zero tolerance for dropped frames or missing data during capture sessions. | Q3: ALL OF IT - complete data capture |
| NFR-4 | Format Flexibility | Must support simultaneous export in ALL formats (CSV + JSON + binary + WebSocket + serial) to enable diverse analysis workflows without reconfiguration. | Q3: All of the above formats |
| NFR-5 | Timing Precision | All timing measurements must use ESP.getCycleCount() at 240MHz for sub-microsecond resolution. Aggregate timing into microsecond values for logging. | Q4: Timing breakdowns absolutely critical |
| NFR-6 | Signal Integrity | Captured intermediate arrays must be bit-exact copies of the live signal processing buffers. No filtering, downsampling, or modification during capture. | Derived from algorithm reverse-engineering goal |
| NFR-7 | Metadata Completeness | Every data export must include full context: timestamp, hardware version, firmware version, algorithm parameters (Goertzel block sizes, window functions, auto-ranging scales, tempo frequency ranges). | Derived from benchmarking requirement |
| NFR-8 | Non-Functional Metrics Excluded | Do NOT log or display CPU usage percentage, heap allocation, RAM usage, or FPS metrics - these are explicitly not relevant to algorithm analysis. | Q4: Don't care about CPU usage, heap, RAM, or FPS |

## §5. User Stories & Acceptance Criteria

### Epic 1: Audio Pipeline Deep-Dive Instrumentation

#### Story §5.1: Goertzel Frequency Analysis Signal Capture
**As an** algorithm developer
**I want** to capture spectrogram[64], spectrogram_smooth[64], and per-bin noise floor values on every audio frame
**So that** I can analyze how the Goertzel algorithm transforms raw audio into frequency magnitudes and understand the noise subtraction and auto-ranging behavior

**Acceptance Criteria:**
- [ ] §5.1.1: spectrogram[64] array logged every frame with full float32 precision (no quantization)
- [ ] §5.1.2: spectrogram_smooth[64] array logged showing the result of moving average filtering
- [ ] §5.1.3: Per-bin noise floor values logged (from noise_history[10][64]) to show adaptive noise subtraction
- [ ] §5.1.4: Auto-ranging scale factor logged to understand magnitude normalization
- [ ] §5.1.5: Goertzel algorithm parameters logged: block_size per bin, window_step, coeff values for all 64 bins
- [ ] §5.1.6: Timing logged: DSP_MAGNITUDES_US broken down into per-bin calculation time if possible

#### Story §5.2: Spectral Flux and Novelty Curve Capture
**As an** algorithm developer
**I want** to capture the complete novelty_curve[1024] buffer at 50 Hz update rate with timestamp alignment
**So that** I can understand how spectral flux (onset detection) is calculated from frame-to-frame magnitude changes

**Acceptance Criteria:**
- [ ] §5.2.1: novelty_curve[1024] array logged at exactly 50 Hz (every 20ms) with precise timestamps
- [ ] §5.2.2: Per-bin spectral flux values logged (magnitude_current - magnitude_previous for all 64 bins) before summing
- [ ] §5.2.3: log1p transformation parameter logged showing dynamic range compression approach
- [ ] §5.2.4: novelty_curve_normalized[1024] logged separately showing auto-scaled version used for tempo detection
- [ ] §5.2.5: Silence detection threshold and contrast values logged when check_silence() triggers

#### Story §5.3: Tempo/Beat Detection Signal Capture
**As an** algorithm developer
**I want** to capture tempi[96].magnitude, tempi[96].phase, tempi[96].beat for all 96 BPM bins on every frame
**So that** I can reverse-engineer how the novelty-domain Goertzel algorithm estimates tempo and phase-locks to beats

**Acceptance Criteria:**
- [ ] §5.3.1: tempi[96].magnitude array logged every frame showing beat strength across 48-144 BPM range
- [ ] §5.3.2: tempi[96].phase array logged showing phase angles in radians for each tempo bin
- [ ] §5.3.3: tempi[96].beat array logged showing sin(phase) output for beat synchronization
- [ ] §5.3.4: tempi_smooth[96] array logged showing exponentially smoothed magnitude (0.975 decay factor)
- [ ] §5.3.5: tempo_confidence value logged showing the dominant tempo's relative strength
- [ ] §5.3.6: Tempo algorithm parameters logged: BEAT_SHIFT_PERCENT (phase offset), block_size per tempo, coeff values
- [ ] §5.3.7: Timing logged: DSP_TEMPO_US with breakdown into magnitude calculation vs phase sync if possible

#### Story §5.4: VU Meter and Chromagram Capture
**As an** algorithm developer
**I want** to capture vu_level, vu_max, vu_floor, and chromagram[12] on every frame
**So that** I can understand loudness detection and pitch class analysis behavior

**Acceptance Criteria:**
- [ ] §5.4.1: vu_level (smoothed loudness) logged every frame with float32 precision
- [ ] §5.4.2: vu_max (peak detector) and vu_floor (noise floor) logged to show auto-ranging
- [ ] §5.4.3: chromagram[12] array logged showing pitch class energy distribution
- [ ] §5.4.4: VU smoothing buffer vu_smooth[12] logged to show temporal averaging
- [ ] §5.4.5: Timing logged: DSP_VU_US

#### Story §5.5: I2S Audio Capture Instrumentation
**As an** algorithm developer
**I want** to capture raw sample_history[4096] buffer and I2S health metrics
**So that** I can verify audio input integrity and understand DC offset/clipping behavior

**Acceptance Criteria:**
- [ ] §5.5.1: sample_history[4096] buffer logged periodically (e.g., every 10th frame to manage data volume) as float32 array
- [ ] §5.5.2: I2S health metrics logged: sample rate accuracy (actual vs 12.8kHz target), DMA errors, buffer overruns
- [ ] §5.5.3: DC offset value logged (currently hardcoded as 360) to show preprocessing
- [ ] §5.5.4: Clipping event counter logged showing how often samples hit [-131072, 131072] limits
- [ ] §5.5.5: Timing logged: DSP_ACQUIRE_US for I2S read operation

### Epic 2: Visual Pipeline Instrumentation

#### Story §5.6: Per-Algorithm Light Mode Timing
**As an** algorithm developer
**I want** microsecond timing for each light mode's draw() function execution
**So that** I can compare algorithmic complexity across the 12+ visualization modes

**Acceptance Criteria:**
- [ ] §5.6.1: All light mode draw() functions wrapped with profile_function() for automatic timing
- [ ] §5.6.2: Per-mode timing logged with mode name and execution time in microseconds
- [ ] §5.6.3: Light modes instrumented: FFT, Spectrum, Octave, Beat Tunnel, Bloom, Hype, Spectronome, Tempiscope, Metronome, Analog, Perlin, and any beta modes
- [ ] §5.6.4: Timing aggregated over 250ms intervals showing min/max/avg/stddev per mode
- [ ] §5.6.5: Mode transition events logged with timestamps to correlate timing changes

#### Story §5.7: Post-Processing Pipeline Buffer Capture
**As an** algorithm developer
**I want** to capture leds[] CRGBF array after each post-processing stage
**So that** I can understand how LPF, tonemapping, warmth, and gamma affect the final LED output

**Acceptance Criteria:**
- [ ] §5.7.1: leds[] array captured after apply_image_lpf() showing low-pass filtered output
- [ ] §5.7.2: leds[] array captured after apply_tonemapping() showing dynamic range compression
- [ ] §5.7.3: leds[] array captured after apply_warmth() showing incandescent LUT application
- [ ] §5.7.4: leds[] array captured after apply_gamma_correction() showing final pre-quantization output
- [ ] §5.7.5: Timing logged for each post-processing stage: GPU_LPF_US, TONEMAPPING_US, WARMTH_US, GAMMA_US
- [ ] §5.7.6: Post-processing parameters logged: lpf_cutoff_frequency, warmth value, gamma value

#### Story §5.8: Custom LED Driver Instrumentation
**As an** algorithm developer
**I want** timing for LED transmission (transmit_leds) and quantization/dithering operations
**So that** I can understand the visual output pipeline bottlenecks

**Acceptance Criteria:**
- [ ] §5.8.1: Timing logged: LED_TRANSMIT_US for RMT hardware transmission
- [ ] §5.8.2: Timing logged: LED_QUANTIZE_US for float-to-8bit conversion with dithering
- [ ] §5.8.3: Dithering parameters logged: temporal_dithering enabled/disabled, dither pattern
- [ ] §5.8.4: NUM_LEDS and LED data length logged for correlation with transmission time

### Epic 3: Multi-Format Data Export Infrastructure

#### Story §5.9: CSV Export for Spreadsheet Analysis
**As an** algorithm developer
**I want** all captured metrics exported to CSV files with timestamp-aligned rows
**So that** I can analyze data in Excel/Python pandas and create correlation plots

**Acceptance Criteria:**
- [ ] §5.9.1: CSV header row contains column names for timestamp, all timing metrics, and array indices (e.g., spectrogram_0 through spectrogram_63)
- [ ] §5.9.2: Each frame logged as a single CSV row with all metrics at that timestamp
- [ ] §5.9.3: Array data flattened into separate columns (spectrogram[64] becomes 64 columns)
- [ ] §5.9.4: CSV file creation triggered by user command or auto-started on boot with filename containing timestamp
- [ ] §5.9.5: CSV written to LittleFS (limited frames) or streamed via serial for continuous capture
- [ ] §5.9.6: Frame limit management: auto-stop at max_frames for LittleFS, unlimited for serial streaming

#### Story §5.10: JSON Export for Structured Metadata
**As an** system architect
**I want** JSON-formatted snapshots with all algorithm parameters and hardware configuration
**So that** I can compare v1.1 vs Lightwave-Ledstrip configurations and ensure reproducible benchmarks

**Acceptance Criteria:**
- [ ] §5.10.1: JSON export includes complete metadata: timestamp, hardware_version, firmware_version, sample_rate, NUM_FREQS, NUM_TEMPI, NOVELTY_LOG_HZ
- [ ] §5.10.2: JSON export includes algorithm parameters: Goertzel block_sizes[64], coeff[64], window_step[64], tempo frequencies[96], BEAT_SHIFT_PERCENT
- [ ] §5.10.3: JSON export includes configuration: current_mode, brightness, softness, color, warmth settings
- [ ] §5.10.4: JSON export optionally includes signal data arrays as nested objects (for smaller snapshots)
- [ ] §5.10.5: JSON file created on demand via command or periodically (e.g., every 1000 frames)
- [ ] §5.10.6: JSON validation: parseable by standard JSON libraries without errors

#### Story §5.11: Binary Signal Dumps for Offline Processing
**As an** algorithm developer
**I want** raw intermediate arrays exported as binary files (float32 format)
**So that** I can load signals into MATLAB/NumPy for advanced analysis without CSV parsing overhead

**Acceptance Criteria:**
- [ ] §5.11.1: Binary export format: header with array dimensions, followed by raw float32 data (little-endian)
- [ ] §5.11.2: Separate binary files per array type: spectrogram.bin, novelty_curve.bin, tempi_magnitude.bin, tempi_phase.bin, sample_history.bin
- [ ] §5.11.3: Header includes: magic bytes (0x48494C01 = "HIL" version 1), array length, element size, element type (float32=1), timestamp
- [ ] §5.11.4: Binary data streamed via WebSocket or serial for external capture (no local storage of large binary dumps)
- [ ] §5.11.5: Small binary snapshots can be written to LittleFS for diagnostics
- [ ] §5.11.6: Python/MATLAB read example provided in documentation

#### Story §5.12: WebSocket Real-Time Streaming
**As an** algorithm developer
**I want** intermediate signal arrays streamed via WebSocket in binary frames
**So that** I can visualize signals in real-time on a connected client without waiting for file export

**Acceptance Criteria:**
- [ ] §5.12.1: Extend broadcast() function to support binary WebSocket frames (not just text)
- [ ] §5.12.2: Binary frame format: message type byte (0x01=spectrogram, 0x02=novelty, 0x03=tempi), array length (uint16), float32 data
- [ ] §5.12.3: Streaming modes: full-rate (every frame), decimated (every Nth frame), or on-demand (triggered by client request)
- [ ] §5.12.4: Client can subscribe to specific arrays: send subscription message "SUBSCRIBE:spectrogram" to receive only that array
- [ ] §5.12.5: Timing metrics streamed as text frames: "HIL|DSP_MAGNITUDES_US|1234" (existing format)
- [ ] §5.12.6: Backpressure handling: if WebSocket send buffer full, drop frames rather than blocking audio pipeline

#### Story §5.13: Serial Continuous Logging
**As an** system architect
**I want** continuous logging via serial output for multi-hour test recordings
**So that** I can run overnight benchmarks with external capture tools (e.g., picocom, screen, Python script)

**Acceptance Criteria:**
- [ ] §5.13.1: Logging enabled via command: "log|start|serial" or "log|start|file|<max_frames>"
- [ ] §5.13.2: Serial logging streams CSV data via Serial at 2Mbaud for external capture
- [ ] §5.13.3: LittleFS logging writes small CSV snapshots (limited by flash size ~1.5MB)
- [ ] §5.13.4: Frame limit: LittleFS auto-stops at max_frames, serial continues indefinitely
- [ ] §5.13.5: Log stop via command: "log|stop" flushes buffers and closes files gracefully
- [ ] §5.13.6: Status feedback: "log|status" command returns current mode, frames logged, errors

### Epic 4: Benchmarking and Comparison Infrastructure

#### Story §5.14: Synchronized Test Signal Playback
**As a** system architect
**I want** to play identical audio test signals to v1.1, Lightwave-Ledstrip, and Tab5.DSP simultaneously
**So that** I can compare algorithm outputs with controlled inputs for true apples-to-apples validation

**Acceptance Criteria:**
- [ ] §5.14.1: Document procedure for synchronized playback: use audio splitter to feed identical signal to all three systems
- [ ] §5.14.2: Test signal library created: sine sweeps, white noise, percussive transients, music samples (various genres and BPM)
- [ ] §5.14.3: Test signals recorded with timestamps for alignment during offline analysis
- [ ] §5.14.4: Logging started on all systems simultaneously (manual trigger acceptable, automatic sync nice-to-have)
- [ ] §5.14.5: Timestamp synchronization strategy documented: use common NTP time source or relative timestamps from test signal start

#### Story §5.15: Comparative Analysis Tooling
**As a** system architect
**I want** Python scripts to load v1.1, Lightwave-Ledstrip, and Tab5.DSP logs and generate comparison plots
**So that** I can quickly identify algorithmic differences and performance deltas

**Acceptance Criteria:**
- [ ] §5.15.1: Python script loads CSV exports from all three systems and aligns timestamps
- [ ] §5.15.2: Script generates spectrogram comparison plots: v1.1 vs Lightwave-Ledstrip frequency bins over time
- [ ] §5.15.3: Script generates tempo detection comparison: beat phase alignment, tempo estimate accuracy
- [ ] §5.15.4: Script generates timing comparison: DSP stage latencies across systems
- [ ] §5.15.5: Script outputs metrics: RMS difference between spectrograms, beat detection agreement percentage, latency deltas
- [ ] §5.15.6: Script configurable for different test signals and time ranges

### Epic 5: Real-Time Dashboard (Optional/Secondary)

#### Story §5.16: Timing Breakdown Display
**As an** algorithm developer
**I want** a web page displaying current microsecond timing for each DSP and rendering stage
**So that** I can monitor timing breakdowns in real-time during development

**Acceptance Criteria:**
- [ ] §5.16.1: Dashboard served via existing PsychicHttp server at /dashboard endpoint
- [ ] §5.16.2: Timing table displays: DSP_ACQUIRE_US, DSP_MAGNITUDES_US, DSP_VU_US, DSP_TEMPO_US, GPU_LPF_US with current values
- [ ] §5.16.3: Timing values update every 250ms via WebSocket (existing broadcast interval)
- [ ] §5.16.4: Historical min/max/avg displayed per metric (computed client-side over last 100 samples)
- [ ] §5.16.5: No CPU/RAM/FPS metrics displayed (explicitly excluded per requirements)

#### Story §5.17: Real-Time Signal Visualization
**As an** algorithm developer
**I want** live waveform plots of spectrogram, novelty_curve, and tempi magnitude arrays
**So that** I can visualize DSP outputs while testing and debugging

**Acceptance Criteria:**
- [ ] §5.17.1: Chart.js library used for lightweight real-time plotting (no heavy dependencies)
- [ ] §5.17.2: Spectrogram plot: 64-bin bar chart updating every frame, Y-axis = magnitude [0, 1], X-axis = frequency bins
- [ ] §5.17.3: Novelty curve plot: scrolling line chart showing last 1024 samples (20.48 seconds), auto-scaled Y-axis
- [ ] §5.17.4: Tempi magnitude plot: 96-bin bar chart showing BPM strength distribution, X-axis = 48-144 BPM
- [ ] §5.17.5: Plots subscribe to WebSocket binary frames for signal data (per Story §5.12)
- [ ] §5.17.6: Frame rate throttling: client-side decimation to max 30 FPS plot updates (no need for faster)

#### Story §5.18: Capture Controls
**As an** algorithm developer
**I want** buttons to start/stop logging and select which metrics to capture
**So that** I can control data collection without modifying code

**Acceptance Criteria:**
- [ ] §5.18.1: Dashboard includes "Start CSV Log" / "Stop CSV Log" buttons that send LOG_START/LOG_STOP commands
- [ ] §5.18.2: Checkboxes to enable/disable specific arrays: "Log Spectrogram", "Log Novelty", "Log Tempi", "Log Sample History"
- [ ] §5.18.3: Selected arrays sent in LOG_START command: "LOG_START:CSV:spectrogram,novelty,tempi"
- [ ] §5.18.4: Status indicator shows current log state: "Logging to SD card: 12,345 frames, 3.2 MB" or "Not logging"
- [ ] §5.18.5: Export button triggers JSON snapshot download with current configuration and recent signal data

## §6. User Experience Contract
*→ Extracted into Blueprint §1.5/1.6 User Journey diagrams*

### §6.1 User Visibility Specification

| ID | User Action | User Sees (Exact) | User Does NOT See | Timing |
|----|-------------|-------------------|-------------------|--------|
| V-1 | Enable extended HIL monitoring | Dashboard shows "Extended Monitoring: ACTIVE" badge, timing breakdowns populate with current values | Internal buffer allocation, profiler_functions[] array expansion, conditional compilation flags | Immediate (<100ms) |
| V-2 | Click "Start CSV Log" | Button changes to "Stop CSV Log", status shows "Logging: 0 frames", incrementing frame counter | File descriptor operations, CSV row buffering, filesystem writes | Immediate button change, frame counter updates every 250ms |
| V-3 | Audio processing completes with monitoring active | Dashboard timing table shows microsecond values for each DSP stage, signal plots update with new data | Intermediate array memcpy operations, HILMonitor::log() function calls, WebSocket frame serialization | Timing updates every 250ms, plots update every 33ms (30 FPS) |
| V-4 | Select "Log Spectrogram" checkbox only | LOG_START command sent with "spectrogram" parameter, status shows "Logging 1 array type" | Conditional logic skipping other array captures, reduced CSV column count | Immediate checkbox state change |
| V-5 | Request JSON export | Browser downloads "emotiscope_snapshot_YYYYMMDD_HHMMSS.json" file | JSON serialization of algorithm parameters and metadata, buffer copies to construct snapshot | 1-2 second delay before download dialog (JSON generation time) |
| V-6 | WebSocket disconnects during logging | LittleFS/Serial logging continues uninterrupted, dashboard unavailable but data collection proceeds | WebSocket error handling, broadcast() failures ignored, buffer drops for disconnected clients | Reconnect shows accumulated data when dashboard reloads |
| V-7 | LittleFS frame limit reached | Serial output shows "LittleFS frame limit reached - logging stopped", file finalized | Filesystem close operations, automatic stop trigger | Immediate status update |

### §6.2 Timing & Feedback Expectations

| ID | Event | Expected Timing | User Feedback | Failure Feedback |
|----|-------|-----------------|---------------|------------------|
| T-1 | Enable extended monitoring on boot | <500ms initialization | "HIL Extended Monitoring: ACTIVE" appears in serial log and dashboard | "HIL Extended Monitoring: FAILED - insufficient memory" if buffer allocation fails |
| T-2 | First CSV log row written | <100ms after LOG_START command | Frame counter increments from 0 to 1 within 250ms | "CSV Write Error: filesystem error" if LittleFS unavailable |
| T-3 | WebSocket binary frame sent | <16ms (one audio frame at 200 FPS) | Dashboard plots update smoothly at 30 FPS | Plot freezes, "WebSocket Disconnected" badge appears if connection lost |
| T-4 | JSON snapshot generation | 1-3 seconds for full snapshot with 10 seconds of signal history | Download dialog appears with timestamped filename | Timeout after 5 seconds with "Snapshot generation timeout - try shorter history" error |
| T-5 | LittleFS frame limit reached | Immediate stop | Status shows "Logging stopped - frame limit reached" | N/A - graceful stop, not failure |
| T-6 | Dashboard load on first connection | 2-5 seconds to load Chart.js and render initial plots | Loading spinner, then plots populate with live data | "Failed to connect to /dashboard - check firmware" if endpoint unreachable |
| T-7 | Capture control button click | <50ms button state change | Button text toggles, status indicator updates within 250ms | Button grayed out with "Command failed - see serial log" if device rejects command |

## §7. Artifact Ownership
*→ Extracted into Blueprint §2 System Boundaries*

### §7.1 Creation Responsibility

| ID | Artifact | Created By | When | App's Role |
|----|----------|------------|------|------------|
| O-1 | CSV log files (e.g., /hil_1234567890.csv on LittleFS) | Emotiscope firmware | On log\|start\|file command | Create: Open file, write header row, append data rows every frame until limit |
| O-2 | Serial CSV stream | Emotiscope firmware | On log\|start\|serial command | Create: Stream CSV rows to Serial at 2Mbaud for external capture |
| O-3 | JSON snapshot files (e.g., emotiscope_snapshot_20260114_143500.json) | Emotiscope firmware | On user request from dashboard or command | Create: Serialize current state + metadata + recent signal history |
| O-4 | Dashboard HTML/JS files | Emotiscope firmware (embedded in flash) | On build/flash, served via PsychicHttp | Create: Compile-time embed, runtime serve at /dashboard endpoint |
| O-5 | LittleFS log files | Emotiscope firmware | On log\|start\|file command | Create: File in root directory, limited by flash size (~1.5MB partition) |
| O-6 | WebSocket binary frames (signal data) | Emotiscope firmware | Every frame (or decimated) when clients subscribed | Create: Serialize array to binary frame, broadcast via existing WebSocket infrastructure |
| O-7 | Serial log output stream | Emotiscope firmware | Continuous when serial logging enabled | Create: Format CSV/binary to serial at 2000000 baud, handle buffer overruns |
| O-8 | Python analysis scripts (comparison tools) | Developer (external to firmware) | Created manually outside device | Observe: Scripts load CSV/JSON/binary files created by firmware for offline analysis |
| O-9 | Test signal audio files (.wav) | Developer (external to firmware) | Created manually using DAW or Python | Observe: Played to device via audio splitter, not created by firmware |
| O-10 | Intermediate signal arrays (spectrogram[64], novelty_curve[1024], etc.) | Emotiscope firmware (DSP pipeline) | Every audio frame during normal operation | Observe: HIL monitoring reads and logs these arrays but does NOT modify DSP algorithm operation |

### §7.2 External System Dependencies

| ID | External System | What It Creates | How App Knows | Failure Handling |
|----|-----------------|-----------------|---------------|------------------|
| E-1 | LittleFS Filesystem | File allocation on internal flash | LittleFS.begin() returns true, file.write() succeeds | Fall back to serial logging, show "filesystem error" |
| E-2 | WebSocket Client (web browser) | TCP connection, HTTP upgrade request, frame acknowledgments | PsychicHttp onOpen() callback fires, client added to list | Gracefully degrade: logging continues, binary frames dropped for disconnected clients |
| E-3 | External Audio Source | Test signal audio (sine waves, music, etc.) | I2S microphone receives samples (can't distinguish test vs live audio) | Not detectable - app processes whatever audio I2S provides |
| E-4 | NTP Time Server (optional) | Synchronized timestamps across multiple devices for benchmarking | ESP32 SNTP library sets system time, millis() reflects UTC | Use relative timestamps (millis() since boot) if NTP unavailable |
| E-5 | Python Analysis Environment | Loaded CSV/JSON/binary files, generated comparison plots | (App doesn't know - external post-processing) | N/A - firmware completes its role by creating files |

### §7.3 Derived Ownership Rules
*These become Blueprint §2.3 Boundary Rules*

| Source | Rule | Rationale |
|--------|------|----------|
| O-1, O-2, O-3 | App MUST create all log files with unique timestamped filenames to prevent overwrites | User expects each logging session to produce distinct files |
| O-4 | App MUST embed dashboard HTML/JS at compile time (stored in LittleFS) | Ensures dashboard always available |
| O-10 | App MUST NOT modify DSP algorithm behavior when HIL monitoring enabled | Monitoring is observation-only to ensure captured data reflects actual operation |
| E-1 | App MUST handle LittleFS errors gracefully and fall back to serial logging | Flash filesystem can fail or fill up |
| E-2 | App MUST continue logging even if WebSocket clients disconnect | Real-time dashboard is optional - data collection must be robust to network issues |
| O-6 | App MUST handle WebSocket send buffer full condition by dropping frames (not blocking) | Real-time audio pipeline cannot be blocked by slow network clients |
| O-7 | App MUST flush serial buffer periodically to prevent overruns during high-volume logging | Serial at 2Mbaud has limited buffer - must manage backpressure |
| O-8, O-9 | App MUST NOT attempt to load external Python scripts or test signals | External analysis tools operate on app-created files, not the reverse |

## §8. State Requirements
*→ Extracted into Blueprint §3 State Transition Specifications*

### §8.1 State Isolation

| ID | Requirement | Rationale | Enforcement |
|----|-------------|-----------|-------------|
| SI-1 | HIL monitoring state must be fully independent of light mode selection and configuration | User must be able to log data while freely changing visualization modes without affecting capture | Monitoring state stored in separate globals (not in light_modes[] or configuration struct) |
| SI-2 | Logging state (CSV/binary/WebSocket) must not affect DSP algorithm execution timing | DSP pipeline must run identically whether monitoring is active or inactive (except for overhead) | DSP functions must not branch on logging flags - monitoring reads completed results |
| SI-3 | Multiple WebSocket clients must each independently subscribe to different signal arrays | Client A viewing spectrogram should not affect Client B viewing tempo, and vice versa | Per-client subscription bitmask tracking which arrays each client wants |
| SI-4 | SD card logging and WebSocket streaming must operate concurrently without interference | User running dashboard while also logging to SD card should work without data corruption | Separate buffer copies for file writes vs WebSocket sends |
| SI-5 | Dashboard visibility must not alter captured data content or format | Same CSV/JSON/binary files must be produced whether dashboard is connected or not | Dashboard subscribes passively - no control messages affect logging behavior |

### §8.2 State Lifecycle

| ID | State | Initial | Created When | Cleared When | Persists Across |
|----|-------|---------|--------------|--------------|------------------|
| SL-1 | HIL monitoring enabled flag | false | On boot if HIL_EXTENDED defined, or via ENABLE_HIL command | Never (compile-time or explicit DISABLE_HIL command) | Mode changes, reboots (if compiled in) |
| SL-2 | CSV log file handle | NULL | log\|start\|file command received | log\|stop command or frame limit reached | Not cleared - file handle maintained until stop |
| SL-3 | Binary dump file handles (per array type) | NULL | LOG_START:BINARY command received | LOG_STOP or manual rotation | Rotated regularly, not closed on errors (retried) |
| SL-4 | WebSocket client subscription masks (per client) | 0x0000 (none) | SUBSCRIBE:array_name message received from client | Client disconnect or UNSUBSCRIBE message | Survives temporary network glitches if connection maintained |
| SL-5 | Signal capture buffers (temp copies for logging) | Unallocated | First LOG_START command (malloc on demand) | LOG_STOP frees buffers to reclaim memory | Reallocated on next LOG_START |
| SL-6 | Frame counter (logs written) | 0 | LOG_START command | LOG_STOP or file rotation (resets to 0 for new file) | Survives brief pauses, reset on new log session |
| SL-7 | Timing statistics (min/max/avg per metric) | Cleared | First monitoring sample collected | On dashboard "Reset Stats" button or after 100k samples (overflow prevention) | Survives mode changes, cleared manually only |
| SL-8 | JSON snapshot buffer (recent signal history) | Empty | First frame after monitoring enabled | On snapshot export (copied then cleared) or after 10s history filled (circular overwrite) | Circular buffer, never fully cleared |
| SL-9 | Filesystem error state | No error | LittleFS.open() fails or file.write() fails | Successful write after error | Survives across failed writes, cleared on success |
| SL-10 | Serial logging state | Inactive | LOG_START:SERIAL command or SD fallback | LOG_STOP command | Not affected by WebSocket state, independent control |

## §9. Technical Considerations
*→ Informs Blueprint §5, §6, §9, §11*

### §9.1 Architecture Decisions

**Decision 1: Extend existing profiler.h and hil_monitor.h rather than new infrastructure**
- **Rationale**: profiler.h already captures function-level timing with ESP.getCycleCount() and hil_monitor.h has WebSocket broadcasting every 250ms. Extending proven infrastructure reduces development risk vs building from scratch.
- **Approach**: Add logArray() and logSignal() methods to HILMonitor class for multi-value logging. Extend profile_function() with optional array capture callback.

**Decision 2: Conditional compilation via #ifdef HIL_EXTENDED for full instrumentation**
- **Rationale**: Core monitoring (timing only) should be always-on with minimal overhead. Extended monitoring (full signal dumps) significantly increases memory usage and processing time, so make it opt-in.
- **Approach**: #define HIL_EXTENDED in platformio.ini build flags to enable array logging and binary dumps. Base builds only log timing metrics.

**Decision 3: Copy-on-capture for signal arrays to avoid blocking DSP pipeline**
- **Rationale**: memcpy of spectrogram[64] or tempi[96] takes ~1-5µs, but serializing to CSV/binary takes hundreds of microseconds. Copy first, serialize async.
- **Approach**: Allocate separate capture buffers, memcpy in DSP context, queue for background serialization (FreeRTOS task or deferred work).

**Decision 4: Binary WebSocket frames for signal data, text frames for metrics**
- **Rationale**: float32 arrays as JSON text: spectrogram[64] = 64 * 20 chars = 1280 bytes. Binary: 64 * 4 bytes = 256 bytes. 5x efficiency for real-time streaming.
- **Approach**: Extend broadcast() to support binary frames. Clients parse frame type byte to dispatch to JSON or binary handler.

**Decision 5: Serial primary for continuous logging, LittleFS for snapshots, WebSocket for real-time**
- **Rationale**: Device has no SD card - only LittleFS (~1.5MB). Serial at 2Mbaud can stream ~200KB/s continuously to external capture. LittleFS useful for small CSV snapshots. WebSocket is real-time but connection can drop.
- **Approach**: Serial for multi-hour continuous logging, LittleFS for short captures (~500 frames), WebSocket for real-time dashboard updates.

**Decision 6: Python analysis scripts external to firmware**
- **Rationale**: Comparison plotting requires matplotlib/numpy (not available on ESP32). Keep firmware focused on data capture, do analysis on PC.
- **Approach**: Provide reference Python scripts in /tools directory that load CSV/JSON/binary files and generate comparison reports.

### §9.2 Integration Points

**Integration Point 1: cpu_core.h audio pipeline instrumentation**
- Wrap existing DSP function calls with signal capture: after calculate_magnitudes(), memcpy(spectrogram_capture, spectrogram, sizeof(spectrogram))
- Add calls to HILMonitor::logArray("spectrogram", spectrogram_capture, 64) after each captured stage
- Insert timing checkpoints using existing uint32_t t_start = micros() pattern around new capture operations

**Integration Point 2: gpu_core.h visual pipeline instrumentation**
- Wrap light_modes[configuration.current_mode].draw() with profile_function() to capture per-mode timing
- Add leds[] buffer capture after each post-processing stage: memcpy(leds_capture, leds, sizeof(CRGBF) * NUM_LEDS)
- Log intermediate buffers with HILMonitor::logArray("leds_post_lpf", leds_capture, NUM_LEDS * 3) (3 floats per LED)

**Integration Point 3: wireless.h WebSocket server extension**
- Add binary frame support to PsychicHttp WebSocket handler: check frame opcode for binary vs text
- Implement subscription management: clients send "SUBSCRIBE:spectrogram" to register for binary updates
- Extend broadcastStats() to also send subscribed binary arrays to each client based on subscription mask

**Integration Point 4: hil_export.h LittleFS/Serial logging**
- LittleFS logging: create timestamped CSV file, write rows until frame limit, auto-close
- Serial logging: stream CSV rows at 2Mbaud for external capture tools
- Implement CSV write function: format row as "timestamp,vu_level,vu_max,...,spectrogram_0,...,spectrogram_63\n"

**Integration Point 5: commands.h command processing**
- Add new commands: log|start|serial, log|start|file, log|stop, log|status
- Implement command responses: send back acknowledgment "log_started|serial" or error "log_error|failed_to_create_file"
- Add dashboard control message handling: buttons send commands via WebSocket that route to same command processor

### §9.3 Constraints

**Constraint 1: ESP32-S3 memory limits**
- Total RAM: ~500KB available after framework overhead. Each float32 array capture: spectrogram[64]=256B, novelty_curve[1024]=4KB, tempi[96]=384B.
- Implication: Cannot buffer thousands of frames in RAM. Must write to SD card or stream incrementally.
- Mitigation: Allocate capture buffers only when logging active (malloc on LOG_START, free on LOG_STOP).

**Constraint 2: No SD card available**
- Device only has LittleFS (~1.5MB flash partition). Cannot store multi-hour logs locally.
- Implication: Must use serial streaming for continuous logging, captured by external PC tool.
- Mitigation: Serial at 2Mbaud provides ~200KB/s throughput. Use `picocom`, `screen`, or Python script to capture.

**Constraint 3: WebSocket send buffer size**
- PsychicHttp WebSocket buffer: ~16KB. Sending full arrays every frame: spectrogram[64]=256B + novelty[1024]=4KB + tempi[96]=384B = ~4.6KB per frame at 200 FPS = 920KB/s.
- Implication: Must throttle WebSocket updates or clients will lag behind real-time.
- Mitigation: Decimate WebSocket updates to 30 FPS (every 6-7 audio frames), or subscribe to subset of arrays.

**Constraint 4: No performance optimization allowed**
- User explicitly stated: "I don't care about device suffering from performance degradation, I only care about extracting metrics."
- Implication: Can use simple/slow approaches (memcpy everything, log every frame) without optimization.
- Benefit: Reduces development complexity - no need for clever buffering or selective logging heuristics.

**Constraint 5: Dashboard is lowest priority**
- User: "Highest priority is the initial algorithm reverse-engineering work - focus on data logging infrastructure first."
- Implication: Dashboard is nice-to-have for real-time debugging but not required for core goal.
- Approach: Implement CSV/binary/JSON export first, dashboard as time permits.

## §10. Success Metrics
*→ Informs Blueprint §10 Testing Strategy*

| ID | Metric | Target | Measurement Method |
|----|--------|--------|--------------------|
| M-1 | Algorithm reverse-engineering completeness | 100% of intermediate DSP signals captured: spectrogram[64], novelty_curve[1024], tempi[96], chromagram[12], sample_history[4096] | Manual verification: load CSV export, confirm all expected columns present and populated |
| M-2 | Timing breakdown granularity | <1 microsecond resolution, all audio DSP stages individually measured | Check CSV export contains DSP_ACQUIRE_US, DSP_MAGNITUDES_US, DSP_VU_US, DSP_TEMPO_US with sub-microsecond precision |
| M-3 | Data export format coverage | All 5 formats functional: CSV, JSON, binary, WebSocket, serial | Run test capturing to each format, verify files/streams created and parseable |
| M-4 | Multi-hour logging reliability | Zero data loss over 4-hour continuous serial logging session (800,000+ frames at 200 FPS) | Run overnight serial logging with external capture tool, verify frame counter increments continuously, no gaps in CSV row numbers |
| M-5 | Benchmarking capability | Timestamp alignment accuracy <10ms between v1.1, Lightwave-Ledstrip, and Tab5.DSP logs for same test signal | Play test signal to all three systems, load CSVs, compute cross-correlation of spectrograms, verify peak within 10ms offset |
| M-6 | Visual pipeline instrumentation completeness | All 12+ light mode algorithms individually timed, all post-processing stages timed | Check CSV export contains per-mode timing columns: FFT_DRAW_US, SPECTRUM_DRAW_US, etc., GPU_LPF_US, GPU_TONEMAP_US, etc. |
| M-7 | Signal visualization responsiveness | Dashboard plots update at 30 FPS with <100ms latency from DSP pipeline to browser display | Open dashboard, trigger audio event (clap), measure time until spectrogram/novelty plot shows response |
| M-8 | Serial streaming efficiency | CSV streaming rate sustainable at 2Mbaud (~200KB/s), no serial buffer overruns | Measure serial output throughput during continuous logging, verify no dropped frames |
| M-9 | Python analysis workflow | Load CSV/binary exports into pandas/NumPy and generate comparison plots in <60 seconds for 1-minute recording | Run provided analysis script on test data, time execution, verify plots generated without errors |
| M-10 | No critical metrics excluded | Timing breakdowns present, CPU/RAM/FPS metrics absent (per user requirements) | Verify CSV columns contain timing metrics only, no CPU_USAGE or FPS_CPU columns (unless used for debugging) |

---

## Appendix A: Answer Traceability

| User Answer (Summary) | Captured In |
|-----------------------|-------------|
| Q1: Audio pipeline first - Goertzel, spectral flux, tempo deep-dive | §1 Overview (Key Outcomes), §2 Problem (Black Box Algorithms), §5.1-§5.5 (Audio Pipeline Epic), FR-1, FR-2 |
| Q1: Visual pipeline follows audio - custom LED driver, multi-layer manipulation | §5.6-§5.8 (Visual Pipeline Epic), FR-3, FR-4, §9.2 Integration Point 2 |
| Q1: Benchmarking infrastructure third priority | §5.14-§5.15 (Benchmarking Epic), FR-7, M-5 |
| Q1: Dashboard if time/resources (lowest priority) | §5.16-§5.18 (Dashboard Epic - marked optional/secondary), FR-12, §9.3 Constraint 5 |
| Q2: No performance constraints - only care about extracting metrics | NFR-1, NFR-2, NFR-3, §9.3 Constraint 4 |
| Q3: All data formats - CSV, JSON, binary, WebSocket, logs | FR-5, FR-6, FR-7, FR-8, FR-9, §5.9-§5.13 (Multi-Format Export Epic), M-3 |
| Q4: Timing breakdowns absolutely critical | FR-2, §5.1-§5.8 (all timing acceptance criteria), NFR-5, M-2, §5.16 (Timing Breakdown Display) |
| Q4: Real-time signal visualization valuable | FR-12, §5.17 (Signal Visualization), FR-8 (WebSocket streaming) |
| Q4: Trigger captures, metric selection nice to have | FR-10, FR-11, §5.18 (Capture Controls) |
| Q4: Data logging infrastructure priority | §1 Overview (Key Outcomes emphasis on data export), §9.3 Constraint 5 |
| Q4: Don't care about CPU/RAM/FPS metrics | NFR-8, M-10, §5.16.5 (explicit exclusion) |

**Validation:** ✅ All user answers have been captured. No information lost.
