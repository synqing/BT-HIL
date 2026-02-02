# HIL Monitoring System - Implementation Tasks

> **Traceability Note:** This task breakdown extracts directly from PRD §4-§5 and Technical Blueprint §8 Implementation Phases. All tasks map to specific PRD user stories and functional requirements.

## Quick Reference

### Ownership Matrix
| Component | Owner | Integration Point |
|-----------|-------|-------------------|
| Audio Signal Capture | Core 1 (CPU) @ cpu_core.h:17 | hil_capture.h buffers |
| CSV Export | SD Card Task | hil_export.h state |
| WebSocket Streaming | Core 0 (GPU) via wireless.h | Per-client subscriptions |
| Visual Timing | Core 0 (GPU) @ gpu_core.h:17 | profiler.h infrastructure |
| Dashboard (Optional) | External (web browser) | PsychicHttp /dashboard |

### State Variables (from Blueprint §6)
| Variable | Type | Scope | Purpose |
|----------|------|-------|---------|
| `SL-1: monitoring_active` | bool | Global | Enables HIL capture |
| `SL-2: csv_log_file` | File | hil_export.h | Active CSV handle |
| `SL-4: client_subscription_masks[4]` | uint16_t[] | hil_export.h | WebSocket subscriptions |
| `SL-5: capture_buffers` | struct | hil_capture.h | In-memory signal arrays |
| `SL-6: frame_counter` | uint32_t | hil_export.h | Frames since LOG_START |
| `SL-9: sd_card_error` | bool | hil_export.h | Fallback trigger flag |

### Critical Boundaries (from Blueprint §2.3)
| Rule | Description | Implementation |
|------|-------------|----------------|
| **E-1** | Monitoring never affects audio DSP output | memcpy AFTER all DSP functions, no branches in DSP code |
| **E-2** | WebSocket disconnect doesn't stop logging | SD/serial logging independent of clients |
| **O-1** | ESP32 creates all SD card files | No external file management |
| **O-7** | Serial writes never block audio processing | Use Serial.flush() periodically, drop data if buffer full |
| **SI-4** | Logging state survives WebSocket reconnect | State in hil_export.h, not tied to client connection |

---

## Implementation Phases (8 Phases, 89 Tasks)

Priority Order: Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5 → Phase 6 → Phase 7 (Optional) → Phase 8

---

## PHASE 1: Audio Pipeline Signal Capture Infrastructure
*Duration: Highest Priority*
*Implements: PRD Epic §5.1-§5.5, FR-1 (Goertzel), FR-2 (Tempo), Blueprint §8 Phase 1*

### Task 1.1: Create Capture Buffer Infrastructure

**Pre-conditions:**
- `#define HIL_EXTENDED` added to build flags
- Heap memory available (~30KB for full capture buffers)

**Post-conditions:**
- SL-5: `hil_capture_state` struct allocated on boot
- All buffer pointers initialized and non-NULL
- Buffers accessible from both Core 0 and Core 1 via shared memory

**Parent Task:** Create hil_capture.h with capture buffer structures
- **Input:** PRD §5.1-§5.5 signal requirements, Blueprint §6.1 data models
- **Output:** hil_capture.h header file with struct hil_capture_state
- **Ownership:** ESP32 firmware (v1.1_build/hil_capture.h)
- **User Sees:** Nothing (internal infrastructure)
- **State Change:** SL-5 buffers allocated
- **Integration:** Called from main setup() before Core 1 CPU loop starts
- **Implements:** Blueprint Phase 1 checkpoint 1

**Subtasks:**

#### 1.1.1: Define hil_capture_state struct
- **Input:** Blueprint §6.1 data model specification
- **Output:** C++ struct with typed fields for all signal arrays
- **Ownership:** v1.1_build/hil_capture.h (new file)
- **User Sees:** Nothing
- **State Change:** None (definition only)
- **Integration:** Included by cpu_core.h and hil_export.h
- **Implements:** Blueprint §6.1, SL-5 schema
- **Code:**
```cpp
struct hil_capture_state {
    // Audio DSP captures (PRD §5.1-§5.5)
    float spectrogram_capture[64];              // §5.1.1
    float spectrogram_smooth_capture[64];       // §5.1.2
    float chromagram_capture[12];               // §5.1.3
    float vu_level_capture;                     // §5.4.1
    float vu_max_capture;                       // §5.4.2
    float vu_floor_capture;                     // §5.4.2
    float novelty_curve_capture[1024];          // §5.2.1
    float novelty_curve_normalized_capture[1024]; // §5.2.4
    float tempi_magnitude_capture[96];          // §5.3.1
    float tempi_phase_capture[96];              // §5.3.2
    float tempi_beat_capture[96];               // §5.3.3

    // Optional: Raw I2S samples (§5.5.1)
    float sample_history_capture[4096];         // Captured every Nth frame
    uint32_t sample_history_capture_counter;
};
```

#### 1.1.2: Allocate capture buffers on boot
- **Input:** SL-1 `monitoring_active` flag (true if #define HIL_EXTENDED)
- **Output:** Heap-allocated `hil_capture_state*` global pointer
- **Ownership:** v1.1_build/v1.1_build.ino setup() function
- **User Sees:** Serial message: "HIL Extended Monitoring: ENABLED (30KB allocated)"
- **State Change:** SL-5 from NULL to valid pointer
- **Integration:** Called before init_system(), memory committed before dual-core starts
- **Implements:** Blueprint Phase 1 checkpoint 2, NFR-1 (no performance constraints)
- **Validation:** Check malloc() return != NULL, print error if allocation fails

#### 1.1.3: Add access helpers for cross-core safety
- **Input:** hil_capture_state pointer, magnitudes_locked volatile flag pattern
- **Output:** Inline getter functions with memory barriers
- **Ownership:** v1.1_build/hil_capture.h
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Used by cpu_core.h (writes) and hil_export.h (reads)
- **Implements:** Blueprint SI-1 (state isolation between cores)
- **Code Pattern:**
```cpp
inline void capture_spectrogram_safe(float* dest, const float* src, size_t len) {
    portDISABLE_INTERRUPTS();
    memcpy(dest, src, len * sizeof(float));
    portENABLE_INTERRUPTS();
}
```

---

### Task 1.2: Instrument Goertzel Frequency Analysis

**Pre-conditions:**
- hil_capture_state allocated (Task 1.1 complete)
- calculate_magnitudes() in goertzel.h computes spectrogram[64]

**Post-conditions:**
- spectrogram[64] copied to capture buffer every audio frame
- spectrogram_smooth[64] copied to capture buffer every frame
- chromagram[12] copied to capture buffer every frame
- Timing overhead <5µs measured

**Parent Task:** Capture Goertzel outputs after calculate_magnitudes()
- **Input:** spectrogram[], spectrogram_smooth[], chromagram[] from goertzel.h
- **Output:** Captured arrays in hil_capture_state
- **Ownership:** cpu_core.h:38 (after calculate_magnitudes() call)
- **User Sees:** Nothing (silent capture)
- **State Change:** SL-5 arrays updated every frame
- **Integration:** Inserted in run_cpu() after line 38
- **Implements:** PRD §5.1 (Goertzel Capture Story), Blueprint Phase 1 checkpoint 3-4

**Subtasks:**

#### 1.2.1: Add memcpy after calculate_magnitudes()
- **Input:** global spectrogram[NUM_FREQS], spectrogram_smooth[NUM_FREQS]
- **Output:** hil_capture_state->spectrogram_capture[], hil_capture_state->spectrogram_smooth_capture[]
- **Ownership:** cpu_core.h line 38 (after calculate_magnitudes())
- **User Sees:** Nothing
- **State Change:** SL-5 spectrogram arrays updated
- **Integration:** Runs on Core 1 CPU thread, ~200 FPS
- **Implements:** PRD §5.1.1, §5.1.2
- **Code:**
```cpp
calculate_magnitudes();  // existing line 38
#ifdef HIL_EXTENDED
    memcpy(hil_capture_state->spectrogram_capture, spectrogram, NUM_FREQS * sizeof(float));
    memcpy(hil_capture_state->spectrogram_smooth_capture, spectrogram_smooth, NUM_FREQS * sizeof(float));
#endif
```

#### 1.2.2: Add memcpy after get_chromagram()
- **Input:** global chromagram[12]
- **Output:** hil_capture_state->chromagram_capture[]
- **Ownership:** cpu_core.h line 41 (after get_chromagram())
- **User Sees:** Nothing
- **State Change:** SL-5 chromagram array updated
- **Integration:** Runs on Core 1 CPU thread
- **Implements:** PRD §5.1.3
- **Code:**
```cpp
get_chromagram();  // existing line 41
#ifdef HIL_EXTENDED
    memcpy(hil_capture_state->chromagram_capture, chromagram, 12 * sizeof(float));
#endif
```

#### 1.2.3: Measure and log timing overhead
- **Input:** ESP.getCycleCount() before/after memcpy block
- **Output:** HIL|CAPTURE_OVERHEAD_US logged via broadcast()
- **Ownership:** cpu_core.h after memcpy blocks
- **User Sees:** WebSocket message: "HIL|CAPTURE_OVERHEAD_US|4.2"
- **State Change:** None
- **Integration:** Logged every 250ms (same interval as profiler stats)
- **Implements:** Blueprint Phase 1 checkpoint 8 (verify <20µs), NFR-5 (timing visibility)
- **Validation:** Assert overhead <20µs, warn if exceeded

---

### Task 1.3: Instrument VU Meter and Dynamic Range

**Pre-conditions:**
- hil_capture_state allocated (Task 1.1 complete)
- run_vu() in vu.h computes vu_level, vu_max, vu_floor

**Post-conditions:**
- vu_level, vu_max, vu_floor copied to capture buffer every frame
- VU smoothing buffer vu_smooth[12] captured if defined

**Parent Task:** Capture VU meter outputs after run_vu()
- **Input:** vu_level, vu_max, vu_floor globals from vu.h
- **Output:** Scalar values in hil_capture_state
- **Ownership:** cpu_core.h:44 (after run_vu())
- **User Sees:** Nothing
- **State Change:** SL-5 VU scalars updated
- **Integration:** Inserted in run_cpu() after line 44
- **Implements:** PRD §5.4 (VU Meter Story), Blueprint Phase 1 checkpoint 5

**Subtasks:**

#### 1.3.1: Add memcpy after run_vu()
- **Input:** vu_level, vu_max, vu_floor globals
- **Output:** hil_capture_state->{vu_level_capture, vu_max_capture, vu_floor_capture}
- **Ownership:** cpu_core.h line 44
- **User Sees:** Nothing
- **State Change:** SL-5 VU scalars updated
- **Integration:** Core 1 CPU thread
- **Implements:** PRD §5.4.1, §5.4.2
- **Code:**
```cpp
run_vu();  // existing line 44
#ifdef HIL_EXTENDED
    hil_capture_state->vu_level_capture = vu_level;
    hil_capture_state->vu_max_capture = vu_max;
    hil_capture_state->vu_floor_capture = vu_floor;
#endif
```

---

### Task 1.4: Instrument Spectral Flux Novelty Detection

**Pre-conditions:**
- hil_capture_state allocated (Task 1.1 complete)
- update_novelty() in tempo.h computes novelty_curve[1024], novelty_curve_normalized[1024]

**Post-conditions:**
- novelty_curve[1024] and novelty_curve_normalized[1024] copied every frame
- 1024-sample circular buffer captured at 50 Hz

**Parent Task:** Capture novelty detection outputs after update_novelty()
- **Input:** novelty_curve[], novelty_curve_normalized[] from tempo.h
- **Output:** 1024-sample arrays in hil_capture_state
- **Ownership:** gpu_core.h:33 (after update_novelty())
- **User Sees:** Nothing
- **State Change:** SL-5 novelty arrays updated
- **Integration:** Runs on Core 0 GPU thread (novelty updated in gpu_core)
- **Implements:** PRD §5.2 (Spectral Flux Story), Blueprint Phase 1 checkpoint 6

**Subtasks:**

#### 1.4.1: Add memcpy after update_novelty()
- **Input:** novelty_curve[1024], novelty_curve_normalized[1024]
- **Output:** hil_capture_state->{novelty_curve_capture[], novelty_curve_normalized_capture[]}
- **Ownership:** gpu_core.h line 33
- **User Sees:** Nothing
- **State Change:** SL-5 novelty arrays updated
- **Integration:** Core 0 GPU thread, ~60 FPS (GPU runs slower than CPU audio loop)
- **Implements:** PRD §5.2.1, §5.2.4
- **Code:**
```cpp
update_novelty();  // existing line 33
#ifdef HIL_EXTENDED
    memcpy(hil_capture_state->novelty_curve_capture, novelty_curve, 1024 * sizeof(float));
    memcpy(hil_capture_state->novelty_curve_normalized_capture, novelty_curve_normalized, 1024 * sizeof(float));
#endif
```

---

### Task 1.5: Instrument Tempo/Beat Detection

**Pre-conditions:**
- hil_capture_state allocated (Task 1.1 complete)
- update_tempo() in tempo.h computes tempi[96].magnitude, .phase, .beat

**Post-conditions:**
- tempi[96] magnitude, phase, beat arrays copied every frame
- Tempo algorithm parameters logged (block_size, coeff, BEAT_SHIFT_PERCENT)

**Parent Task:** Capture tempo detection outputs after update_tempo()
- **Input:** tempi[96] struct array from tempo.h
- **Output:** 96-element arrays for magnitude, phase, beat in hil_capture_state
- **Ownership:** cpu_core.h after update_tempo() call
- **User Sees:** Nothing
- **State Change:** SL-5 tempo arrays updated
- **Integration:** Inserted in run_cpu() after tempo update
- **Implements:** PRD §5.3 (Tempo Detection Story), Blueprint Phase 1 checkpoint 7

**Subtasks:**

#### 1.5.1: Add memcpy after update_tempo()
- **Input:** tempi[96].magnitude, tempi[96].phase, tempi[96].beat
- **Output:** hil_capture_state->{tempi_magnitude_capture[], tempi_phase_capture[], tempi_beat_capture[]}
- **Ownership:** cpu_core.h after update_tempo()
- **User Sees:** Nothing
- **State Change:** SL-5 tempo arrays updated
- **Integration:** Core 1 CPU thread
- **Implements:** PRD §5.3.1, §5.3.2, §5.3.3
- **Code:**
```cpp
update_tempo(delta);  // existing tempo update call
#ifdef HIL_EXTENDED
    for (uint8_t i = 0; i < 96; i++) {
        hil_capture_state->tempi_magnitude_capture[i] = tempi[i].magnitude;
        hil_capture_state->tempi_phase_capture[i] = tempi[i].phase;
        hil_capture_state->tempi_beat_capture[i] = tempi[i].beat;
    }
#endif
```

#### 1.5.2: Log tempo algorithm parameters on boot
- **Input:** tempi[96].block_size, .coeff, BEAT_SHIFT_PERCENT constant
- **Output:** HIL|TEMPO_PARAMS|JSON logged once at boot
- **Ownership:** tempo.h init_tempo() function
- **User Sees:** WebSocket message with JSON tempo configuration
- **State Change:** None (read-only config)
- **Integration:** Called from init_system()
- **Implements:** PRD §5.3.6 (tempo parameters)

---

### Task 1.6: Optional I2S Raw Sample Capture

**Pre-conditions:**
- hil_capture_state allocated with sample_history_capture[4096]
- acquire_sample_chunk() populates sample_history[]

**Post-conditions:**
- sample_history[4096] captured every Nth frame (e.g., N=10 to reduce data volume)
- I2S health metrics logged: sample rate accuracy, DMA errors, clipping events

**Parent Task:** Capture raw I2S samples and health metrics
- **Input:** sample_history[4096] from microphone.h, I2S driver status
- **Output:** Periodic raw audio snapshots and I2S health metrics
- **Ownership:** cpu_core.h after acquire_sample_chunk()
- **User Sees:** Nothing (large data, stored not broadcast)
- **State Change:** SL-5 sample_history_capture updated every Nth frame
- **Integration:** Conditional capture based on frame counter
- **Implements:** PRD §5.5 (I2S Audio Capture Story), Blueprint Phase 1 optional checkpoint

**Subtasks:**

#### 1.6.1: Add periodic sample_history capture
- **Input:** sample_history[4096], frame_counter % 10 == 0
- **Output:** hil_capture_state->sample_history_capture[] every 10th frame
- **Ownership:** cpu_core.h after acquire_sample_chunk()
- **User Sees:** Nothing
- **State Change:** SL-5 sample buffer updated every 10 frames (~20 Hz)
- **Integration:** Core 1 CPU thread
- **Implements:** PRD §5.5.1
- **Code:**
```cpp
acquire_sample_chunk();  // existing line 31
#ifdef HIL_EXTENDED
    if (hil_capture_state->sample_history_capture_counter++ % 10 == 0) {
        memcpy(hil_capture_state->sample_history_capture, sample_history, 4096 * sizeof(float));
    }
#endif
```

#### 1.6.2: Log I2S health metrics
- **Input:** I2S driver status (i2s_get_status), sample rate counter
- **Output:** HIL|I2S_HEALTH|{sample_rate_actual, dma_errors, clipping_events}
- **Ownership:** microphone.h or cpu_core.h
- **User Sees:** WebSocket message every 250ms
- **State Change:** None (read-only status)
- **Integration:** Logged alongside profiler stats
- **Implements:** PRD §5.5.2, §5.5.3, §5.5.4

---

## PHASE 2: CSV Export Implementation
*Duration: High Priority (after Phase 1)*
*Implements: PRD Epic §5.9, FR-5, Blueprint §8 Phase 2*

### Task 2.1: Create Export State Infrastructure

**Pre-conditions:**
- hil_capture_state accessible (Phase 1 complete)
- SD card library included (SD.h or SD_MMC.h)

**Post-conditions:**
- SL-2: csv_log_file File handle available
- SL-6: frame_counter initialized
- SL-9: sd_card_error flag ready

**Parent Task:** Create hil_export.h with export state structures
- **Input:** Blueprint §6.1 export state schema
- **Output:** hil_export.h header file with struct hil_export_state
- **Ownership:** v1.1_build/hil_export.h (new file)
- **User Sees:** Nothing
- **State Change:** SL-2, SL-6, SL-9 variables defined
- **Integration:** Included by commands.h for LOG_START/LOG_STOP handlers
- **Implements:** Blueprint Phase 2 checkpoint 1, SL-2/SL-6/SL-9 state variables

**Subtasks:**

#### 2.1.1: Define hil_export_state struct
- **Input:** Blueprint §6.1 export state specification
- **Output:** C++ struct with File handles, counters, flags
- **Ownership:** v1.1_build/hil_export.h
- **User Sees:** Nothing
- **State Change:** None (definition only)
- **Integration:** Included by commands.h
- **Implements:** SL-2, SL-6, SL-9, SL-10 state schema
- **Code:**
```cpp
struct hil_export_state {
    File csv_log_file;              // SL-2: Active CSV file handle
    uint32_t frame_counter;         // SL-6: Frames logged since LOG_START
    char csv_filename[64];          // Timestamped filename
    bool sd_card_error;             // SL-9: Fallback trigger
    bool serial_logging_active;     // SL-10: Serial logging mode
    uint16_t client_subscription_masks[4];  // SL-4: WebSocket subscriptions
};
```

---

### Task 2.2: Implement LOG_START:CSV Command

**Pre-conditions:**
- hil_export_state defined (Task 2.1 complete)
- SD card initialized and mounted
- commands.h command parsing infrastructure exists

**Post-conditions:**
- SL-2: CSV file created on SD card with unique timestamped filename
- CSV header row written
- SL-6: frame_counter reset to 0
- V-2: Status broadcast: "LOG_STATUS|ACTIVE|CSV|filename.csv"

**Parent Task:** Handle LOG_START:CSV command to begin CSV logging
- **Input:** "LOG_START:CSV" command string from WebSocket or serial
- **Output:** Timestamped CSV file on SD card, header written
- **Ownership:** commands.h handle_command() function
- **User Sees:** WebSocket broadcast: "LOG_STATUS|ACTIVE|CSV|emotiscope_2025-01-14_152045.csv"
- **State Change:** SL-1 monitoring_active=true, SL-2 file opened, SL-6 counter=0
- **Integration:** Called from wireless.h onMessage() callback
- **Implements:** PRD §5.9.4, Blueprint Phase 2 checkpoint 2-3, V-2 visibility

**Subtasks:**

#### 2.2.1: Generate timestamped CSV filename
- **Input:** millis() or RTC timestamp
- **Output:** Filename string: "emotiscope_YYYY-MM-DD_HHMMSS.csv"
- **Ownership:** hil_export.h create_timestamped_filename() helper
- **User Sees:** Filename in LOG_STATUS broadcast
- **State Change:** hil_export_state->csv_filename populated
- **Integration:** Called by LOG_START command handler
- **Implements:** PRD §5.9.4, O-1 (ESP32 creates files)
- **Code:**
```cpp
void create_timestamped_filename(char* buffer, size_t len) {
    uint32_t ms = millis();
    snprintf(buffer, len, "emotiscope_%010lu.csv", ms);
}
```

#### 2.2.2: Open CSV file on SD card
- **Input:** Filename from 2.2.1, SD card mount point
- **Output:** SL-2 csv_log_file File handle
- **Ownership:** commands.h LOG_START handler
- **User Sees:** Error message if SD card fails: "SD CARD ERROR - cannot create file"
- **State Change:** SL-2 from NULL to valid File, or SL-9=true if error
- **Integration:** SD.open(filename, FILE_WRITE)
- **Implements:** PRD §5.9.4, E-1 (SD error handling)
- **Validation:** Check File object validity, set SL-9=true if open fails

#### 2.2.3: Write CSV header row
- **Input:** All signal array names and timing metric names
- **Output:** CSV header line: "timestamp_ms,DSP_ACQUIRE_US,DSP_MAGNITUDES_US,..."
- **Ownership:** hil_export.h write_csv_header() function
- **User Sees:** Nothing (internal to file)
- **State Change:** CSV file first line written
- **Integration:** Called immediately after file open in LOG_START handler
- **Implements:** PRD §5.9.1, Blueprint Phase 2 checkpoint 4
- **Code:**
```cpp
void write_csv_header(File& f) {
    f.print("timestamp_ms,DSP_ACQUIRE_US,DSP_MAGNITUDES_US,DSP_VU_US,DSP_TEMPO_US,");
    for (uint8_t i = 0; i < 64; i++) f.printf("spectrogram_%d,", i);
    for (uint16_t i = 0; i < 1024; i++) f.printf("novelty_%d,", i);
    for (uint8_t i = 0; i < 96; i++) f.printf("tempi_mag_%d,", i);
    for (uint8_t i = 0; i < 96; i++) f.printf("tempi_phase_%d,", i);
    f.println();
}
```

#### 2.2.4: Reset frame counter and broadcast status
- **Input:** Command completion status
- **Output:** SL-6 frame_counter=0, V-2 status broadcast
- **Ownership:** commands.h LOG_START handler end
- **User Sees:** "LOG_STATUS|ACTIVE|CSV|emotiscope_1234567890.csv"
- **State Change:** SL-6=0, V-2 status updated
- **Integration:** broadcast() call at end of LOG_START handler
- **Implements:** V-2 (user visibility), SL-6 initialization

---

### Task 2.3: Implement Per-Frame CSV Row Export

**Pre-conditions:**
- CSV file open (SL-2 valid)
- hil_capture_state populated with current frame data

**Post-conditions:**
- CSV row written to SD card every audio frame
- SL-6 frame_counter incremented
- File flushed periodically to prevent corruption

**Parent Task:** Write CSV row for every captured frame
- **Input:** hil_capture_state arrays, hil_frame_timing struct
- **Output:** CSV row appended to file: "1234567890,287,1523,412,..."
- **Ownership:** cpu_core.h end of run_cpu() loop or dedicated export task
- **User Sees:** Nothing (silent logging), status updated every 250ms
- **State Change:** SL-2 file grows, SL-6 increments
- **Integration:** Called at end of audio frame processing
- **Implements:** PRD §5.9.2, §5.9.3, Blueprint Phase 2 checkpoint 5-7

**Subtasks:**

#### 2.3.1: Format CSV row with timestamp and timing
- **Input:** millis(), DSP_ACQUIRE_US, DSP_MAGNITUDES_US, DSP_VU_US, DSP_TEMPO_US
- **Output:** CSV string: "1234567890,287,1523,412,3456,"
- **Ownership:** hil_export.h format_csv_row_prefix() helper
- **User Sees:** Nothing
- **State Change:** None (formatting only)
- **Integration:** Called before array serialization
- **Implements:** PRD §5.9.2

#### 2.3.2: Flatten arrays into CSV columns
- **Input:** spectrogram_capture[64], novelty_curve_capture[1024], tempi_magnitude_capture[96], tempi_phase_capture[96]
- **Output:** Comma-separated values: "0.0234,0.0189,...,0.0012,0.045,0.051,..."
- **Ownership:** hil_export.h write_arrays_to_csv() function
- **User Sees:** Nothing
- **State Change:** CSV row appended to file
- **Integration:** File.print() loop over each array
- **Implements:** PRD §5.9.3 (array flattening)
- **Code:**
```cpp
void write_arrays_to_csv(File& f, hil_capture_state* state) {
    for (uint8_t i = 0; i < 64; i++) f.printf("%.4f,", state->spectrogram_capture[i]);
    for (uint16_t i = 0; i < 1024; i++) f.printf("%.4f,", state->novelty_curve_capture[i]);
    for (uint8_t i = 0; i < 96; i++) f.printf("%.4f,", state->tempi_magnitude_capture[i]);
    for (uint8_t i = 0; i < 96; i++) f.printf("%.4f,", state->tempi_phase_capture[i]);
    f.println();  // End row
}
```

#### 2.3.3: Increment frame counter and flush periodically
- **Input:** SL-6 frame_counter
- **Output:** frame_counter++, File.flush() every 100 frames
- **Ownership:** hil_export.h after CSV row write
- **User Sees:** Nothing
- **State Change:** SL-6 increments
- **Integration:** Called after write_arrays_to_csv()
- **Implements:** SL-6 update, file integrity (flush prevents corruption)
- **Code:**
```cpp
hil_export_state->frame_counter++;
if (hil_export_state->frame_counter % 100 == 0) {
    hil_export_state->csv_log_file.flush();
}
```

---

### Task 2.4: Implement LOG_STOP Command

**Pre-conditions:**
- CSV logging active (SL-2 file open, SL-6 > 0)

**Post-conditions:**
- CSV file flushed and closed
- SL-2 file handle cleared
- V-2 status broadcast with final stats

**Parent Task:** Handle LOG_STOP command to finalize CSV logging
- **Input:** "LOG_STOP" command string
- **Output:** CSV file closed, final statistics broadcast
- **Ownership:** commands.h handle_command() function
- **User Sees:** "LOG_COMPLETE|CSV|emotiscope_1234567890.csv|frames:15234|size_mb:42.3"
- **State Change:** SL-1=false, SL-2 closed, SL-6 logged for stats
- **Integration:** Called from wireless.h onMessage() callback
- **Implements:** PRD §5.9.5, Blueprint Phase 2 checkpoint 8, V-2 final status

**Subtasks:**

#### 2.4.1: Flush and close CSV file
- **Input:** SL-2 csv_log_file handle
- **Output:** File flushed, closed, handle invalidated
- **Ownership:** commands.h LOG_STOP handler
- **User Sees:** Nothing
- **State Change:** SL-2 file closed
- **Integration:** File.flush(), File.close()
- **Implements:** PRD §5.9.5, file integrity

#### 2.4.2: Calculate and broadcast final statistics
- **Input:** SL-6 frame_counter, file size in bytes
- **Output:** Broadcast: "LOG_COMPLETE|CSV|filename.csv|frames:15234|size_mb:42.3"
- **Ownership:** commands.h LOG_STOP handler end
- **User Sees:** WebSocket message with logging summary
- **State Change:** V-2 status updated to COMPLETE
- **Integration:** broadcast() with formatted stats
- **Implements:** V-2 user visibility, M-2 success metric (frames logged)

---

### Task 2.5: Add File Rotation at 10MB

**Pre-conditions:**
- CSV logging active
- File size exceeds 10MB (10,485,760 bytes)

**Post-conditions:**
- Current CSV file closed
- New CSV file opened with incremented sequence number
- Header written to new file
- Logging continues seamlessly

**Parent Task:** Implement CSV file rotation to prevent oversized files
- **Input:** File.size() check every 100 frames
- **Output:** Multiple CSV files: emotiscope_0001.csv, emotiscope_0002.csv, ...
- **Ownership:** hil_export.h check_and_rotate_csv() function
- **User Sees:** Broadcast: "LOG_ROTATED|CSV|emotiscope_0002.csv"
- **State Change:** SL-2 file handle replaced, new file opened
- **Integration:** Called during CSV row write if size check triggers
- **Implements:** PRD §5.9.6, Blueprint Phase 2 checkpoint 9

**Subtasks:**

#### 2.5.1: Check file size threshold
- **Input:** csv_log_file.size()
- **Output:** Boolean: size > 10MB
- **Ownership:** hil_export.h every 100 frames
- **User Sees:** Nothing
- **State Change:** None (check only)
- **Integration:** Called before CSV row write
- **Implements:** PRD §5.9.6

#### 2.5.2: Rotate to new file
- **Input:** Current filename, sequence counter
- **Output:** New file opened with incremented sequence: emotiscope_0002.csv
- **Ownership:** hil_export.h rotate_csv_file() function
- **User Sees:** "LOG_ROTATED|CSV|emotiscope_0002.csv"
- **State Change:** SL-2 file handle replaced
- **Integration:** Close old file, open new, write header
- **Implements:** PRD §5.9.6, O-1 (ESP32 manages files)

---

## PHASE 3: Binary Dump and JSON Export
*Duration: Medium Priority*
*Implements: PRD §5.10 (JSON), §5.11 (Binary), FR-6, FR-7, Blueprint §8 Phase 3*

### Task 3.1: Implement Binary File Format

**Pre-conditions:**
- hil_capture_state accessible
- SD card available

**Post-conditions:**
- Binary files created with magic header + float32 arrays
- Separate files per array type: spectrogram.bin, novelty.bin, tempi_magnitude.bin, tempi_phase.bin

**Parent Task:** Create binary export with structured headers
- **Input:** hil_capture_state arrays
- **Output:** .bin files with headers and little-endian float32 data
- **Ownership:** hil_export.h write_binary_files() function
- **User Sees:** Nothing (triggered by command)
- **State Change:** Binary files created on SD card
- **Integration:** Called by EXPORT_BINARY command handler
- **Implements:** PRD §5.11, FR-7, Blueprint Phase 3 checkpoint 1-3

**Subtasks:**

#### 3.1.1: Define binary file header struct
- **Input:** Blueprint §6.2 binary format specification
- **Output:** C++ struct with magic, array_length, element_size, timestamp
- **Ownership:** hil_export.h
- **User Sees:** Nothing
- **State Change:** None (definition)
- **Integration:** Used by binary write functions
- **Implements:** Blueprint §6.2 binary format, PRD §5.11.1
- **Code:**
```cpp
struct binary_file_header {
    uint32_t magic = 0x48494C01;  // "HIL" version 1
    uint16_t array_length;
    uint8_t element_size = 4;     // sizeof(float)
    uint8_t element_type = 1;     // 1=float32
    uint32_t timestamp_ms;
    uint32_t reserved = 0;
};
```

#### 3.1.2: Write binary files for each array type
- **Input:** spectrogram[64], novelty_curve[1024], tempi_magnitude[96], tempi_phase[96]
- **Output:** spectrogram_0001.bin, novelty_0001.bin, tempi_magnitude_0001.bin, tempi_phase_0001.bin
- **Ownership:** hil_export.h write_binary_array() helper
- **User Sees:** Nothing
- **State Change:** Binary files created
- **Integration:** Called by EXPORT_BINARY command
- **Implements:** PRD §5.11.2 (separate files), Blueprint Phase 3 checkpoint 2
- **Code:**
```cpp
void write_binary_array(const char* filename, float* data, uint16_t len) {
    File f = SD.open(filename, FILE_WRITE);
    binary_file_header hdr;
    hdr.array_length = len;
    hdr.timestamp_ms = millis();
    f.write((uint8_t*)&hdr, sizeof(hdr));
    f.write((uint8_t*)data, len * sizeof(float));
    f.close();
}
```

#### 3.1.3: Add binary file rotation
- **Input:** File sequence counter
- **Output:** Rotated files: spectrogram_0002.bin when 10MB exceeded
- **Ownership:** hil_export.h
- **User Sees:** Nothing
- **State Change:** Sequence counter incremented
- **Integration:** Check file size before write, rotate if needed
- **Implements:** PRD §5.11.4

---

### Task 3.2: Implement JSON Snapshot Export

**Pre-conditions:**
- hil_capture_state accessible
- ArduinoJSON library available

**Post-conditions:**
- JSON file created with complete metadata + algorithm parameters + signal snapshot
- JSON validated as parseable

**Parent Task:** Create JSON export with metadata and configuration
- **Input:** Hardware version, firmware version, algorithm parameters, configuration, signal data
- **Output:** JSON file: emotiscope_snapshot_1234567890.json
- **Ownership:** hil_export.h export_json_snapshot() function
- **User Sees:** Broadcast: "JSON_EXPORTED|emotiscope_snapshot_1234567890.json"
- **State Change:** JSON file created on SD card
- **Integration:** Called by EXPORT_JSON command handler
- **Implements:** PRD §5.10, FR-6, Blueprint Phase 3 checkpoint 5-8

**Subtasks:**

#### 3.2.1: Serialize metadata block
- **Input:** Hardware version, firmware version, sample rate, NUM_FREQS, NUM_TEMPI
- **Output:** JSON object: {"metadata": {"timestamp_ms": ..., "hardware_version": "v1.1", ...}}
- **Ownership:** hil_export.h using ArduinoJSON
- **User Sees:** Nothing
- **State Change:** None (formatting)
- **Integration:** First block in JSON structure
- **Implements:** PRD §5.10.1
- **Code:**
```cpp
DynamicJsonDocument doc(32768);
JsonObject meta = doc.createNestedObject("metadata");
meta["timestamp_ms"] = millis();
meta["hardware_version"] = "v1.1";
meta["firmware_version"] = "1.1.0-hil";
meta["sample_rate_hz"] = 12800;
meta["num_freqs"] = NUM_FREQS;
```

#### 3.2.2: Serialize algorithm parameters
- **Input:** Goertzel block_sizes[64], coeff[64], tempo frequencies[96], BEAT_SHIFT_PERCENT
- **Output:** JSON object: {"algorithm_params": {"goertzel_block_sizes": [...], ...}}
- **Ownership:** hil_export.h
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Second block in JSON
- **Implements:** PRD §5.10.2

#### 3.2.3: Serialize configuration
- **Input:** current_mode, brightness, softness, color, warmth settings
- **Output:** JSON object: {"configuration": {"current_mode": "Spectrum", ...}}
- **Ownership:** hil_export.h
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Third block in JSON
- **Implements:** PRD §5.10.3

#### 3.2.4: Optionally include signal snapshot
- **Input:** Last 1-10 seconds of spectrogram data (circular buffer)
- **Output:** JSON array: {"signal_snapshot": {"spectrogram": [[...], [...], ...]}}
- **Ownership:** hil_export.h
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Fourth block in JSON (optional, flag-controlled)
- **Implements:** PRD §5.10.4

#### 3.2.5: Write and validate JSON file
- **Input:** Serialized JSON document
- **Output:** JSON file on SD card, validation check
- **Ownership:** hil_export.h
- **User Sees:** Error if JSON invalid: "JSON_ERROR|Invalid JSON structure"
- **State Change:** JSON file created
- **Integration:** serializeJson(doc, file), validate with deserializeJson()
- **Implements:** PRD §5.10.5, §5.10.6

---

## PHASE 4: WebSocket Binary Frames and Real-Time Streaming
*Duration: Medium Priority*
*Implements: PRD §5.12, FR-8, Blueprint §8 Phase 4*

### Task 4.1: Extend WebSocket for Binary Frames

**Pre-conditions:**
- PsychicHttp WebSocket infrastructure exists in wireless.h
- broadcast() function supports text frames

**Post-conditions:**
- broadcast() overloaded to support binary frames
- Binary frame format defined: type byte + length + payload

**Parent Task:** Add binary frame support to WebSocket broadcast
- **Input:** Binary data pointer, length, frame type
- **Output:** Binary WebSocket frame sent to clients
- **Ownership:** wireless.h broadcast() overload
- **User Sees:** Nothing (internal protocol)
- **State Change:** None
- **Integration:** Called by array streaming logic
- **Implements:** PRD §5.12.1, §5.12.2, Blueprint Phase 4 checkpoint 1-2

**Subtasks:**

#### 4.1.1: Define binary frame structure
- **Input:** Blueprint §7.2 WebSocket binary format spec
- **Output:** Frame structure: [type:uint8_t][length:uint16_t][payload:bytes]
- **Ownership:** wireless.h
- **User Sees:** Nothing
- **State Change:** None (definition)
- **Integration:** Used by binary broadcast
- **Implements:** PRD §5.12.2

#### 4.1.2: Implement broadcast_binary() function
- **Input:** Binary data pointer, length, type (spectrogram=1, novelty=2, tempi=3)
- **Output:** Binary frame sent to all connected clients
- **Ownership:** wireless.h
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Called by HIL monitoring code
- **Implements:** PRD §5.12.1, Blueprint Phase 4 checkpoint 1
- **Code:**
```cpp
void broadcast_binary(uint8_t type, const void* data, uint16_t length) {
    uint8_t frame[3 + length];
    frame[0] = type;
    frame[1] = (length >> 8) & 0xFF;
    frame[2] = length & 0xFF;
    memcpy(&frame[3], data, length);
    // PsychicHttp sendBinary to all clients
    for (auto client : ws_clients) {
        client->binary(frame, sizeof(frame));
    }
}
```

---

### Task 4.2: Implement Per-Client Subscription System

**Pre-conditions:**
- Binary broadcast available (Task 4.1 complete)
- Maximum 4 WebSocket clients supported

**Post-conditions:**
- SL-4: client_subscription_masks[4] tracks per-client array subscriptions
- SUBSCRIBE command handler added
- Only subscribed clients receive binary frames

**Parent Task:** Add subscription tracking to prevent flooding all clients
- **Input:** "SUBSCRIBE:spectrogram" command from client
- **Output:** SL-4 bitmask updated for that client
- **Ownership:** commands.h SUBSCRIBE handler, wireless.h subscription tracking
- **User Sees:** Broadcast confirmation: "SUBSCRIBED|client_0|spectrogram"
- **State Change:** SL-4 bitmask set for that client/array
- **Integration:** Check bitmask before sending binary frames
- **Implements:** PRD §5.12.4, SL-4 state variable, Blueprint Phase 4 checkpoint 3-4

**Subtasks:**

#### 4.2.1: Add client subscription bitmasks
- **Input:** Client slot (0-3), array type bitmask (bit 0=spectrogram, bit 1=novelty, bit 2=tempi_mag, bit 3=tempi_phase)
- **Output:** SL-4 client_subscription_masks[4] array
- **Ownership:** hil_export.h
- **User Sees:** Nothing
- **State Change:** SL-4 initialized to 0 (no subscriptions)
- **Integration:** Checked before binary broadcast
- **Implements:** SL-4 state variable, PRD §5.12.4

#### 4.2.2: Implement SUBSCRIBE command handler
- **Input:** "SUBSCRIBE:spectrogram" or "SUBSCRIBE:novelty"
- **Output:** Set corresponding bit in client_subscription_masks[client_slot]
- **Ownership:** commands.h
- **User Sees:** "SUBSCRIBED|client_0|spectrogram"
- **State Change:** SL-4 bit set for that client
- **Integration:** Called from wireless.h onMessage()
- **Implements:** PRD §5.12.4

#### 4.2.3: Filter binary sends by subscription
- **Input:** Array type, client subscription masks
- **Output:** Binary frame sent only to subscribed clients
- **Ownership:** wireless.h broadcast_binary() modified
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Check bitmask before client->binary() call
- **Implements:** PRD §5.12.4, bandwidth optimization

---

### Task 4.3: Implement Real-Time Array Streaming

**Pre-conditions:**
- Binary broadcast and subscriptions ready (Tasks 4.1-4.2 complete)
- hil_capture_state populated every frame

**Post-conditions:**
- Binary frames sent for subscribed arrays every frame or decimated to 30 FPS
- Backpressure handled: drop frames if WebSocket buffer full

**Parent Task:** Stream signal arrays to subscribed WebSocket clients
- **Input:** hil_capture_state arrays, subscription masks
- **Output:** Binary frames sent at controlled rate
- **Ownership:** cpu_core.h or dedicated streaming task
- **User Sees:** Real-time signal data in browser/dashboard
- **State Change:** None (streaming only)
- **Integration:** Called at end of audio frame or decimated
- **Implements:** PRD §5.12.3, §5.12.6, Blueprint Phase 4 checkpoint 5-6

**Subtasks:**

#### 4.3.1: Decimate streaming to 30 FPS
- **Input:** Audio frame rate ~200 FPS
- **Output:** Binary frames sent every ~7th frame (200/30 ≈ 6.67)
- **Ownership:** hil_export.h or cpu_core.h
- **User Sees:** Smoother dashboard updates
- **State Change:** None
- **Integration:** Frame counter % 7 == 0 trigger
- **Implements:** PRD §5.12.3 (30 FPS throttle)

#### 4.3.2: Handle WebSocket backpressure
- **Input:** WebSocket send buffer status
- **Output:** Drop frames if buffer full (non-blocking send)
- **Ownership:** wireless.h broadcast_binary()
- **User Sees:** Nothing (silent drop)
- **State Change:** None
- **Integration:** Check buffer before send, skip if full
- **Implements:** PRD §5.12.6, E-2 boundary rule (don't block on WebSocket)

#### 4.3.3: Continue text frames for timing
- **Input:** DSP timing metrics (DSP_ACQUIRE_US, etc.)
- **Output:** Text frames continue as before: "HIL|DSP_ACQUIRE_US|287"
- **Ownership:** cpu_core.h existing timing logs
- **User Sees:** Timing metrics in WebSocket console
- **State Change:** None
- **Integration:** Existing HILMonitor::log() calls unchanged
- **Implements:** PRD §5.12.5 (text+binary coexistence)

---

## PHASE 5: Serial Logging and SD Card Fallback
*Duration: High Priority (robustness)*
*Implements: PRD §5.13, FR-9, Blueprint §8 Phase 5*

### Task 5.1: Implement Serial Logging Mode

**Pre-conditions:**
- CSV formatting functions available (Phase 2)
- Serial configured at 2Mbaud

**Post-conditions:**
- SL-10: serial_logging_active flag functional
- CSV rows stream to Serial when enabled
- LOG_START:SERIAL command handler added

**Parent Task:** Add serial logging as alternative to SD card
- **Input:** "LOG_START:SERIAL" command
- **Output:** CSV rows stream to Serial at 2Mbaud
- **Ownership:** commands.h LOG_START:SERIAL handler, hil_export.h serial write
- **User Sees:** CSV rows on serial console (Arduino IDE serial monitor or screen/minicom)
- **State Change:** SL-10=true, logging to serial instead of SD
- **Integration:** Alternative output path in CSV export logic
- **Implements:** PRD §5.13.1, §5.13.3, FR-9, Blueprint Phase 5 checkpoint 1-2

**Subtasks:**

#### 5.1.1: Add LOG_START:SERIAL command handler
- **Input:** "LOG_START:SERIAL" command string
- **Output:** SL-10=true, serial logging begins
- **Ownership:** commands.h
- **User Sees:** Broadcast: "LOG_STATUS|ACTIVE|SERIAL"
- **State Change:** SL-10=true
- **Integration:** Called from wireless.h or serial command parser
- **Implements:** PRD §5.13.1

#### 5.1.2: Stream CSV rows to Serial
- **Input:** Formatted CSV row string
- **Output:** Serial.println(csv_row)
- **Ownership:** hil_export.h write_csv_row() modified
- **User Sees:** CSV rows scrolling in serial console
- **State Change:** None (output only)
- **Integration:** if (SL-10) Serial.println(row); else file.println(row);
- **Implements:** PRD §5.13.3

#### 5.1.3: Add periodic Serial.flush()
- **Input:** Frame counter
- **Output:** Serial.flush() every 100 frames
- **Ownership:** hil_export.h
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Called after Serial.println()
- **Implements:** O-7 boundary rule (prevent serial buffer overrun)

---

### Task 5.2: Implement SD Card Error Detection and Fallback

**Pre-conditions:**
- SD card write functions return error codes
- Serial logging mode available (Task 5.1)

**Post-conditions:**
- SL-9: sd_card_error flag set on write failure
- Automatic fallback to serial logging when SD fails
- V-7: User notified of error and fallback

**Parent Task:** Detect SD write errors and fall back to serial
- **Input:** File.write() return value == 0 (error)
- **Output:** SL-9=true, SL-10=true (switch to serial), V-7 error broadcast
- **Ownership:** hil_export.h SD write wrapper
- **User Sees:** "SD_CARD_ERROR|FULL_OR_FAILED - switching to serial logging"
- **State Change:** SL-9=true, SL-10=true, CSV file closed
- **Integration:** Wrapped around File.write() calls
- **Implements:** E-1, T-7, V-7, SL-9, SL-10, Blueprint Phase 5 checkpoint 3-5

**Subtasks:**

#### 5.2.1: Wrap File.write() with error checking
- **Input:** File.write(data, len) return value
- **Output:** Boolean success/fail
- **Ownership:** hil_export.h safe_file_write() wrapper
- **User Sees:** Nothing (internal)
- **State Change:** SL-9=true if write fails
- **Integration:** Replace all File.write() calls
- **Implements:** E-1 (SD error detection)
- **Code:**
```cpp
bool safe_file_write(File& f, const void* data, size_t len) {
    size_t written = f.write((const uint8_t*)data, len);
    if (written != len) {
        hil_export_state->sd_card_error = true;  // SL-9
        return false;
    }
    return true;
}
```

#### 5.2.2: Trigger fallback to serial on SD error
- **Input:** SL-9=true (SD error detected)
- **Output:** SL-10=true (enable serial logging), close CSV file
- **Ownership:** hil_export.h SD error handler
- **User Sees:** "SD_CARD_ERROR|Switching to serial logging"
- **State Change:** SL-9=true, SL-10=true, SL-2 file closed
- **Integration:** Called when safe_file_write() returns false
- **Implements:** T-7 (fallback), V-7 (error notification)
- **Code:**
```cpp
if (!safe_file_write(csv_log_file, data, len)) {
    csv_log_file.close();  // SL-2 cleared
    hil_export_state->serial_logging_active = true;  // SL-10
    broadcast("SD_CARD_ERROR|Switching to serial logging");
}
```

---

### Task 5.3: Implement LOG_STATUS Command

**Pre-conditions:**
- Logging state variables accessible (SL-2, SL-6, SL-9, SL-10)

**Post-conditions:**
- LOG_STATUS command returns current logging state
- Includes: mode, filename, frames logged, file size, errors

**Parent Task:** Add LOG_STATUS command for monitoring state
- **Input:** "LOG_STATUS" command string
- **Output:** Broadcast current logging status
- **Ownership:** commands.h LOG_STATUS handler
- **User Sees:** "LOG_STATUS|ACTIVE|CSV|emotiscope_1234.csv|frames:5432|size_mb:12.3|errors:0"
- **State Change:** None (read-only query)
- **Integration:** Called from WebSocket or serial commands
- **Implements:** PRD §5.13.6, V-2 (user visibility), Blueprint Phase 5 checkpoint 7

**Subtasks:**

#### 5.3.1: Query current logging state
- **Input:** SL-2, SL-6, SL-9, SL-10
- **Output:** Status string with all relevant info
- **Ownership:** commands.h LOG_STATUS handler
- **User Sees:** Complete status broadcast
- **State Change:** None
- **Integration:** Broadcast formatted status
- **Implements:** PRD §5.13.6

---

## PHASE 6: Visual Pipeline Instrumentation
*Duration: Medium Priority*
*Implements: PRD §5.6-§5.8, FR-3, FR-4, Blueprint §8 Phase 6*

### Task 6.1: Instrument Light Mode Draw Functions

**Pre-conditions:**
- light_modes[] array defined in light_modes.h
- profile_function() template available from profiler.h

**Post-conditions:**
- All 12+ light mode draw() functions wrapped with profile_function()
- Per-mode timing logged every 250ms

**Parent Task:** Wrap all light mode draw() calls with profiling
- **Input:** light_modes[current_mode].draw function pointer
- **Output:** Timing logged: "HIL_STAT|draw_spectrum|892|250" (avg_us, hits)
- **Ownership:** gpu_core.h line 47 (light_modes[].draw() call site)
- **User Sees:** WebSocket broadcast with per-mode timing
- **State Change:** profiler_functions[] updated with mode timing
- **Integration:** Replace bare draw() call with profile_function() wrapper
- **Implements:** PRD §5.6.1, §5.6.2, §5.6.3, Blueprint Phase 6 checkpoint 1-3

**Subtasks:**

#### 6.1.1: Wrap light_modes[].draw() with profile_function()
- **Input:** light_modes[configuration.current_mode].draw
- **Output:** Timing automatically captured by profiler.h
- **Ownership:** gpu_core.h line 47
- **User Sees:** Nothing (silent capture)
- **State Change:** profiler_functions[] entry created for each mode
- **Integration:** profile_function([&](){ light_modes[...].draw(); }, mode_name);
- **Implements:** PRD §5.6.1
- **Code:**
```cpp
// Replace existing:
// light_modes[configuration.current_mode].draw();
// With:
profile_function([&]() {
    light_modes[configuration.current_mode].draw();
}, light_modes[configuration.current_mode].name);
```

#### 6.1.2: Broadcast per-mode timing statistics
- **Input:** profiler_functions[] with mode names
- **Output:** HIL_STAT messages every 250ms
- **Ownership:** hil_monitor.h broadcastStats() (already exists)
- **User Sees:** "HIL_STAT|draw_spectrum|892|250"
- **State Change:** None (broadcast only)
- **Integration:** Existing broadcastStats() call handles this automatically
- **Implements:** PRD §5.6.2, §5.6.4

---

### Task 6.2: Instrument Post-Processing Pipeline

**Pre-conditions:**
- Post-processing functions defined: apply_image_lpf(), apply_tonemapping(), apply_warmth(), apply_gamma_correction()
- leds[] CRGBF array accessible

**Post-conditions:**
- Timing logged for each post-processing stage
- Optional: leds[] buffer captured after each stage (flag-controlled due to memory)

**Parent Task:** Add timing instrumentation to post-processing
- **Input:** Post-processing function calls in gpu_core.h
- **Output:** Timing metrics: GPU_LPF_US, TONEMAPPING_US, WARMTH_US, GAMMA_US
- **Ownership:** gpu_core.h post-processing call sites
- **User Sees:** WebSocket broadcast with post-processing timing
- **State Change:** Timing metrics updated
- **Integration:** Wrap each post-processing function with timing capture
- **Implements:** PRD §5.7.5, Blueprint Phase 6 checkpoint 6

**Subtasks:**

#### 6.2.1: Add timing for apply_image_lpf()
- **Input:** ESP.getCycleCount() before/after apply_image_lpf()
- **Output:** GPU_LPF_US logged
- **Ownership:** gpu_core.h
- **User Sees:** "HIL|GPU_LPF_US|156"
- **State Change:** Timing updated
- **Integration:** Surround apply_image_lpf() with timing code
- **Implements:** PRD §5.7.5
- **Code:**
```cpp
uint32_t t_lpf = micros();
apply_image_lpf();
HILMonitor::log("GPU_LPF_US", (int)(micros() - t_lpf));
```

#### 6.2.2: Add timing for other post-processing stages
- **Input:** apply_tonemapping(), apply_warmth(), apply_gamma_correction()
- **Output:** TONEMAPPING_US, WARMTH_US, GAMMA_US logged
- **Ownership:** gpu_core.h
- **User Sees:** WebSocket broadcast for each stage
- **State Change:** Timing metrics updated
- **Integration:** Same pattern as LPF
- **Implements:** PRD §5.7.5

#### 6.2.3: Optional: Capture leds[] after each stage
- **Input:** leds[NUM_LEDS] CRGBF array
- **Output:** Snapshot buffers (if flag enabled): leds_post_lpf[], leds_post_tonemap[], etc.
- **Ownership:** hil_capture.h (requires large memory allocation)
- **User Sees:** Nothing
- **State Change:** Optional capture buffers populated
- **Integration:** Conditional memcpy if CAPTURE_LED_BUFFERS defined
- **Implements:** PRD §5.7.1-§5.7.4
- **Note:** Large memory footprint (NUM_LEDS * sizeof(CRGBF) * 4 stages), make conditional

---

### Task 6.3: Instrument LED Driver Transmission

**Pre-conditions:**
- transmit_leds() function defined in led_driver.h

**Post-conditions:**
- LED_TRANSMIT_US timing logged
- LED_QUANTIZE_US timing logged (if quantization separate)

**Parent Task:** Add timing for LED transmission and quantization
- **Input:** transmit_leds() call, quantization step
- **Output:** LED_TRANSMIT_US, LED_QUANTIZE_US metrics
- **Ownership:** led_driver.h or gpu_core.h
- **User Sees:** WebSocket broadcast with LED timing
- **State Change:** Timing updated
- **Integration:** Wrap transmit_leds() with timing
- **Implements:** PRD §5.8.1, §5.8.2, Blueprint Phase 6 checkpoint 7-8

**Subtasks:**

#### 6.3.1: Add timing for transmit_leds()
- **Input:** ESP.getCycleCount() before/after transmit_leds()
- **Output:** LED_TRANSMIT_US logged
- **Ownership:** gpu_core.h or led_driver.h
- **User Sees:** "HIL|LED_TRANSMIT_US|234"
- **State Change:** Timing updated
- **Integration:** Surround transmit_leds() with timing
- **Implements:** PRD §5.8.1

#### 6.3.2: Add timing for quantization/dithering
- **Input:** Float-to-8bit conversion step
- **Output:** LED_QUANTIZE_US logged
- **Ownership:** led_driver.h quantize_and_dither() or similar
- **User Sees:** "HIL|LED_QUANTIZE_US|45"
- **State Change:** Timing updated
- **Integration:** Wrap quantization step with timing
- **Implements:** PRD §5.8.2

---

## PHASE 7: Dashboard UI (Optional / Secondary Priority)
*Duration: Low Priority (Nice-to-Have)*
*Implements: PRD §5.16-§5.18, FR-12, Blueprint §8 Phase 7*

### Task 7.1: Create Dashboard HTML Page

**Pre-conditions:**
- PsychicHttp web server running
- WebSocket binary frames available (Phase 4)

**Post-conditions:**
- Dashboard HTML/CSS/JS embedded in firmware flash
- /dashboard endpoint serves dashboard
- Dashboard loads in browser

**Parent Task:** Implement web-based dashboard for monitoring
- **Input:** Embedded HTML/CSS/JS files
- **Output:** Dashboard accessible at http://emotiscope.local/dashboard
- **Ownership:** web_core.h endpoint registration, dashboard files in data/ folder
- **User Sees:** Full dashboard UI in browser
- **State Change:** None (read-only dashboard)
- **Integration:** Served by PsychicHttp
- **Implements:** PRD §5.16.1, Blueprint Phase 7 checkpoint 1-3, O-4 boundary rule

**Subtasks:**

#### 7.1.1: Create dashboard HTML structure
- **Input:** PRD §5.16-§5.18 UI requirements
- **Output:** dashboard.html with sections for timing, plots, controls
- **Ownership:** data/dashboard.html (embedded at compile time)
- **User Sees:** Dashboard layout in browser
- **State Change:** None
- **Integration:** Embedded via PROGMEM or LittleFS
- **Implements:** PRD §5.16.1

#### 7.1.2: Add Chart.js library for plots
- **Input:** Chart.js CDN or embedded library
- **Output:** JavaScript charts for spectrogram, novelty, tempi
- **Ownership:** dashboard.html <script> tags
- **User Sees:** Live updating charts
- **State Change:** None
- **Integration:** WebSocket binary frames feed Chart.js
- **Implements:** PRD §5.17.2-§5.17.4

#### 7.1.3: Register /dashboard endpoint
- **Input:** PsychicHttp route registration
- **Output:** Endpoint serves dashboard.html
- **Ownership:** web_core.h
- **User Sees:** Dashboard loads at /dashboard URL
- **State Change:** None
- **Integration:** server.on("/dashboard", handleDashboard);
- **Implements:** PRD §5.16.1, O-4

---

### Task 7.2: Implement Timing Breakdown Table

**Pre-conditions:**
- Dashboard HTML loaded
- WebSocket text frames with timing data available

**Post-conditions:**
- Timing table displays DSP_ACQUIRE_US, DSP_MAGNITUDES_US, etc.
- Table updates every 250ms via WebSocket
- Historical min/max/avg calculated client-side

**Parent Task:** Display timing breakdown in dashboard table
- **Input:** HIL|DSP_*_US WebSocket text frames
- **Output:** HTML table with timing values
- **Ownership:** dashboard.html JavaScript
- **User Sees:** Live timing table in browser
- **State Change:** None (client-side only)
- **Integration:** Parse WebSocket text frames, update table
- **Implements:** PRD §5.16.2-§5.16.4, Blueprint Phase 7 checkpoint 4-7

**Subtasks:**

#### 7.2.1: Create timing table HTML
- **Input:** Timing metric names from PRD
- **Output:** HTML table with rows for each metric
- **Ownership:** dashboard.html
- **User Sees:** Table with columns: Metric, Current, Min, Max, Avg
- **State Change:** None
- **Integration:** Static HTML structure
- **Implements:** PRD §5.16.2

#### 7.2.2: Update table from WebSocket
- **Input:** "HIL|DSP_ACQUIRE_US|287" text frames
- **Output:** Table cells updated with current values
- **Ownership:** dashboard.html WebSocket onMessage handler
- **User Sees:** Live updating table values
- **State Change:** None
- **Integration:** Parse pipe-delimited messages, update table cells
- **Implements:** PRD §5.16.3

#### 7.2.3: Calculate historical min/max/avg
- **Input:** Circular buffer of last N timing values (client-side)
- **Output:** Min/Max/Avg columns updated
- **Ownership:** dashboard.html JavaScript
- **User Sees:** Statistical summary of timing
- **State Change:** None
- **Integration:** Track values in array, compute stats every update
- **Implements:** PRD §5.16.4

---

### Task 7.3: Implement Signal Visualization Plots

**Pre-conditions:**
- Dashboard loaded with Chart.js
- WebSocket binary frames subscribed (Phase 4)

**Post-conditions:**
- Spectrogram bar chart (64 bins) updates in real-time
- Novelty curve scrolling line chart (1024 samples)
- Tempi magnitude bar chart (96 bins)
- Plots throttled to 30 FPS client-side

**Parent Task:** Display live signal plots in dashboard
- **Input:** WebSocket binary frames for spectrogram, novelty, tempi
- **Output:** Chart.js plots rendering signal data
- **Ownership:** dashboard.html JavaScript + Chart.js
- **User Sees:** Live signal visualizations
- **State Change:** None (client-side rendering)
- **Integration:** Parse binary frames, update Chart.js datasets
- **Implements:** PRD §5.17.2-§5.17.6, Blueprint Phase 7 checkpoint 8-11

**Subtasks:**

#### 7.3.1: Create spectrogram bar chart
- **Input:** WebSocket binary frame type=1 (spectrogram[64])
- **Output:** Chart.js bar chart with 64 bars
- **Ownership:** dashboard.html
- **User Sees:** Live frequency spectrum bars
- **State Change:** None
- **Integration:** Parse binary frame, update chart.data.datasets[0].data
- **Implements:** PRD §5.17.2

#### 7.3.2: Create novelty curve scrolling line chart
- **Input:** WebSocket binary frame type=2 (novelty_curve[1024])
- **Output:** Chart.js line chart scrolling left
- **Ownership:** dashboard.html
- **User Sees:** Live spectral flux waveform
- **State Change:** None
- **Integration:** Parse binary frame, shift array, update chart
- **Implements:** PRD §5.17.3

#### 7.3.3: Create tempi magnitude bar chart
- **Input:** WebSocket binary frame type=3 (tempi_magnitude[96])
- **Output:** Chart.js bar chart with 96 bars (BPM range)
- **Ownership:** dashboard.html
- **User Sees:** Live tempo detection bars
- **State Change:** None
- **Integration:** Parse binary frame, update chart
- **Implements:** PRD §5.17.4

#### 7.3.4: Throttle plot updates to 30 FPS
- **Input:** Binary frames arriving at 200 FPS (decimated to 30 FPS by Phase 4)
- **Output:** Chart.js update() called max 30 times/second
- **Ownership:** dashboard.html requestAnimationFrame() loop
- **User Sees:** Smooth plot updates without jank
- **State Change:** None
- **Integration:** Debounce chart updates
- **Implements:** PRD §5.17.6

---

### Task 7.4: Implement Logging Control Buttons

**Pre-conditions:**
- Dashboard loaded
- LOG_START/LOG_STOP commands available (Phase 2, Phase 5)

**Post-conditions:**
- "Start CSV Log" and "Stop Log" buttons functional
- Array subscription checkboxes control WebSocket binary streams
- Logging status displayed (frames, file size)

**Parent Task:** Add user controls to dashboard
- **Input:** Button clicks, checkbox changes
- **Output:** Commands sent via WebSocket
- **Ownership:** dashboard.html JavaScript
- **User Sees:** Interactive control panel
- **State Change:** Commands trigger backend logging state changes
- **Integration:** Send WebSocket text commands on button click
- **Implements:** PRD §5.18.1-§5.18.5, Blueprint Phase 7 checkpoint 12-14

**Subtasks:**

#### 7.4.1: Add Start/Stop CSV Log buttons
- **Input:** Button click events
- **Output:** "LOG_START:CSV" or "LOG_STOP" commands sent via WebSocket
- **Ownership:** dashboard.html
- **User Sees:** Buttons to control logging
- **State Change:** Backend logging starts/stops
- **Integration:** ws.send("LOG_START:CSV") on click
- **Implements:** PRD §5.18.1

#### 7.4.2: Add array subscription checkboxes
- **Input:** Checkbox change events for spectrogram, novelty, tempi
- **Output:** "SUBSCRIBE:spectrogram" commands sent
- **Ownership:** dashboard.html
- **User Sees:** Checkboxes to control binary streams
- **State Change:** Backend SL-4 subscription masks updated
- **Integration:** ws.send("SUBSCRIBE:"+array_name) on change
- **Implements:** PRD §5.18.2

#### 7.4.3: Display logging status
- **Input:** LOG_STATUS responses from backend
- **Output:** Status panel showing frames logged, file size
- **Ownership:** dashboard.html
- **User Sees:** "Logging: ACTIVE | CSV | 5432 frames | 12.3 MB"
- **State Change:** None (display only)
- **Integration:** Parse LOG_STATUS messages, update status div
- **Implements:** PRD §5.18.4

---

## PHASE 8: Benchmarking and Python Analysis Tools
*Duration: Medium Priority*
*Implements: PRD §5.14-§5.15, M-5, Blueprint §8 Phase 8*

### Task 8.1: Create Test Signal Library

**Pre-conditions:**
- None (external assets)

**Post-conditions:**
- Test signals available: sine sweeps, white noise, percussive transients, music samples
- Signals stored in standardized format (WAV, 12.8kHz if possible)

**Parent Task:** Prepare test signals for benchmarking
- **Input:** Audio generation tools (Audacity, Python soundfile)
- **Output:** Test signal WAV files in test_signals/ directory
- **Ownership:** External (not embedded firmware)
- **User Sees:** Nothing (offline preparation)
- **State Change:** None
- **Integration:** Used during benchmarking procedures
- **Implements:** PRD §5.14.2, Blueprint Phase 8 checkpoint 1

**Subtasks:**

#### 8.1.1: Generate sine sweep (20Hz - 6.4kHz)
- **Input:** Audacity or Python chirp generation
- **Output:** sine_sweep_20hz_6400hz.wav at 12.8kHz sample rate
- **Ownership:** test_signals/ directory
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Played during Goertzel benchmarking
- **Implements:** PRD §5.14.2

#### 8.1.2: Generate white noise
- **Input:** Python np.random.randn()
- **Output:** white_noise_10s.wav at 12.8kHz
- **Ownership:** test_signals/
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Tests noise floor and auto-ranging
- **Implements:** PRD §5.14.2

#### 8.1.3: Generate percussive transients
- **Input:** Drum samples or synthesized clicks
- **Output:** percussive_transients.wav at 12.8kHz
- **Ownership:** test_signals/
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Tests spectral flux and tempo detection
- **Implements:** PRD §5.14.2

#### 8.1.4: Select music samples
- **Input:** Royalty-free music clips (various genres, BPMs)
- **Output:** music_sample_*.wav at 12.8kHz
- **Ownership:** test_signals/
- **User Sees:** Nothing
- **State Change:** None
- **Integration:** Real-world algorithm validation
- **Implements:** PRD §5.14.2

---

### Task 8.2: Document Synchronized Playback Procedure

**Pre-conditions:**
- Test signals available (Task 8.1)
- Multiple devices to benchmark: Emotiscope v1.1, Lightwave-Ledstrip, Tab5.DSP

**Post-conditions:**
- Procedure documented for timestamp-aligned multi-device capture
- Manual trigger method specified (audio splitter setup)

**Parent Task:** Create benchmarking procedure documentation
- **Input:** PRD §5.14 benchmarking requirements
- **Output:** BENCHMARKING.md document
- **Ownership:** planning/ directory
- **User Sees:** Documentation file
- **State Change:** None
- **Integration:** Reference for running benchmarks
- **Implements:** PRD §5.14.1, §5.14.4, Blueprint Phase 8 checkpoint 2

**Subtasks:**

#### 8.2.1: Document audio splitter setup
- **Input:** Audio source (phone/laptop) → splitter → N devices
- **Output:** Diagram and instructions in BENCHMARKING.md
- **Ownership:** planning/BENCHMARKING.md (new file)
- **User Sees:** Setup instructions
- **State Change:** None
- **Integration:** None
- **Implements:** PRD §5.14.1

#### 8.2.2: Document manual trigger procedure
- **Input:** Synchronized button press or audio cue
- **Output:** Procedure: "1. Start LOG on all devices, 2. Press sync button, 3. Play test signal"
- **Ownership:** BENCHMARKING.md
- **User Sees:** Step-by-step instructions
- **State Change:** None
- **Integration:** None
- **Implements:** PRD §5.14.4

#### 8.2.3: Document timestamp alignment strategy
- **Input:** CSV exports with relative timestamps (millis())
- **Output:** Procedure for aligning timestamps in Python post-processing
- **Ownership:** BENCHMARKING.md
- **User Sees:** Alignment methodology
- **State Change:** None
- **Integration:** Used by Python analysis script (Task 8.3)
- **Implements:** M-5, PRD §5.14.5

---

### Task 8.3: Create Python Analysis Script

**Pre-conditions:**
- CSV exports available from v1.1, Lightwave-Ledstrip, Tab5.DSP
- Python 3.8+ with pandas, numpy, matplotlib

**Post-conditions:**
- analyze_hil_comparison.py script loads and aligns CSV exports
- Generates comparison plots: spectrogram diff, tempo accuracy, timing deltas
- Calculates metrics: RMS difference, beat detection agreement

**Parent Task:** Implement Python script for multi-device comparison
- **Input:** CSV files from all devices
- **Output:** Comparison plots and metrics report
- **Ownership:** tools/analyze_hil_comparison.py (new file)
- **User Sees:** Plots and metrics showing algorithm differences
- **State Change:** None
- **Integration:** Run offline after benchmarking captures
- **Implements:** PRD §5.15, M-5, Blueprint Phase 8 checkpoint 4-11

**Subtasks:**

#### 8.3.1: Load CSV exports with pandas
- **Input:** CSV files from v1.1, Lightwave, Tab5
- **Output:** pandas DataFrames with all signals
- **Ownership:** tools/analyze_hil_comparison.py load_csv() function
- **User Sees:** Nothing (internal)
- **State Change:** None
- **Integration:** Called at script start
- **Implements:** PRD §5.15.1

#### 8.3.2: Implement timestamp alignment
- **Input:** Relative timestamps (millis()) from each device
- **Output:** Aligned DataFrames with common time axis
- **Ownership:** analyze_hil_comparison.py align_timestamps() function
- **User Sees:** Nothing (internal)
- **State Change:** None
- **Integration:** Called after load_csv()
- **Implements:** M-5, PRD §5.15.1
- **Code:**
```python
def align_timestamps(df1, df2, sync_event_threshold=0.5):
    # Find sync event (e.g., first large amplitude)
    sync_idx1 = df1[df1['vu_level'] > sync_event_threshold].index[0]
    sync_idx2 = df2[df2['vu_level'] > sync_event_threshold].index[0]
    # Align timestamps
    offset = df2.loc[sync_idx2, 'timestamp_ms'] - df1.loc[sync_idx1, 'timestamp_ms']
    df2['timestamp_ms'] -= offset
    return df1, df2
```

#### 8.3.3: Generate spectrogram comparison plot
- **Input:** Aligned spectrogram[64] arrays from both devices
- **Output:** Matplotlib heatmap showing difference
- **Ownership:** analyze_hil_comparison.py plot_spectrogram_diff()
- **User Sees:** Plot: spectrogram_comparison.png
- **State Change:** None
- **Integration:** Called for each comparison
- **Implements:** PRD §5.15.2

#### 8.3.4: Generate tempo detection comparison
- **Input:** tempi_magnitude[96], tempi_phase[96] from both devices
- **Output:** Plot showing phase alignment and beat detection agreement
- **Ownership:** analyze_hil_comparison.py plot_tempo_comparison()
- **User Sees:** Plot: tempo_comparison.png
- **State Change:** None
- **Integration:** Called for each comparison
- **Implements:** PRD §5.15.3

#### 8.3.5: Generate timing comparison plot
- **Input:** DSP_*_US timing metrics from both devices
- **Output:** Bar chart comparing latencies
- **Ownership:** analyze_hil_comparison.py plot_timing_comparison()
- **User Sees:** Plot: timing_comparison.png
- **State Change:** None
- **Integration:** Called for each comparison
- **Implements:** PRD §5.15.4

#### 8.3.6: Calculate RMS difference metric
- **Input:** Aligned spectrogram arrays
- **Output:** RMS difference: sqrt(mean((spec1 - spec2)^2))
- **Ownership:** analyze_hil_comparison.py calculate_rms_diff()
- **User Sees:** Metric printed: "RMS Difference: 0.023"
- **State Change:** None
- **Integration:** Called in metrics summary
- **Implements:** PRD §5.15.5

#### 8.3.7: Calculate beat detection agreement
- **Input:** tempi_beat[96] sin(phase) outputs
- **Output:** Agreement percentage: where both detect beat within 50ms
- **Ownership:** analyze_hil_comparison.py calculate_beat_agreement()
- **User Sees:** Metric printed: "Beat Detection Agreement: 94.3%"
- **State Change:** None
- **Integration:** Called in metrics summary
- **Implements:** PRD §5.15.5

#### 8.3.8: Make script configurable
- **Input:** Command-line arguments: --v11 file1.csv --lightwave file2.csv --signal sine_sweep
- **Output:** Configurable comparisons for different signals and time ranges
- **Ownership:** analyze_hil_comparison.py argparse setup
- **User Sees:** Command-line interface
- **State Change:** None
- **Integration:** Script entry point
- **Implements:** PRD §5.15.6

---

## Requirements Traceability Matrix

| PRD Requirement | Tasks Implementing |
|-----------------|-------------------|
| **FR-1: Goertzel Capture** | 1.2.1, 1.2.2, 2.3.2 |
| **FR-2: Tempo Detection** | 1.5.1, 1.5.2, 2.3.2 |
| **FR-3: Visual Timing** | 6.1.1, 6.1.2, 6.2.1, 6.2.2 |
| **FR-4: Post-Processing** | 6.2.1, 6.2.2, 6.2.3 |
| **FR-5: CSV Export** | 2.2.3, 2.3.1, 2.3.2 |
| **FR-6: JSON Export** | 3.2.1-3.2.5 |
| **FR-7: Binary Export** | 3.1.1-3.1.3 |
| **FR-8: WebSocket Streaming** | 4.1.2, 4.3.1-4.3.3 |
| **FR-9: Serial Logging** | 5.1.1-5.1.3 |
| **FR-12: Dashboard** | 7.1.1-7.4.3 |
| **NFR-1: No Performance Constraints** | 1.1.2 (allocate all buffers) |
| **NFR-2: Accept Audio FPS Reduction** | 2.3.2 (SD writes may slow audio) |
| **NFR-5: Microsecond Timing** | 1.2.3, 6.1.2, 6.2.1 |
| **Story §5.1: Goertzel** | 1.2.1, 1.2.2 |
| **Story §5.2: Spectral Flux** | 1.4.1 |
| **Story §5.3: Tempo** | 1.5.1, 1.5.2 |
| **Story §5.4: VU Meter** | 1.3.1 |
| **Story §5.5: I2S Audio** | 1.6.1, 1.6.2 |
| **Story §5.6: Light Mode Timing** | 6.1.1, 6.1.2 |
| **Story §5.7: Post-Processing Buffers** | 6.2.1-6.2.3 |
| **Story §5.8: LED Driver** | 6.3.1, 6.3.2 |
| **Story §5.9: CSV Export** | 2.2.1-2.4.2, 2.5.1-2.5.2 |
| **Story §5.10: JSON Export** | 3.2.1-3.2.5 |
| **Story §5.11: Binary Export** | 3.1.1-3.1.3 |
| **Story §5.12: WebSocket Streaming** | 4.1.1-4.3.3 |
| **Story §5.13: Serial/SD** | 5.1.1-5.2.2 |
| **Story §5.14: Benchmarking Setup** | 8.1.1-8.2.3 |
| **Story §5.15: Python Analysis** | 8.3.1-8.3.8 |
| **Story §5.16: Timing Dashboard** | 7.2.1-7.2.3 |
| **Story §5.17: Signal Plots** | 7.3.1-7.3.4 |
| **Story §5.18: Logging Controls** | 7.4.1-7.4.3 |

---

## Success Criteria Checklist

From PRD §10 Success Metrics:

- [ ] **M-1:** Complete data capture achieved - all intermediate DSP arrays logged every frame
  - Verified by: Task 1.2, 1.3, 1.4, 1.5 completion + CSV export showing all columns

- [ ] **M-2:** 10,000+ audio frames captured in single session without crashes
  - Verified by: LOG_STATUS showing frame_counter > 10,000

- [ ] **M-3:** CSV exports successfully load in Excel/pandas without errors
  - Verified by: Task 2.3 + manual Excel open test

- [ ] **M-4:** Binary exports load in NumPy/MATLAB with correct float32 decoding
  - Verified by: Task 3.1 + Python test script

- [ ] **M-5:** Timestamp alignment within 10ms across v1.1, Lightwave, Tab5 captures
  - Verified by: Task 8.3.2 timestamp alignment validation

- [ ] **M-6:** Goertzel magnitude RMS difference <5% between v1.1 and reference
  - Verified by: Task 8.3.6 RMS calculation

- [ ] **M-7:** Tempo phase alignment within π/4 radians (45°) for reference BPM
  - Verified by: Task 8.3.4 tempo comparison plot

- [ ] **M-8:** Dashboard loads and displays real-time data at 30 FPS
  - Verified by: Task 7.3.4 + browser performance profiling

---

## Task Summary

**Total Tasks:** 89 implementation tasks across 8 phases

**Phase Breakdown:**
- Phase 1 (Audio Pipeline): 18 tasks
- Phase 2 (CSV Export): 13 tasks
- Phase 3 (Binary/JSON): 11 tasks
- Phase 4 (WebSocket): 10 tasks
- Phase 5 (Serial/Fallback): 7 tasks
- Phase 6 (Visual Pipeline): 10 tasks
- Phase 7 (Dashboard): 14 tasks (Optional)
- Phase 8 (Benchmarking): 16 tasks

**Critical Path:** Phase 1 → Phase 2 → Phase 5 (robust data capture and export)

**Optional:** Phase 7 (Dashboard) - nice-to-have but not required for core algorithm analysis

---

## Implementation Notes

### Build Configuration
- Add `-DHIL_EXTENDED` to platformio.ini build_flags to enable all monitoring features
- Remove flag for standard build (no monitoring overhead)

### Memory Budget
- Capture buffers: ~30KB total
- CSV buffer: ~2KB per row (flattened arrays)
- Binary file headers: 16 bytes per file
- JSON snapshot: ~32KB (ArduinoJSON document size)

### Performance Impact
- memcpy overhead: <5µs per array (measured in Task 1.2.3)
- SD card writes: 5-20ms blocking (acceptable per NFR-2)
- WebSocket overhead: Negligible with 30 FPS decimation
- Total audio FPS reduction: ~10-20% acceptable per NFR-2

### Validation Strategy
- Phase 1: Verify capture with serial dumps of first 10 frames
- Phase 2: Load CSV in pandas, check column count and data types
- Phase 3: Load binary in NumPy, verify magic header and float values
- Phase 4: Open browser console, verify binary frames received
- Phase 5: Capture serial output to file, verify data integrity
- Phase 6: Check profiler stats show all mode names
- Phase 8: Run Python script on real captures, verify plots generated

---

**Document Version:** 1.0
**Generated From:** PRD v1.0 + Technical Blueprint v1.0
**Last Updated:** 2026-01-14
