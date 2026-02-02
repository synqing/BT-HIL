# Phase 1: Responsiveness Harness - Implementation Complete

## Overview

Responsiveness measurement infrastructure has been implemented to establish baseline latency metrics. This enables objective measurement of "hair trigger" responsiveness before making tempo subsystem changes.

## Files Created

1. **`main/audio_responsiveness.h`** - Header with API definitions
2. **`main/audio_responsiveness.c`** - Implementation with transient detection and latency tracking

## Files Modified

1. **`main/audio_producer.c`**
   - Added `#include "audio_responsiveness.h"`
   - Call `responsiveness_init()` in `audio_producer_start()`
   - Call `responsiveness_feed_frame()` in `fast_lane_task()` after frame publish

2. **`main/main.c`**
   - Added `#include "audio_responsiveness.h"`
   - Call `responsiveness_log_stats()` in heartbeat loop

3. **`main/CMakeLists.txt`**
   - Added `audio_responsiveness.c` to SRCS

## Features Implemented

### Transient Detection

- **Onset-based**: Detects when `onset_strength` crosses threshold (0.1) upward
- **VU-based backup**: Also detects when `vu_peak` crosses threshold (0.05) upward
- Uses capture timestamp (`t_capture_us`) for accurate timing

### Latency Chain Tracking

1. **Transient detection** → `t_transient_us` (from `AudioFrame.t_capture_us`)
2. **Onset peak** → Tracks peak `onset_strength` during transient
3. **Visual commit** → Currently stubbed to publish time (will be updated when visual consumer integrated)

### Statistics Computation

- **History buffer**: 64 events
- **Metrics computed**:
  - Median latency
  - Average latency
  - p95 latency (95th percentile)
  - Maximum latency
- **Separate tracking**:
  - Onset latency: `t_transient_us` → `t_publish_us`
  - E2E latency: `t_transient_us` → `t_visual_commit_us`

### Gate Validation

- **Onset latency target**: ≤16 ms (1-2 hops at 8 ms)
- **E2E latency gate**: ≤24 ms (hard limit)
- Logs warnings when p95 exceeds gates

## Usage

### Automatic Logging

Statistics are logged automatically every ~1 second via `responsiveness_log_stats()`:

```
I (12345) responsiveness: Responsiveness stats (n=42 transients):
I (12345) responsiveness:   Onset latency: median=8500 avg=9200 p95=12000 max=15000 us (target ≤16000 us)
I (12345) responsiveness:   E2E latency:   median=8500 avg=9200 p95=12000 max=15000 us (target ≤24000 us)
```

### Manual Query

Use `responsiveness_get_stats()` to query current statistics programmatically:

```c
uint32_t transient_count, onset_median, onset_p95, e2e_median, e2e_p95;
responsiveness_get_stats(&transient_count, &onset_median, &onset_p95, 
                         &e2e_median, &e2e_p95);
```

## Testing

### Impulse Test Mode

1. **Generate transient**: Finger snap, tap microphone, or play percussive audio
2. **Observe logs**: Check for "Transient detected" DEBUG logs
3. **Wait for stats**: Statistics log every ~1 second
4. **Validate gates**: Check if p95 latency meets targets

### Expected Behavior

- **Transient detection**: Should detect within 1-2 hops (8-16 ms)
- **Onset latency**: Should be ≤16 ms for "hair trigger" feel
- **E2E latency**: Should be ≤24 ms (gate)

## Next Steps

Phase 1 establishes baseline metrics. Once baseline is measured:

1. **Phase 2**: Implement fixed-rate novelty logger (50 Hz, independent of slow lane)
2. **Phase 3**: Replace autocorrelation with Goertzel tempo bank
3. **Phase 4**: Add octave resolver + parity mode

## Configuration

Thresholds can be adjusted in `audio_responsiveness.h`:

```c
#define RESPONSIVENESS_ONSET_THRESHOLD 0.1f  /* Onset strength threshold */
#define RESPONSIVENESS_VU_THRESHOLD 0.05f    /* VU peak threshold */
#define RESPONSIVENESS_HISTORY_LEN 64        /* Latency history size */
```

## Notes

- Visual commit timestamp is currently stubbed to publish time
- Will be updated when visual consumer (LED output) is integrated
- Statistics use internal timing (1 second interval) to avoid excessive logging
- Transient detection uses both onset and VU thresholds for robustness
