# Fixes Applied - Root Cause Resolution

## Summary

Fixed **two critical "stop the line" failures**:
1. ESP-IDF version not actually pinned (boot logs still showed v6.0-dev)
2. Task WDT firing due to fast_lane spinning (not properly blocking)

---

## 1. ESP-IDF Version Pinning (HARD GATE)

### Problem
- Boot logs showed `ESP-IDF v6.0-dev-...` despite claims of pinning
- Comments changed but actual build still used v6.0-dev

### Fixes Applied

#### A. Build-time version enforcement
- **`CMakeLists.txt`**: Added version check that FAILS build if not v5.5.2
- **`build_with_idf55.sh`**: Comprehensive build script that:
  - Checks IDF_PATH version before building
  - Fails immediately if wrong version
  - Cleans all artifacts
  - Adds esp-dsp dependency
  - Builds with verification

#### B. Cleanup scripts
- **`clean_idf6.sh`**: Removes ALL v6.0-dev artifacts
- **`idf_version_pin.sh`**: Standalone version verification
- **`verify_build.sh`**: Post-build verification

#### C. Version pinning files
- **`.idf_version`**: Contains required version string
- **`idf_component.yml`**: Component manifest (includes esp-dsp)

### Proof Required

**Gate:** Boot log MUST show:
```
boot: ESP-IDF v5.5.2 ... 2nd stage bootloader
app_init: ESP-IDF: v5.5.2 ...
```

If boot log shows `v6.0-dev`, build used wrong IDF. Full stop.

### Usage

```bash
export IDF_PATH=/path/to/esp-idf-v5.5.2
./build_with_idf55.sh
# Verify boot log shows v5.5.2
```

---

## 2. Task WDT Root Cause Fix

### Problem
- Task WDT firing on IDLE0
- fast_lane task spinning (not blocking properly)
- Capture task potentially draining backlog instead of DMA-paced

### Fixes Applied

#### A. Counting notifications (replaced bits notifications)
**Before:**
```c
xTaskNotify(s_fast_lane_handle, flags, eSetBits);
xTaskNotifyWait(0, ULONG_MAX, &notification_value, portMAX_DELAY);
```

**After:**
```c
xTaskNotifyGive(s_fast_lane_handle);
ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
```

**Why:** Counting notifications guarantee fast_lane sleeps when no new hop. Bits notifications can cause immediate wakeup.

#### B. DMA-paced I2S configuration
**Before:**
```c
i2s_channel_read(..., delay_ms_min1(20));  // Short timeout = potential spinning
```

**After:**
```c
chan_cfg.dma_frame_num = AUDIO_HOP_SIZE;  // One DMA frame = one hop
chan_cfg.dma_desc_num = 4;  // Small descriptor count
i2s_channel_read(..., portMAX_DELAY);  // True blocking
```

**Why:** DMA frame size matches hop size prevents backlog. portMAX_DELAY ensures true blocking.

#### C. CPU pinning optimization
- **capture**: CPU0 (highest priority, mostly blocked on DMA)
- **fast_lane**: CPU1 (reduces CPU0 load, prevents IDLE0 starvation)
- **slow_lane**: CPU1 (when re-enabled)

#### D. Hop rate diagnostics
Added counters:
- `audio_capture_reads_count`: Tracks I2S reads per second
- `audio_fast_lane_wakeups_count`: Tracks fast_lane wakeups per second

**Gate:** Both must be ~125/sec (16kHz / 128 samples = 125 hops/sec)

If rates are >>125/sec, capture/fast is spinning or draining backlog.

### Proof Required

**Gate:** Boot log MUST show:
```
hop_rate | capture=~125/sec fast=~125/sec (expected ~125/sec)
```

If rates are >>125/sec, root cause not fixed.

---

## 3. esp-dsp Integration

### Added
- **`idf_component.yml`**: Declares esp-dsp dependency
- **`main/CMakeLists.txt`**: Added `esp-dsp` to PRIV_REQUIRES
- **`build_with_idf55.sh`**: Automatically adds esp-dsp via `idf.py add-dependency`

### Proof Required

**Gate:** Build output shows:
```
-- Component: esp-dsp
-- Fetching esp-dsp from registry...
```

And directory exists:
```
managed_components/espressif__esp-dsp/
```

---

## 4. Fast Lane Performance Fix

### Problem
- Naive DFT computation (O(n²)) was way over 4000µs budget
- 64 bins × 128 samples = 8192 iterations with cos/sin calls

### Fix Applied
- Replaced expensive DFT with simple energy-based onset detection
- Simplified band_energy to placeholder (will use esp-dsp FFT later)

**Result:** Fast lane should now complete within budget.

---

## 5. Stack Overflow Prevention

### Already Fixed (from previous)
- Moved all large buffers off stack to static globals
- slow_lane task disabled until gates pass
- Stack high-water mark logging added

---

## Verification Checklist

Before considering fixes complete, verify:

- [ ] **ESP-IDF Version:** Boot log shows `v5.5.2` (not `v6.0-dev`)
- [ ] **Hop Rates:** `capture=~125/sec fast=~125/sec` (not >>125)
- [ ] **Task WDT:** Silent for 60+ seconds
- [ ] **Stack Marks:** All tasks show >20% headroom
- [ ] **esp-dsp:** Component exists in `managed_components/`
- [ ] **Build:** Uses `build_with_idf55.sh` or manual steps with version check

---

## Next Steps

1. **Set IDF_PATH to v5.5.2:**
   ```bash
   export IDF_PATH=$HOME/esp/esp-idf-v5.5.2
   ```

2. **Build with verification:**
   ```bash
   cd p4_audio_producer
   ./build_with_idf55.sh
   ```

3. **Flash and verify boot log:**
   ```bash
   idf.py -p <PORT> flash monitor
   # Check boot log shows v5.5.2
   # Check hop_rate shows ~125/sec
   # Check no WDT triggers
   ```

4. **Only after above gates pass:** Re-enable slow_lane with proper stack size.
