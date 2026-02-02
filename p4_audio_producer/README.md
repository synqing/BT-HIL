# P4 Audio Producer

Bulletproof two-lane Audio Producer for ESP32-P4-WIFI6. Authority: [docs/DEPARTMENTAL_AUDIO_PRODUCER_SPEC.md](../docs/DEPARTMENTAL_AUDIO_PRODUCER_SPEC.md).

**Requires:** ESP-IDF v5.5.2, target `esp32p4`. Document or verify IDF version in CI.

## Build and run

**CRITICAL: ESP-IDF v5.5.2 is REQUIRED. Build will FAIL if wrong version.**

### Quick Build (Recommended)

```bash
cd p4_audio_producer

# Set IDF_PATH to ESP-IDF v5.5.2 installation
export IDF_PATH=/path/to/esp-idf-v5.5.2   # MUST be v5.5.2

# Build with automatic version check and esp-dsp integration
./build_with_idf55.sh

# Flash and monitor
idf.py -p <PORT> flash monitor
```

### Manual Build Steps

```bash
cd p4_audio_producer

# 1. Set IDF_PATH
export IDF_PATH=/path/to/esp-idf-v5.5.2

# 2. Verify version (fails if not v5.5.2)
./idf_version_pin.sh

# 3. Source ESP-IDF
. $IDF_PATH/export.sh

# 4. Clean all artifacts (removes v6.0-dev contamination)
./clean_idf6.sh

# 5. Add esp-dsp dependency
idf.py add-dependency "espressif/esp-dsp"

# 6. Build
idf.py set-target esp32p4
idf.py build

# 7. Verify build environment
./verify_build.sh

# 8. Flash and monitor
idf.py -p <PORT> flash monitor
```

### Version Verification Gates

**Gate 1: Build-time check**
- `CMakeLists.txt` checks IDF version and fails if not v5.5.2
- `build_with_idf55.sh` verifies version before building

**Gate 2: Boot log verification**
After flashing, boot log MUST show:
- `boot: ESP-IDF v5.5.2 ... 2nd stage bootloader`
- `app_init: ESP-IDF: v5.5.2 ...`

If boot log shows `v6.0-dev`, the build used wrong IDF. Full stop.

**Gate 3: Runtime diagnostics**
- `hop_rate | capture=~125/sec fast=~125/sec` (expected ~125/sec)
- No Task WDT triggers
- Stack high-water marks show headroom

## Departments

- **Dept 0:** `audio_config.h` (single source of truth), compile-time assertions, boot banner.
- **Dept 1:** Minimal heartbeat.
- **Dept 2:** Audio proof harness (I2S std + ring buffer + fast/slow lane + AudioFrame + counters); synthetic capture by default.
- **Dept 3:** Latency chain (t_capture, t_publish, t_visual_commit) and periodic latency logs.
- **Dept 4:** IAudioCapture HAL in `include/IAudioCapture.h`; P4 implementation in `main/audio_capture_hal.c`.
- **Dept 5:** ESP-Hosted SDIO stub; full WiFi bring-up only after 72h stress and audio gates pass.

## Gates

- **Dept 0:** Build + compile-time assertion + boot banner.
- **Dept 2:** 0 capture overruns (10 min); fast lane within 8 ms budget; max margin logged.
- **Dept 3:** E2e latency ≤ 16 ms typical, ≤ 24 ms gate.
- **Dept 4:** HAL integrated; 10-min gate still pass.
- **Dept 5:** WiFi up; 10-min gate still pass (after 72h stress).
- **72h stress:** Zero crashes; leak target per project spec; 0 capture overruns and latency within gate.
