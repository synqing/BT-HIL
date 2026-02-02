# HIL CSV Capture Guide

**Emotiscope.HIL - Hardware-in-the-Loop DSP Data Export**

This document describes how to capture real-time DSP pipeline data from Emotiscope for offline analysis in Python, MATLAB, or other tools.

---

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Quick Start](#quick-start)
4. [Capture Methods](#capture-methods)
5. [CSV Format Specification](#csv-format-specification)
6. [Column Reference](#column-reference)
7. [DSP Signal Specifications](#dsp-signal-specifications)
8. [Analysis Examples](#analysis-examples)
9. [Troubleshooting](#troubleshooting)

---

## Overview

The HIL CSV capture system exports the complete DSP pipeline state at each audio frame (~30 Hz). This includes:

- **Spectrogram**: 64 Goertzel frequency bins (A1-C7, semitone-spaced)
- **Chromagram**: 12 pitch class energy values (C through B)
- **VU Meter**: Level, peak, and noise floor
- **Tempo Detection**: 96 BPM bins with magnitude, phase, and beat sync

### Use Cases

- Reverse-engineering the Emotiscope audio analysis algorithms
- Validating DSP implementations in other projects
- Analyzing frequency response and tempo detection accuracy
- Debugging audio-reactive behavior

---

## Prerequisites

### Hardware

| Component | Specification |
|-----------|---------------|
| Device | ESP32-S3-DevKitC-1 with Emotiscope firmware |
| Connection | USB-C to computer |
| Serial Port | `/dev/tty.usbmodem1101` (macOS) or `COMx` (Windows) |
| Baud Rate | 921600 |

### Software

```bash
# Required tools
brew install arduino-cli websocat

# Verify device connection
ls /dev/tty.usb*
```

### Firmware

The device must be running **HIL-extended firmware** (built with `-DHIL_EXTENDED` flag):

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc,PartitionScheme=custom,FlashSize=8M" \
  --build-property "build.extra_flags=-DHIL_EXTENDED" \
  v1.1_build

arduino-cli upload \
  -p /dev/tty.usbmodem1101 \
  --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc,PartitionScheme=custom,FlashSize=8M" \
  v1.1_build
```

---

## Quick Start

### 1. Start Serial Capture

```bash
# Start capturing serial output to file
arduino-cli monitor -p /dev/tty.usbmodem1101 -c baudrate=921600 --raw > capture.txt &
CAPTURE_PID=$!
```

### 2. Enable CSV Export via WebSocket

```bash
# Connect to device and start export
echo "log|start|serial" | websocat ws://192.168.1.111/ws
```

### 3. Wait for Desired Duration

```bash
# Capture for 30 seconds
sleep 30
```

### 4. Stop Export and Save

```bash
# Stop the CSV export
echo "log|stop" | websocat ws://192.168.1.111/ws

# Stop serial capture
kill $CAPTURE_PID
```

### 5. Extract Clean CSV

```bash
# Extract data rows (skip any non-CSV lines)
grep -E "^[0-9]{5,}," capture.txt > data_only.csv

# Add header (see Column Reference below)
# Or use the provided Python script
```

---

## Capture Methods

### Method 1: Arduino CLI Monitor (Recommended)

Best for long captures with reliable data integrity.

```bash
# Start monitor in raw mode
arduino-cli monitor -p /dev/tty.usbmodem1101 -c baudrate=921600 --raw > /tmp/capture.txt &

# Send start command
echo "log|start|serial" | websocat ws://192.168.1.111/ws

# Wait...
sleep 60

# Send stop command
echo "log|stop" | websocat ws://192.168.1.111/ws
```

### Method 2: Direct Serial Read

For scripted/automated captures.

```bash
# Configure serial port
stty -f /dev/tty.usbmodem1101 raw 921600

# Capture with timeout
timeout 30 cat /dev/tty.usbmodem1101 > capture.txt &

# Start export
echo "log|start|serial" | websocat ws://192.168.1.111/ws
```

### Method 3: Python Serial Capture

For integration with analysis scripts.

```python
import serial
import time

ser = serial.Serial('/dev/tty.usbmodem1101', 921600, timeout=1)

# Start capture (send via separate WebSocket connection)
# ... start log|start|serial via websocket ...

data = []
start_time = time.time()
while time.time() - start_time < 30:  # 30 second capture
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line and line[0].isdigit():
        data.append(line)

ser.close()
```

---

## CSV Format Specification

### File Structure

```
timestamp_ms,vu_level,vu_max,vu_floor,spec_0,...,spec_63,chroma_0,...,chroma_11,tempi_mag_0,...,tempi_mag_95,tempi_phase_0,...,tempi_phase_95,tempi_beat_0,...,tempi_beat_95
94152,0.8127,0.8127,0.0002,0.0220,0.0359,...
94184,0.8127,0.8127,0.0002,0.0110,0.0315,...
```

### Column Layout

| Range | Count | Description |
|-------|-------|-------------|
| 0 | 1 | `timestamp_ms` - Milliseconds since boot |
| 1-3 | 3 | VU meter: `vu_level`, `vu_max`, `vu_floor` |
| 4-67 | 64 | Spectrogram: `spec_0` through `spec_63` |
| 68-79 | 12 | Chromagram: `chroma_0` through `chroma_11` |
| 80-175 | 96 | Tempo magnitudes: `tempi_mag_0` through `tempi_mag_95` |
| 176-271 | 96 | Tempo phases: `tempi_phase_0` through `tempi_phase_95` |
| 272-367 | 96 | Beat sync: `tempi_beat_0` through `tempi_beat_95` |

**Total: 368 columns**

### Data Types

| Column | Type | Range | Unit |
|--------|------|-------|------|
| timestamp_ms | uint32 | 0 - 4,294,967,295 | milliseconds |
| vu_level | float | 0.0 - 1.0 | normalized amplitude |
| vu_max | float | 0.0 - 1.0 | peak amplitude |
| vu_floor | float | 0.0 - 1.0 | noise floor |
| spec_N | float | 0.0 - ~1.0 | Goertzel magnitude |
| chroma_N | float | 0.0 - ~1.0 | pitch class energy |
| tempi_mag_N | float | 0.0 - ~1.0 | tempo strength |
| tempi_phase_N | float | -π to +π | phase angle (radians) |
| tempi_beat_N | float | -1.0 to +1.0 | sin(phase) beat sync |

---

## Column Reference

### VU Meter (Columns 1-3)

| Column | Name | Description |
|--------|------|-------------|
| 1 | `vu_level` | Current RMS amplitude (smoothed) |
| 2 | `vu_max` | Peak detector with slow decay |
| 3 | `vu_floor` | Adaptive noise floor estimate |

### Spectrogram (Columns 4-67)

64 Goertzel frequency bins, semitone-spaced from A1 (55 Hz) to C7 (2093 Hz).

| Column | Name | Frequency (Hz) | Musical Note |
|--------|------|----------------|--------------|
| 4 | `spec_0` | 55.00 | A1 |
| 5 | `spec_1` | 58.27 | A#1/Bb1 |
| 6 | `spec_2` | 61.74 | B1 |
| 7 | `spec_3` | 65.41 | C2 |
| 8 | `spec_4` | 69.30 | C#2/Db2 |
| 9 | `spec_5` | 73.42 | D2 |
| 10 | `spec_6` | 77.78 | D#2/Eb2 |
| 11 | `spec_7` | 82.41 | E2 |
| 12 | `spec_8` | 87.31 | F2 |
| 13 | `spec_9` | 92.50 | F#2/Gb2 |
| 14 | `spec_10` | 98.00 | G2 |
| 15 | `spec_11` | 103.83 | G#2/Ab2 |
| 16 | `spec_12` | 110.00 | A2 |
| ... | ... | ... | ... |
| 67 | `spec_63` | 2093.00 | C7 |

**Full frequency table:**

```
Bin  Freq(Hz)   Note    | Bin  Freq(Hz)   Note    | Bin  Freq(Hz)   Note
-----|------------------|-----|------------------|-----|------------------
0    55.00     A1      | 24   220.00    A3      | 48   880.00    A5
1    58.27     A#1     | 25   233.08    A#3     | 49   932.33    A#5
2    61.74     B1      | 26   246.94    B3      | 50   987.77    B5
3    65.41     C2      | 27   261.63    C4      | 51   1046.50   C6
4    69.30     C#2     | 28   277.18    C#4     | 52   1108.73   C#6
5    73.42     D2      | 29   293.66    D4      | 53   1174.66   D6
6    77.78     D#2     | 30   311.13    D#4     | 54   1244.51   D#6
7    82.41     E2      | 31   329.63    E4      | 55   1318.51   E6
8    87.31     F2      | 32   349.23    F4      | 56   1396.91   F6
9    92.50     F#2     | 33   369.99    F#4     | 57   1479.98   F#6
10   98.00     G2      | 34   392.00    G4      | 58   1567.98   G6
11   103.83    G#2     | 35   415.30    G#4     | 59   1661.22   G#6
12   110.00    A2      | 36   440.00    A4      | 60   1760.00   A6
13   116.54    A#2     | 37   466.16    A#4     | 61   1864.66   A#6
14   123.47    B2      | 38   493.88    B4      | 62   1975.53   B6
15   130.81    C3      | 39   523.25    C5      | 63   2093.00   C7
16   138.59    C#3     | 40   554.37    C#5     |
17   146.83    D3      | 41   587.33    D5      |
18   155.56    D#3     | 42   622.25    D#5     |
19   164.81    E3      | 43   659.26    E5      |
20   174.61    F3      | 44   698.46    F5      |
21   185.00    F#3     | 45   739.99    F#5     |
22   196.00    G3      | 46   783.99    G5      |
23   207.65    G#3     | 47   830.61    G#5     |
```

**Frequency formula:** `freq = 55 * 2^(bin/12)` (A1 = 55 Hz base)

### Chromagram (Columns 68-79)

12 pitch class columns representing energy in each musical note regardless of octave.

| Column | Name | Pitch Class |
|--------|------|-------------|
| 68 | `chroma_0` | C |
| 69 | `chroma_1` | C# / Db |
| 70 | `chroma_2` | D |
| 71 | `chroma_3` | D# / Eb |
| 72 | `chroma_4` | E |
| 73 | `chroma_5` | F |
| 74 | `chroma_6` | F# / Gb |
| 75 | `chroma_7` | G |
| 76 | `chroma_8` | G# / Ab |
| 77 | `chroma_9` | A |
| 78 | `chroma_10` | A# / Bb |
| 79 | `chroma_11` | B |

The chromagram is computed by summing spectrogram bins that correspond to the same pitch class across octaves.

### Tempo Detection (Columns 80-367)

96 BPM bins covering 48-143 BPM range.

#### Magnitude (Columns 80-175)

| Column | Name | BPM |
|--------|------|-----|
| 80 | `tempi_mag_0` | 48 BPM |
| 81 | `tempi_mag_1` | 49 BPM |
| ... | ... | ... |
| 175 | `tempi_mag_95` | 143 BPM |

**BPM formula:** `bpm = 48 + index`

#### Phase (Columns 176-271)

Phase angle in radians (-π to +π) indicating where in the beat cycle the signal is.

| Column | Name | Description |
|--------|------|-------------|
| 176 | `tempi_phase_0` | Phase at 48 BPM |
| ... | ... | ... |
| 271 | `tempi_phase_95` | Phase at 143 BPM |

#### Beat Sync (Columns 272-367)

`sin(phase)` value for easy beat synchronization (-1.0 to +1.0).

| Column | Name | Description |
|--------|------|-------------|
| 272 | `tempi_beat_0` | Beat sync at 48 BPM |
| ... | ... | ... |
| 367 | `tempi_beat_95` | Beat sync at 143 BPM |

**Beat detection:** When `tempi_beat_N` crosses from negative to positive, a beat occurs at that BPM.

---

## DSP Signal Specifications

### Audio Input

| Parameter | Value |
|-----------|-------|
| Sample Rate | 12,800 Hz |
| Sample Buffer | 4,096 samples |
| Buffer Duration | ~320 ms |
| ADC Resolution | 12-bit |
| Input | I2S MEMS microphone |

### Goertzel Analysis

| Parameter | Value |
|-----------|-------|
| Bins | 64 |
| Frequency Range | 55 Hz (A1) to 2093 Hz (C7) |
| Spacing | Semitone (12-TET) |
| Window | None (rectangular) |
| Output | Magnitude (not power) |

### Tempo Detection

| Parameter | Value |
|-----------|-------|
| BPM Range | 48 - 143 BPM |
| Bins | 96 |
| Resolution | 1 BPM |
| Method | Novelty-domain Goertzel |
| Novelty Source | Spectral flux |

### Frame Rate

| Parameter | Value |
|-----------|-------|
| CPU Core Rate | ~30 Hz |
| GPU Core Rate | ~400 Hz |
| CSV Export Rate | ~30 Hz (CPU-bound) |

---

## Analysis Examples

### Python: Load and Plot Spectrogram

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv('hil_capture.csv')

# Extract spectrogram columns
spec_cols = [f'spec_{i}' for i in range(64)]
spectrogram = df[spec_cols].values

# Create time axis
time_ms = df['timestamp_ms'].values
time_s = (time_ms - time_ms[0]) / 1000.0

# Frequency axis (semitone-spaced from A1)
frequencies = 55 * (2 ** (np.arange(64) / 12))

# Plot spectrogram
plt.figure(figsize=(12, 6))
plt.pcolormesh(time_s, frequencies, spectrogram.T, shading='auto', cmap='magma')
plt.yscale('log')
plt.ylabel('Frequency (Hz)')
plt.xlabel('Time (s)')
plt.colorbar(label='Magnitude')
plt.title('Emotiscope Spectrogram')
plt.show()
```

### Python: Find Dominant BPM

```python
import pandas as pd
import numpy as np

df = pd.read_csv('hil_capture.csv')

# Extract tempo magnitude columns
tempi_cols = [f'tempi_mag_{i}' for i in range(96)]
tempi_mags = df[tempi_cols].values

# Average across all frames
avg_mags = tempi_mags.mean(axis=0)

# Find dominant BPM
dominant_idx = np.argmax(avg_mags)
dominant_bpm = 48 + dominant_idx
dominant_strength = avg_mags[dominant_idx]

print(f"Dominant BPM: {dominant_bpm} (strength: {dominant_strength:.4f})")

# Plot tempo histogram
plt.figure(figsize=(10, 4))
plt.bar(np.arange(48, 144), avg_mags)
plt.axvline(dominant_bpm, color='red', linestyle='--', label=f'Peak: {dominant_bpm} BPM')
plt.xlabel('BPM')
plt.ylabel('Average Magnitude')
plt.title('Tempo Detection Histogram')
plt.legend()
plt.show()
```

### Python: VU Meter Analysis

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('hil_capture.csv')

time_s = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0

plt.figure(figsize=(12, 4))
plt.fill_between(time_s, 0, df['vu_level'], alpha=0.7, label='VU Level')
plt.plot(time_s, df['vu_max'], 'r-', linewidth=0.5, label='VU Max')
plt.plot(time_s, df['vu_floor'], 'g--', linewidth=0.5, label='VU Floor')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('VU Meter')
plt.legend()
plt.ylim(0, 1)
plt.show()
```

### Python: Beat Detection

```python
import pandas as pd
import numpy as np

df = pd.read_csv('hil_capture.csv')

# Find the dominant BPM
tempi_cols = [f'tempi_mag_{i}' for i in range(96)]
avg_mags = df[tempi_cols].mean()
dominant_idx = avg_mags.argmax()
dominant_bpm = 48 + dominant_idx

print(f"Dominant BPM: {dominant_bpm}")

# Get beat sync for dominant BPM
beat_col = f'tempi_beat_{dominant_idx}'
beat_sync = df[beat_col].values

# Detect zero crossings (negative to positive = beat)
beats = []
for i in range(1, len(beat_sync)):
    if beat_sync[i-1] < 0 and beat_sync[i] >= 0:
        beats.append(df['timestamp_ms'].iloc[i])

print(f"Detected {len(beats)} beats")

# Calculate inter-beat intervals
if len(beats) > 1:
    ibis = np.diff(beats)
    measured_bpm = 60000 / np.mean(ibis)
    print(f"Measured BPM from IBIs: {measured_bpm:.1f}")
```

### MATLAB: Load and Analyze

```matlab
% Load CSV
data = readtable('hil_capture.csv');

% Extract arrays
timestamps = data.timestamp_ms;
vu_level = data.vu_level;

% Spectrogram (columns 5-68 in MATLAB 1-indexed)
spec_cols = 5:68;
spectrogram = table2array(data(:, spec_cols));

% Tempo magnitudes (columns 81-176)
tempi_cols = 81:176;
tempi_mags = table2array(data(:, tempi_cols));

% Find dominant BPM
avg_mags = mean(tempi_mags, 1);
[~, dominant_idx] = max(avg_mags);
dominant_bpm = 47 + dominant_idx;  % 48 + (idx-1) due to 1-indexing

fprintf('Dominant BPM: %d\n', dominant_bpm);

% Plot spectrogram
figure;
imagesc((timestamps - timestamps(1))/1000, 1:64, spectrogram');
set(gca, 'YDir', 'normal');
xlabel('Time (s)');
ylabel('Frequency Bin');
title('Emotiscope Spectrogram');
colorbar;
```

---

## Troubleshooting

### No Data Captured

**Symptom:** Serial capture file is empty or contains no CSV data.

**Solutions:**
1. Verify device is connected: `ls /dev/tty.usb*`
2. Verify HIL firmware is running: Send `hil|status` via WebSocket
3. Check baud rate: Must be 921600
4. Use `arduino-cli monitor --raw` instead of `cat`

### Corrupted/Interleaved Data

**Symptom:** CSV rows contain non-numeric text mixed in.

**Solution:** This was fixed in the firmware. Ensure you have the latest version with `hil_csv_serial_export_active` flag. Rebuild and upload:

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc,PartitionScheme=custom,FlashSize=8M" \
  --build-property "build.extra_flags=-DHIL_EXTENDED" v1.1_build
arduino-cli upload -p /dev/tty.usbmodem1101 --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc,PartitionScheme=custom,FlashSize=8M" v1.1_build
```

### Wrong Column Count

**Symptom:** CSV rows have different number of columns than expected (368).

**Causes:**
- Partial row at start/end of capture
- Export options changed mid-capture

**Solution:** Filter rows by column count:
```bash
awk -F',' 'NF==368' capture.csv > clean.csv
```

### WebSocket Connection Failed

**Symptom:** `websocat` returns error or no response.

**Solutions:**
1. Verify device IP: Check serial output at boot for `IP Address: x.x.x.x`
2. Check WiFi: Device must be on same network
3. Default IP: `192.168.1.111`
4. Try ping: `ping 192.168.1.111`

### Export Already Active

**Symptom:** `log_error|already_active` response.

**Solution:** Stop existing export first:
```bash
echo "log|stop" | websocat ws://192.168.1.111/ws
sleep 1
echo "log|start|serial" | websocat ws://192.168.1.111/ws
```

---

## WebSocket Commands Reference

| Command | Response | Description |
|---------|----------|-------------|
| `ping` | `pong` | Connection test |
| `hil\|status` | `hil_status\|1\|1\|1` | Check HIL availability |
| `log\|start\|serial` | `log_started\|serial` | Start CSV to serial |
| `log\|stop` | `log_stopped\|<frames>` | Stop export |
| `log\|status` | `hil_export_status\|...` | Check export state |
| `hil\|snapshot` | `hil_snapshot\|...` | Single-frame snapshot |

---

## File Locations

| File | Description |
|------|-------------|
| `v1.1_build/hil_export.h` | CSV export implementation |
| `v1.1_build/hil_capture.h` | Capture state structure |
| `v1.1_build/commands.h` | WebSocket command handlers |
| `v1.1_build/cpu_core.h` | DSP pipeline (Goertzel, VU, tempo) |
| `v1.1_build/goertzel.h` | Goertzel magnitude calculation |
| `v1.1_build/tempo.h` | Tempo detection algorithm |

---

## Version History

| Date | Change |
|------|--------|
| 2026-01-16 | Added `hil_csv_serial_export_active` flag to suppress system logging during CSV export |
| 2026-01-15 | Initial HIL CSV export implementation |

---

## Contact

For issues with HIL capture, check the firmware source in `v1.1_build/` or create an issue in the repository.
