# Quick Start - Build with ESP-IDF v5.5.2

## The Problem You Hit

You set `IDF_PATH=/path/to/esp-idf-v5.5.2` (placeholder) instead of the actual path.

## The Fix

The build script now **auto-detects** v5.5.2 if IDF_PATH is not set or is a placeholder.

## How to Build (3 Steps)

```bash
cd /Users/spectrasynq/Workspace_Management/Software/Emotiscope.HIL/p4_audio_producer

# Option 1: Let script auto-detect (recommended)
./build_with_idf55.sh

# Option 2: Set IDF_PATH manually
export IDF_PATH=$HOME/esp/esp-idf-v5.5.2
./build_with_idf55.sh
```

## If Python Environment Issues

If you see errors about Python dependencies or `idf6.0_py3.11_env`:

```bash
cd $HOME/esp/esp-idf-v5.5.2
./install.sh
# Or let export.sh auto-install when you source it
```

## Verify It Worked

After building and flashing, boot log MUST show:
```
boot: ESP-IDF v5.5.2 ... 2nd stage bootloader
app_init: ESP-IDF: v5.5.2 ...
hop_rate | capture=~125/sec fast=~125/sec
```

If boot log shows `v6.0-dev`, the build used wrong IDF. Full stop.
