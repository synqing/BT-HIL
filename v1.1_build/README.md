Emotiscope v1.1 Firmware – Arduino CLI Build Instructions
=========================================================

This directory contains the Emotiscope v1.1 firmware. The PlatformIO
configuration here is kept only for reference and MUST NOT be used to
build this version. The only supported build path is via `arduino-cli`.


Supported Build Tool
--------------------

- Tool: `arduino-cli`
- Board core: `esp32:esp32`
- Target board: `esp32:esp32:esp32s3` (ESP32-S3 Dev Module)

PlatformIO (`platformio.ini`) is known to fail for this project and
should not be used for building or uploading v1.1 firmware.


Prerequisites
-------------

1. Install Arduino CLI (if not already installed):

   - macOS (Homebrew example):
     - `brew install arduino-cli`

2. Install the ESP32 core for Arduino:

   - `arduino-cli core update-index`
   - `arduino-cli core install esp32:esp32`

3. Optional: list ESP32-S3 boards to verify the core is available:

   - `arduino-cli board listall esp32s3`


Build Commands (from repo root)
-------------------------------

All commands below assume your current working directory is the repo
root, e.g.:

- `/Users/spectrasynq/Workspace_Management/Software/Emotiscope.HIL`

The sketch directory for this firmware is `v1.1_build`.


1. Standard v1.1 Firmware (no HIL overhead)
-------------------------------------------

Build the standard firmware without extended HIL monitoring:

```bash
arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,PSRAM=disabled,PartitionScheme=custom \
  --build-property "compiler.cpp.extra_flags=-O2" \
  v1.1_build
```

This produces a standard v1.1 binary without the `HIL_EXTENDED` flag.


2. HIL-Extended v1.1 Firmware
-----------------------------

Build the HIL-instrumented firmware (allocates capture buffers and
enables additional memcpy instrumentation):

```bash
arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,PSRAM=disabled,PartitionScheme=custom \
  --build-property "compiler.cpp.extra_flags=-O2 -DHIL_EXTENDED" \
  v1.1_build
```

Notes:

- The only difference from the standard build is the addition of
  `-DHIL_EXTENDED` in `compiler.cpp.extra_flags`.
- This flag enables extended HIL monitoring code paths and increases
  memory usage, but is the preferred build when performing detailed
  DSP/visual pipeline analysis.


Upload Notes
------------

These instructions focus on compilation. Upload can be performed using
the usual `arduino-cli upload` flow once the correct serial port is
known, for example:

```bash
arduino-cli upload \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,PSRAM=disabled,PartitionScheme=custom \
  -p /dev/cu.usbmodem21401 \
  v1.1_build
```

Adjust the serial port as appropriate for the host system in use.


Important Reminder
------------------

- Do NOT attempt to build Emotiscope v1.1 with PlatformIO.
- Use the Arduino CLI commands above for all future builds. 

