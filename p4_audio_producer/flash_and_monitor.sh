#!/bin/bash
# Flash and monitor script for P4 Audio Producer
# Device: /dev/tty.usbmodem5AAF2781791

set -e

PORT="/dev/tty.usbmodem5AAF2781791"
IDF_PATH="/Users/spectrasynq/esp/esp-idf-v5.5.2"
PYTHON="/Users/spectrasynq/.espressif/python_env/idf5.5_py3.9_env/bin/python"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

echo "=== P4 Audio Producer - Flash and Monitor ==="
echo ""
echo "Device port: $PORT"
echo ""

# Check if binary exists
if [ ! -f "build/p4_audio_producer.bin" ]; then
    echo "ERROR: Binary not found. Run build_with_idf55.sh first."
    exit 1
fi

echo "Step 1: Put device into bootloader mode"
echo "  - Press and HOLD the BOOT button"
echo "  - Press and RELEASE the RESET button (while holding BOOT)"
echo "  - RELEASE the BOOT button"
echo ""
echo "Press ENTER when ready to flash..."
read -r

echo ""
echo "Step 2: Flashing firmware..."
$PYTHON "$IDF_PATH/components/esptool_py/esptool/esptool.py" \
  --chip esp32p4 \
  --port "$PORT" \
  --baud 115200 \
  write_flash \
  0x20000 build/p4_audio_producer.bin

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Flash successful!"
    echo ""
    echo "Step 3: Monitoring serial output (Ctrl+C to exit)..."
    echo ""
    # Use Python's serial library to monitor (more reliable than screen)
    $PYTHON -c "
import serial
import sys
try:
    ser = serial.Serial('$PORT', 115200, timeout=1)
    print('Connected. Waiting for output...\n')
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            sys.stdout.write(data.decode('utf-8', errors='replace'))
            sys.stdout.flush()
        import time
        time.sleep(0.1)
except KeyboardInterrupt:
    print('\n\nMonitoring stopped.')
    ser.close()
except Exception as e:
    print(f'Error: {e}')
    sys.exit(1)
"
else
    echo ""
    echo "✗ Flash failed. Please check:"
    echo "  1. Device is in bootloader mode"
    echo "  2. Port $PORT is correct"
    echo "  3. No other program is using the serial port"
    exit 1
fi
