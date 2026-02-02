#!/bin/bash
# Baseline gate script - verifies green baseline invariants
# Usage: ./scripts/baseline_gate.sh /dev/tty.usbmodem5AAF2781791

set -e

PORT="${1:-/dev/tty.usbmodem5AAF2781791}"
TIMEOUT=30
TEMP_LOG=$(mktemp)

echo "=== Baseline Gate Test ==="
echo "Port: $PORT"
echo "Timeout: ${TIMEOUT}s"
echo ""

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "ERROR: Port $PORT does not exist"
    exit 1
fi

# Monitor serial output
echo "Monitoring serial output..."
timeout $TIMEOUT cat "$PORT" > "$TEMP_LOG" 2>&1 || true

# Check proof lines
PASS=0
FAIL=0

echo ""
echo "=== Checking Proof Lines ==="

# Check ESP-IDF version
if grep -q "ESP-IDF: v5.5.2" "$TEMP_LOG"; then
    echo "✓ ESP-IDF: v5.5.2"
    ((PASS++))
else
    echo "✗ ESP-IDF: v5.5.2 (NOT FOUND)"
    ((FAIL++))
fi

# Check FreeRTOS tick
if grep -q "FreeRTOS tick: 1000 Hz" "$TEMP_LOG"; then
    echo "✓ FreeRTOS tick: 1000 Hz"
    ((PASS++))
else
    echo "✗ FreeRTOS tick: 1000 Hz (NOT FOUND)"
    ((FAIL++))
fi

# Check I2S DMA config
if grep -q "I2S configured: DMA frame=128" "$TEMP_LOG"; then
    echo "✓ I2S configured: DMA frame=128"
    ((PASS++))
else
    echo "✗ I2S configured: DMA frame=128 (NOT FOUND)"
    ((FAIL++))
fi

# Check hop rate (capture and fast should be ~125 Hz)
if grep -qE "hop_rate.*capture=12[0-9]/sec.*fast=12[0-9]/sec" "$TEMP_LOG" || \
   grep -qE "Hop rates: capture=12[0-9]\.[0-9]+ Hz.*fast=12[0-9]\.[0-9]+ Hz" "$TEMP_LOG"; then
    echo "✓ hop_rate: capture/fast ~125 Hz"
    ((PASS++))
else
    echo "✗ hop_rate: capture/fast ~125 Hz (NOT FOUND or OUT OF RANGE)"
    ((FAIL++))
fi

# Check overruns
if grep -qE "capture_overruns=0.*fast_overruns=0" "$TEMP_LOG" || \
   grep -qE "capture_overruns=0 fast_overruns=0" "$TEMP_LOG"; then
    echo "✓ capture_overruns=0 fast_overruns=0"
    ((PASS++))
else
    echo "✗ capture_overruns=0 fast_overruns=0 (NOT FOUND or NON-ZERO)"
    ((FAIL++))
fi

# Check max_fast_us < 500
if grep -qE "max_fast_us=[0-4][0-9][0-9]" "$TEMP_LOG" || \
   grep -qE "max_fast_us=[0-9][0-9]" "$TEMP_LOG"; then
    echo "✓ max_fast_us < 500"
    ((PASS++))
else
    # Check if max_fast_us exists but might be >= 500
    if grep -qE "max_fast_us=" "$TEMP_LOG"; then
        MAX_FAST=$(grep -oE "max_fast_us=[0-9]+" "$TEMP_LOG" | head -1 | cut -d= -f2)
        if [ -n "$MAX_FAST" ] && [ "$MAX_FAST" -lt 500 ]; then
            echo "✓ max_fast_us=$MAX_FAST < 500"
            ((PASS++))
        else
            echo "✗ max_fast_us=$MAX_FAST >= 500 (FAIL)"
            ((FAIL++))
        fi
    else
        echo "✗ max_fast_us (NOT FOUND)"
        ((FAIL++))
    fi
fi

echo ""
echo "=== Results ==="
echo "Passed: $PASS"
echo "Failed: $FAIL"

# Cleanup
rm -f "$TEMP_LOG"

if [ $FAIL -eq 0 ]; then
    echo ""
    echo "✓ BASELINE GATE: PASS"
    exit 0
else
    echo ""
    echo "✗ BASELINE GATE: FAIL"
    exit 1
fi
