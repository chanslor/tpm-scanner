#!/bin/bash
#
# TPMS Scanner - Build and Deploy Script
#
# Usage:
#   ./deploy.sh          - Compile and upload (default)
#   ./deploy.sh compile  - Compile only
#   ./deploy.sh upload   - Upload only
#   ./deploy.sh monitor  - Serial monitor
#   ./deploy.sh libs     - Install required libraries
#

set -e

# Configuration
SKETCH_DIR="$(cd "$(dirname "$0")" && pwd)"
SKETCH_NAME="tpm-scanner"
BUILD_DIR="${SKETCH_DIR}/build"

# Board settings for standard ESP32 DevKit (30-pin)
# Using default partition since no TFT display library needed
BOARD_FQBN="esp32:esp32:esp32"

# Find the serial port
find_port() {
    # Try common USB serial ports
    for port in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
        if [ -e "$port" ]; then
            echo "$port"
            return 0
        fi
    done

    # Fallback: list available ports
    echo "Error: No serial port found!" >&2
    echo "Available ports:" >&2
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  None found" >&2
    return 1
}

# Compile the sketch
compile() {
    echo "=== Compiling ${SKETCH_NAME} ==="
    echo "Board: ${BOARD_FQBN}"
    echo "Build directory: ${BUILD_DIR}"
    echo ""

    mkdir -p "${BUILD_DIR}"

    arduino-cli compile \
        --fqbn "${BOARD_FQBN}" \
        --build-path "${BUILD_DIR}" \
        --warnings default \
        "${SKETCH_DIR}"

    echo ""
    echo "=== Compilation successful ==="

    # Show binary size
    if [ -f "${BUILD_DIR}/${SKETCH_NAME}.ino.bin" ]; then
        SIZE=$(ls -lh "${BUILD_DIR}/${SKETCH_NAME}.ino.bin" | awk '{print $5}')
        echo "Binary size: ${SIZE}"
    fi
}

# Upload to board
upload() {
    PORT=$(find_port)
    if [ -z "$PORT" ]; then
        exit 1
    fi

    echo "=== Uploading to ${PORT} ==="

    arduino-cli upload \
        --fqbn "${BOARD_FQBN}" \
        --port "${PORT}" \
        --input-dir "${BUILD_DIR}" \
        "${SKETCH_DIR}"

    echo ""
    echo "=== Upload successful ==="
    echo "Run './deploy.sh monitor' in a separate terminal to view output"
}

# Serial monitor
monitor() {
    PORT=$(find_port)
    if [ -z "$PORT" ]; then
        exit 1
    fi

    echo "=== Starting serial monitor on ${PORT} ==="
    echo "Press Ctrl+C to exit"
    echo ""

    arduino-cli monitor \
        --port "${PORT}" \
        --config baudrate=115200
}

# Install required libraries
install_libs() {
    echo "=== Installing required libraries ==="

    # Adafruit SSD1306 - OLED display driver (primary)
    arduino-cli lib install "Adafruit SSD1306"

    # Adafruit GFX - Graphics library for OLED
    arduino-cli lib install "Adafruit GFX Library"

    # U8g2 - Alternative OLED display driver
    arduino-cli lib install "U8g2"

    echo ""
    echo "=== Libraries installed ==="
    echo ""
    echo "Adafruit SSD1306 library installed for OLED display support."
}

# Show usage
usage() {
    echo "TPMS Scanner - Build and Deploy Script"
    echo ""
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  (default)  - Compile and upload"
    echo "  compile    - Compile only"
    echo "  upload     - Upload only (requires previous compile)"
    echo "  monitor    - Open serial monitor"
    echo "  libs       - Install required libraries"
    echo "  help       - Show this message"
    echo ""
}

# Main entry point
case "${1:-}" in
    compile)
        compile
        ;;
    upload)
        upload
        ;;
    monitor)
        monitor
        ;;
    libs)
        install_libs
        ;;
    help|--help|-h)
        usage
        ;;
    "")
        # Default: compile and upload
        compile
        echo ""
        upload
        ;;
    *)
        echo "Unknown command: $1"
        usage
        exit 1
        ;;
esac
