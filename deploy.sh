#!/bin/bash
#
# TPMS Scanner - Build and Deploy Script
#
# Usage:
#   ./deploy.sh          - Compile and upload (default)
#   ./deploy.sh compile  - Compile only
#   ./deploy.sh upload   - Upload only
#   ./deploy.sh monitor  - Serial monitor
#   ./deploy.sh libs     - Install TFT_eSPI library
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

    # TFT_eSPI - TFT display driver for ST7796
    arduino-cli lib install "TFT_eSPI"

    echo ""
    echo "=== Libraries installed ==="
    echo ""
    echo "TFT_eSPI library installed."
    echo ""
    echo "IMPORTANT: You must copy User_Setup.h from this project to the TFT_eSPI library folder!"
    echo "  Linux:   ~/Arduino/libraries/TFT_eSPI/"
    echo "  Mac:     ~/Documents/Arduino/libraries/TFT_eSPI/"
    echo "  Windows: Documents\\Arduino\\libraries\\TFT_eSPI\\"
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
