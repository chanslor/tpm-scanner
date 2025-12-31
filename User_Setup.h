/*
 * User_Setup.h - TFT_eSPI Configuration for ESP32-2432S035
 *
 * This file configures the TFT_eSPI library for the ESP32-2432S035 board
 * which has a 3.5" 480x320 TFT display with ST7796 driver.
 *
 * INSTALLATION:
 *   1. Find your Arduino libraries folder:
 *      - Linux: ~/Arduino/libraries/TFT_eSPI/
 *      - Mac: ~/Documents/Arduino/libraries/TFT_eSPI/
 *      - Windows: Documents\Arduino\libraries\TFT_eSPI\
 *
 *   2. BACKUP the original User_Setup.h file
 *
 *   3. REPLACE User_Setup.h with this file (or copy contents)
 *
 *   4. Recompile your sketch
 *
 * TROUBLESHOOTING:
 *   - White screen: Wrong driver (must be ST7796_DRIVER)
 *   - No display: Check backlight pin (GPIO 27)
 *   - Inverted colors: Uncomment TFT_INVERSION_ON or OFF
 *   - Rotated wrong: Adjust tft.setRotation() in code
 */

// ====================================================================================
// DISPLAY DRIVER - CRITICAL! Must match your display
// ====================================================================================

#define ST7796_DRIVER      // 3.5" 480x320 display (ESP32-2432S035)

// ====================================================================================
// DISPLAY DIMENSIONS
// ====================================================================================

#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// ====================================================================================
// ESP32-2432S035 PIN DEFINITIONS
// ====================================================================================

// Display uses HSPI
#define TFT_MISO  12    // SPI MISO
#define TFT_MOSI  13    // SPI MOSI
#define TFT_SCLK  14    // SPI Clock
#define TFT_CS    15    // Chip select
#define TFT_DC     2    // Data/Command
#define TFT_RST   -1    // Reset (not connected, use -1)

// Backlight control
#define TFT_BL    27    // Backlight control pin
#define TFT_BACKLIGHT_ON HIGH  // HIGH to turn on backlight

// ====================================================================================
// SPI CONFIGURATION
// ====================================================================================

// Use HSPI port (display)
// VSPI is left free for CC1101 radio module
#define USE_HSPI_PORT

// SPI frequency - 80MHz works well for this display
#define SPI_FREQUENCY       80000000

// Read frequency (lower for stability)
#define SPI_READ_FREQUENCY  20000000

// Touch SPI frequency (if using SPI touch, not I2C)
#define SPI_TOUCH_FREQUENCY  2500000

// ====================================================================================
// TOUCH CONFIGURATION (GT911 I2C)
// ====================================================================================

// The ESP32-2432S035 uses GT911 capacitive touch via I2C
// Touch is handled separately from TFT_eSPI
// Touch pins:
//   SDA = GPIO 33
//   SCL = GPIO 32
//   INT = GPIO 21 (shared with CC1101 GDO0 - may need to change!)
//   RST = GPIO 25

// Uncomment if using resistive touch (not typical for this board)
// #define TOUCH_CS  33

// ====================================================================================
// FONT CONFIGURATION
// ====================================================================================

// Load standard fonts
#define LOAD_GLCD   // Font 1: Original Adafruit 8 pixel font
#define LOAD_FONT2  // Font 2: Small 16 pixel font
#define LOAD_FONT4  // Font 4: Medium 26 pixel font
#define LOAD_FONT6  // Font 6: Large 48 pixel numeric font
#define LOAD_FONT7  // Font 7: 7 segment 48 pixel font
#define LOAD_FONT8  // Font 8: Large 75 pixel font
#define LOAD_GFXFF  // FreeFonts - allows use of GFX free fonts

// Enable smooth fonts
#define SMOOTH_FONT

// ====================================================================================
// OPTIONAL: COLOR INVERSION
// ====================================================================================

// Uncomment ONE if colors are inverted
// #define TFT_INVERSION_ON
// #define TFT_INVERSION_OFF

// ====================================================================================
// OPTIONAL: RGB/BGR COLOR ORDER
// ====================================================================================

// Uncomment if red and blue are swapped
// #define TFT_RGB_ORDER TFT_BGR

// ====================================================================================
// TRANSACTIONS (for multi-SPI bus compatibility)
// ====================================================================================

// Support for multiple SPI devices on the same bus
// IMPORTANT: Enable this when using CC1101 on VSPI and display on HSPI
#define SUPPORT_TRANSACTIONS

// ====================================================================================
// DEBUG (uncomment to enable)
// ====================================================================================

// #define DEBUG_SERIAL_OUTPUT
