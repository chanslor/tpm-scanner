/*
 * User_Setup.h - TFT_eSPI Configuration for ILI9488 3.5" Display
 *
 * This file configures the TFT_eSPI library for a 3.5" 480x320 TFT
 * display module with ILI9488 driver, connected to ESP32 DevKit via HSPI.
 *
 * DISPLAY: 3.5" TFT LCD Touch Screen Shield 480x320 SPI ILI9488
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
 * WIRING (HSPI - separate from CC1101's VSPI):
 *   TFT_SCLK  -> GPIO 14 (Green wire)
 *   TFT_MISO  -> GPIO 12 (Purple wire)
 *   TFT_MOSI  -> GPIO 13 (White wire)
 *   TFT_CS    -> GPIO 15 (Red wire)
 *   TFT_DC    -> GPIO 2  (Yellow wire)
 *   TFT_RST   -> GPIO 4  (Orange wire)
 *   TFT_BL    -> GPIO 22 (Blue wire)
 *
 * TROUBLESHOOTING:
 *   - White/pulsing screen: Wrong driver (must be ILI9488_DRIVER)
 *   - No display: Check backlight pin (GPIO 22)
 *   - Inverted colors: Uncomment TFT_INVERSION_ON or OFF
 *   - Rotated wrong: Adjust tft.setRotation() in code
 */

// ====================================================================================
// DISPLAY DRIVER - CRITICAL! Must match your display
// ====================================================================================

#define ILI9488_DRIVER     // 3.5" 480x320 display (ILI9488)

// ====================================================================================
// DISPLAY DIMENSIONS
// ====================================================================================

#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// ====================================================================================
// ESP32 DevKit 30-pin + ST7796 TFT PIN DEFINITIONS
// ====================================================================================

// Display uses HSPI (separate ST7796 module wired to ESP32 DevKit)
#define TFT_MISO  12    // SPI MISO (HSPI) - Purple wire
#define TFT_MOSI  13    // SPI MOSI (HSPI) - White wire
#define TFT_SCLK  14    // SPI Clock (HSPI) - Green wire
#define TFT_CS    15    // Chip select - Red wire
#define TFT_DC     2    // Data/Command - Yellow wire
#define TFT_RST    4    // Reset - Orange wire

// Backlight control
#define TFT_BL    22    // Backlight - Blue wire
#define TFT_BACKLIGHT_ON HIGH  // HIGH to turn on backlight

// ====================================================================================
// SPI CONFIGURATION
// ====================================================================================

// Use HSPI port (display)
// VSPI is left free for CC1101 radio module
#define USE_HSPI_PORT

// SPI frequency - 40MHz is stable for this display
#define SPI_FREQUENCY       40000000

// Read frequency (lower for stability)
#define SPI_READ_FREQUENCY  20000000

// ====================================================================================
// TOUCH CONFIGURATION
// ====================================================================================

// Touch is DISABLED - the XPT2046 touch controller on the display module
// was found to have a hardware defect (T_DO not connected internally).
// Use BOOT button (GPIO 0) to cycle display modes instead.

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
#define TFT_RGB_ORDER TFT_BGR  // Color order for ESP32-3248S035C

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
