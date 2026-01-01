/*
 * Display Rendering
 *
 * Handles all display output for TFT (ILI9488 480x320) and OLED (SSD1306 128x64).
 * Provides three display modes: Scatter Plot, Sensor List, and Statistics.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "tpms_decoders.h"

// Display options - uncomment ONE in main .ino file:
// #define USE_OLED_DISPLAY   // 0.96" I2C OLED (SSD1306 128x64)
// #define USE_TFT_DISPLAY    // 3.5" SPI TFT (ILI9488 480x320)

#ifdef USE_TFT_DISPLAY
#include <TFT_eSPI.h>
#endif

#ifdef USE_OLED_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

// ============================================================================
// Display Configuration
// ============================================================================

// OLED Display pins (I2C)
#define OLED_SDA     4
#define OLED_SCL     22
#define OLED_RESET   -1
#define OLED_ADDRESS 0x3C

// TFT Display pins (HSPI)
#define TFT_BL_PIN   22    // Backlight (GPIO 22)

// Screen dimensions
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320
#define HEADER_HEIGHT 40
#define SENSOR_ROW_HEIGHT 55
#define MAX_VISIBLE_SENSORS 5

// Colors (RGB565)
#define COLOR_BG            0x0000    // Black
#define COLOR_HEADER_BG     0x000F    // Dark blue
#define COLOR_TEXT          0xFFFF    // White
#define COLOR_PRESSURE_OK   0x07E0    // Green - normal pressure
#define COLOR_PRESSURE_LOW  0xF800    // Red - low pressure
#define COLOR_PRESSURE_HIGH 0xFFE0    // Yellow - high pressure
#define COLOR_TEMP_NORMAL   0x07E0    // Green
#define COLOR_TEMP_HOT      0xF800    // Red
#define COLOR_RSSI_HIGH     0x07E0    // Green - strong signal
#define COLOR_RSSI_MED      0xFFE0    // Yellow - medium signal
#define COLOR_RSSI_LOW      0xF800    // Red - weak signal
#define COLOR_DIVIDER       0x3186    // Dark gray
#define COLOR_FREQ_315      0x07FF    // Cyan - 315 MHz
#define COLOR_NEW_SENSOR    0xFFE0    // Yellow - newly detected

// Display modes
#define DISPLAY_MODE_SCATTER  0   // PSI scatter plot (default)
#define DISPLAY_MODE_SENSORS  1   // Sensor list with details
#define DISPLAY_MODE_STATS    2   // Statistics screen
#define DISPLAY_MODE_COUNT    3   // Total number of modes

// ============================================================================
// Display Manager Class
// ============================================================================

class DisplayManager {
public:
#ifdef USE_TFT_DISPLAY
  DisplayManager() : displayMode(DISPLAY_MODE_SCATTER), scrollOffset(0),
                     lastSensorCount(-1), needsFullRedraw(true) {}
#elif defined(USE_OLED_DISPLAY)
  DisplayManager() : oled(128, 64, &Wire, OLED_RESET),
                     displayMode(DISPLAY_MODE_SCATTER), scrollOffset(0),
                     lastSensorCount(-1), needsFullRedraw(true) {}
#else
  DisplayManager() : displayMode(DISPLAY_MODE_SCATTER), scrollOffset(0),
                     lastSensorCount(-1), needsFullRedraw(true) {}
#endif

  // Initialize display hardware
  void init() {
#ifdef USE_TFT_DISPLAY
    // Initialize backlight pin
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);  // Turn on backlight

    tft.init();
    tft.setRotation(1);  // Landscape (480x320)
    tft.fillScreen(COLOR_BG);
    Serial.println("TFT Display initialized (480x320) on HSPI");
    Serial.println("TFT pins: MISO=12, MOSI=13, SCLK=14, CS=15, DC=2, RST=4, BL=22");
    Serial.println("Press BOOT button (GPIO 0) to cycle display modes");

#elif defined(USE_OLED_DISPLAY)
    // Initialize I2C with custom pins
    Serial.printf("OLED pins: SDA=GPIO%d, SCL=GPIO%d\n", OLED_SDA, OLED_SCL);
    Wire.begin(OLED_SDA, OLED_SCL);

    // Scan for I2C devices
    Serial.println("Scanning I2C bus...");
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("  I2C device found at 0x%02X\n", addr);
      }
    }

    // Initialize OLED with Adafruit library
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
      Serial.println("ERROR: SSD1306 allocation failed!");
    } else {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(0, 0);
      oled.println("TPMS Scanner");
      oled.println("Initializing...");
      oled.display();
      Serial.println("OLED Display initialized (128x64)");
    }

#else
    Serial.println("Running in Serial-only mode (no display)");
#endif
  }

  // Show splash screen
  void showSplash() {
#ifdef USE_TFT_DISPLAY
    tft.fillScreen(COLOR_BG);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(3);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("TPMS Scanner", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - 40);
    tft.setTextSize(2);
    tft.drawString("CC1101 + ESP32", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    tft.setTextSize(1);
    tft.drawString("Initializing radio module...", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 40);

#elif defined(USE_OLED_DISPLAY)
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(10, 0);
    oled.println("TPMS");
    oled.setCursor(10, 18);
    oled.println("Scanner");
    oled.setTextSize(1);
    oled.setCursor(15, 40);
    oled.println("CC1101 + ESP32");
    oled.setCursor(15, 52);
    oled.println("Initializing...");
    oled.display();
#endif
  }

  // Show init success message
  void showInitSuccess() {
#ifdef USE_TFT_DISPLAY
    tft.setTextColor(COLOR_PRESSURE_OK);
    tft.drawString("CC1101 OK!", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 60);

#elif defined(USE_OLED_DISPLAY)
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(15, 10);
    oled.println("CC1101 OK!");
    oled.setTextSize(1);
    oled.setCursor(15, 40);
    oled.println("Starting scan...");
    oled.display();
#endif
  }

  // Show init failure message
  void showInitFailed() {
#ifdef USE_TFT_DISPLAY
    tft.setTextColor(COLOR_PRESSURE_LOW);
    tft.drawString("CC1101 INIT FAILED!", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 60);
    tft.drawString("Check wiring connections", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 80);

#elif defined(USE_OLED_DISPLAY)
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("CC1101 FAILED!");
    oled.println();
    oled.println("Check wiring:");
    oled.println("VCC=3.3V GND=GND");
    oled.println("CS=D5 SCK=D18");
    oled.println("MOSI=D23 MISO=D19");
    oled.display();
#endif
  }

  // Clear screen for main display
  void clearForMain() {
#ifdef USE_TFT_DISPLAY
    tft.fillScreen(COLOR_BG);
#endif
  }

  // Main draw function
  void draw(TPMSDecoder& decoder, unsigned long totalPackets, unsigned long startTime,
            uint16_t frequency, bool scanning, int8_t currentRSSI) {
#ifdef USE_TFT_DISPLAY
    // Only do full clear when sensor count changes or on first draw
    bool sensorCountChanged = (decoder.getSensorCount() != lastSensorCount);

    drawHeader(decoder.getSensorCount(), totalPackets, startTime, frequency, scanning);

    // Only clear sensor area when needed (reduces flicker)
    if (needsFullRedraw || sensorCountChanged) {
      tft.fillRect(0, HEADER_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - HEADER_HEIGHT, COLOR_BG);
      lastSensorCount = decoder.getSensorCount();
      needsFullRedraw = false;
    }

    // Draw the appropriate screen based on display mode
    switch (displayMode) {
      case DISPLAY_MODE_SCATTER:
        drawScatterPlotTFT(decoder);
        break;
      case DISPLAY_MODE_SENSORS:
        drawSensorListTFT(decoder);
        break;
      case DISPLAY_MODE_STATS:
        drawStatsScreenTFT(decoder, totalPackets, startTime, currentRSSI);
        break;
    }

#elif defined(USE_OLED_DISPLAY)
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    switch (displayMode) {
      case DISPLAY_MODE_SCATTER:
        drawScatterPlotOLED(decoder);
        break;
      case DISPLAY_MODE_SENSORS:
        drawSensorListOLED(decoder);
        break;
      case DISPLAY_MODE_STATS:
        drawStatsScreenOLED(decoder, totalPackets, startTime, currentRSSI);
        break;
    }

    oled.display();

#else
    // Serial-only mode: print status periodically
    static unsigned long lastSerialPrint = 0;
    if (millis() - lastSerialPrint > 2000) {
      lastSerialPrint = millis();
      Serial.printf("\n=== TPMS Scanner Status [%d MHz] ===\n", frequency);
      Serial.printf("Packets: %lu | Sensors: %d | Uptime: %s\n",
                    totalPackets, decoder.getSensorCount(),
                    formatTime(millis() - startTime).c_str());

      if (decoder.getSensorCount() > 0) {
        Serial.println("─────────────────────────────────────────────────────");
        TPMSSensor* sensors = decoder.getSensors();
        for (int i = 0; i < decoder.getSensorCount(); i++) {
          Serial.printf("ID: %08X | %.1f PSI (%.0f kPa) | %.0fC | %ddBm | %s\n",
                        sensors[i].id, sensors[i].pressure_psi, sensors[i].pressure_kpa,
                        sensors[i].temperature_c, sensors[i].rssi, sensors[i].protocol.c_str());
        }
      }
    }
#endif
  }

  // Cycle to next display mode
  void nextMode() {
    displayMode = (displayMode + 1) % DISPLAY_MODE_COUNT;
    needsFullRedraw = true;

    const char* modeNames[] = {"Scatter Plot", "Sensor List", "Statistics"};
    Serial.printf("[MODE] Display mode: %s\n", modeNames[displayMode]);
  }

  uint8_t getMode() const { return displayMode; }
  void forceRedraw() { needsFullRedraw = true; }

private:
#ifdef USE_TFT_DISPLAY
  TFT_eSPI tft;
#endif

#ifdef USE_OLED_DISPLAY
  Adafruit_SSD1306 oled;
#endif

  uint8_t displayMode;
  int scrollOffset;
  int lastSensorCount;
  bool needsFullRedraw;

  // ============================================================================
  // Helper Functions
  // ============================================================================

  String formatTime(unsigned long ms) {
    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;

    char buf[16];
    if (hours > 0) {
      sprintf(buf, "%luh%lum", hours, minutes % 60);
    } else if (minutes > 0) {
      sprintf(buf, "%lum%lus", minutes, seconds % 60);
    } else {
      sprintf(buf, "%lus", seconds);
    }
    return String(buf);
  }

  uint16_t getPressureColor(float psi) {
    if (psi < 25) return COLOR_PRESSURE_LOW;    // Too low
    if (psi > 38) return COLOR_PRESSURE_HIGH;   // Too high
    return COLOR_PRESSURE_OK;                    // Normal
  }

  uint16_t getTempColor(float temp) {
    if (temp > 60) return COLOR_TEMP_HOT;       // Hot tire
    return COLOR_TEMP_NORMAL;
  }

#ifdef USE_TFT_DISPLAY
  // ============================================================================
  // TFT Display Functions (480x320)
  // ============================================================================

  void drawHeader(int sensorCount, unsigned long totalPackets, unsigned long startTime,
                  uint16_t frequency, bool scanning) {
    // Header background
    tft.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, COLOR_HEADER_BG);

    // Title
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("TPMS Scanner", 10, HEADER_HEIGHT / 2 - 5);

    // Current frequency indicator
    uint16_t freqColor = COLOR_FREQ_315;  // 315 MHz only
    tft.setTextColor(freqColor);
    tft.setTextSize(1);
    char freqStr[20];
    sprintf(freqStr, "%d MHz", frequency);
    tft.drawString(freqStr, 170, HEADER_HEIGHT / 2 - 8);

    // Scanning indicator
    if (scanning) {
      static unsigned long lastBlink = 0;
      static bool blinkState = false;
      if (millis() - lastBlink > 500) {
        blinkState = !blinkState;
        lastBlink = millis();
      }
      if (blinkState) {
        tft.fillCircle(160, HEADER_HEIGHT / 2 + 5, 4, COLOR_PRESSURE_OK);
      }
    }

    // Sensor count
    tft.setTextColor(COLOR_TEXT);
    char countStr[20];
    sprintf(countStr, "Sensors: %d", sensorCount);
    tft.drawString(countStr, 170, HEADER_HEIGHT / 2 + 8);

    // Statistics
    tft.setTextDatum(MR_DATUM);
    char statsStr[30];
    sprintf(statsStr, "Pkts: %lu", totalPackets);
    tft.drawString(statsStr, SCREEN_WIDTH - 10, HEADER_HEIGHT / 2 - 8);

    // Uptime
    String uptime = formatTime(millis() - startTime);
    tft.drawString(uptime.c_str(), SCREEN_WIDTH - 10, HEADER_HEIGHT / 2 + 8);
  }

  void drawScatterPlotTFT(TPMSDecoder& decoder) {
    #define TFT_GRAPH_LEFT 40
    #define TFT_GRAPH_RIGHT 460
    #define TFT_GRAPH_TOP (HEADER_HEIGHT + 20)
    #define TFT_GRAPH_BOTTOM 280
    #define TFT_GRAPH_WIDTH (TFT_GRAPH_RIGHT - TFT_GRAPH_LEFT)
    #define TFT_GRAPH_HEIGHT (TFT_GRAPH_BOTTOM - TFT_GRAPH_TOP)

    // Stats at top
    tft.setTextColor(TFT_YELLOW, COLOR_BG);
    tft.setTextSize(3);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(String(decoder.getUniqueSensors()) + " sensors", 20, HEADER_HEIGHT + 5);

    tft.setTextDatum(TR_DATUM);
    tft.drawString(String(decoder.getValidPackets()) + " readings", SCREEN_WIDTH - 20, HEADER_HEIGHT + 5);

    // Draw axes
    tft.drawFastVLine(TFT_GRAPH_LEFT - 2, TFT_GRAPH_TOP, TFT_GRAPH_HEIGHT + 5, TFT_WHITE);
    tft.drawFastHLine(TFT_GRAPH_LEFT - 2, TFT_GRAPH_BOTTOM + 2, TFT_GRAPH_WIDTH + 5, TFT_WHITE);

    // X-axis labels (PSI)
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(String(PSI_MIN), TFT_GRAPH_LEFT, TFT_GRAPH_BOTTOM + 10);
    tft.drawString(String((PSI_MIN + PSI_MAX) / 2), TFT_GRAPH_LEFT + TFT_GRAPH_WIDTH / 2, TFT_GRAPH_BOTTOM + 10);
    tft.drawString(String(PSI_MAX), TFT_GRAPH_RIGHT, TFT_GRAPH_BOTTOM + 10);
    tft.drawString("PSI", TFT_GRAPH_LEFT + TFT_GRAPH_WIDTH / 2, TFT_GRAPH_BOTTOM + 30);

    // Draw scatter dots from histogram
    uint8_t* histogram = decoder.getHistogram();
    for (int bin = 0; bin < PSI_BINS; bin++) {
      if (histogram[bin] > 0) {
        int x = TFT_GRAPH_LEFT + (bin * TFT_GRAPH_WIDTH) / (PSI_BINS - 1);
        int maxDots = TFT_GRAPH_HEIGHT / 8;
        int count = min((int)histogram[bin], maxDots);

        int psi = PSI_MIN + bin;
        uint16_t color = TFT_GREEN;
        if (psi < 28 || psi > 38) color = TFT_YELLOW;
        if (psi < 25 || psi > 42) color = TFT_RED;

        for (int d = 0; d < count; d++) {
          int y = TFT_GRAPH_BOTTOM - 4 - (d * 8);
          if (y >= TFT_GRAPH_TOP) {
            tft.fillRect(x - 2, y, 5, 5, color);
          }
        }
      }
    }

    // If no data, show waiting message
    if (decoder.getValidPackets() == 0) {
      tft.setTextSize(2);
      tft.setTextColor(TFT_DARKGREY, COLOR_BG);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Waiting for TPMS data...", SCREEN_WIDTH / 2, TFT_GRAPH_TOP + TFT_GRAPH_HEIGHT / 2);
    }
  }

  void drawSensorListTFT(TPMSDecoder& decoder) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, COLOR_BG);

    if (decoder.getSensorCount() == 0) {
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Scanning for TPMS sensors...", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
      tft.setTextSize(1);
      tft.drawString("315 MHz (North America)", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 30);

      static int scanAnim = 0;
      scanAnim = (scanAnim + 1) % 4;
      String dots = "";
      for (int i = 0; i <= scanAnim; i++) dots += ".";
      tft.setTextSize(2);
      tft.fillRect(SCREEN_WIDTH / 2 - 30, SCREEN_HEIGHT / 2 + 50, 60, 20, COLOR_BG);
      tft.drawString(dots, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 55);
      return;
    }

    // Draw column headers
    int yPos = HEADER_HEIGHT + 5;
    tft.setTextSize(1);
    tft.setTextColor(TFT_CYAN, COLOR_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("SENSOR ID", 10, yPos);
    tft.drawString("PRESSURE", 150, yPos);
    tft.drawString("TEMP", 280, yPos);
    tft.drawString("SIGNAL", 360, yPos);
    tft.drawString("AGE", 430, yPos);
    yPos += 15;
    tft.drawFastHLine(10, yPos, SCREEN_WIDTH - 20, COLOR_DIVIDER);
    yPos += 5;

    // Draw sensor rows
    TPMSSensor* sensors = decoder.getSensors();
    int visibleCount = min(decoder.getSensorCount() - scrollOffset, MAX_VISIBLE_SENSORS);
    for (int i = 0; i < visibleCount; i++) {
      int sensorIdx = i + scrollOffset;
      drawSensorRow(sensors[sensorIdx], yPos);
      yPos += SENSOR_ROW_HEIGHT;

      if (i < visibleCount - 1) {
        tft.drawFastHLine(10, yPos - 2, SCREEN_WIDTH - 20, COLOR_DIVIDER);
      }
    }

    // Scroll indicators
    if (scrollOffset > 0) {
      tft.fillTriangle(SCREEN_WIDTH - 20, HEADER_HEIGHT + 25,
                       SCREEN_WIDTH - 10, HEADER_HEIGHT + 25,
                       SCREEN_WIDTH - 15, HEADER_HEIGHT + 15, TFT_WHITE);
    }
    if (scrollOffset + MAX_VISIBLE_SENSORS < decoder.getSensorCount()) {
      tft.fillTriangle(SCREEN_WIDTH - 20, SCREEN_HEIGHT - 15,
                       SCREEN_WIDTH - 10, SCREEN_HEIGHT - 15,
                       SCREEN_WIDTH - 15, SCREEN_HEIGHT - 5, TFT_WHITE);
    }
  }

  void drawSensorRow(TPMSSensor& sensor, int yPos) {
    char buf[32];

    // New sensor indicator
    if (sensor.isNew) {
      tft.fillCircle(12, yPos + SENSOR_ROW_HEIGHT / 2, 6, COLOR_NEW_SENSOR);
    }

    // Frequency indicator
    uint16_t freqColor = COLOR_FREQ_315;
    tft.fillRect(22, yPos + 5, 4, SENSOR_ROW_HEIGHT - 10, freqColor);

    // Sensor ID (large)
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    sprintf(buf, "ID: %08X", sensor.id);
    tft.drawString(buf, 35, yPos + 5);

    // Protocol
    tft.setTextSize(1);
    tft.setTextColor(COLOR_DIVIDER);
    tft.drawString(sensor.protocol.c_str(), 35, yPos + 25);

    // Pressure display (prominent)
    uint16_t pressColor = getPressureColor(sensor.pressure_psi);
    tft.setTextColor(pressColor);
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    sprintf(buf, "%.1f PSI", sensor.pressure_psi);
    tft.drawString(buf, 200, yPos + 5);

    // Pressure bar
    drawPressureBar(200, yPos + 30, sensor.pressure_psi);

    // kPa (smaller)
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    sprintf(buf, "(%.0f kPa)", sensor.pressure_kpa);
    tft.drawString(buf, 200, yPos + 42);

    // Temperature
    uint16_t tempColor = getTempColor(sensor.temperature_c);
    tft.setTextColor(tempColor);
    tft.setTextSize(2);
    sprintf(buf, "%.0fC", sensor.temperature_c);
    tft.drawString(buf, 320, yPos + 5);

    // Convert to Fahrenheit
    tft.setTextSize(1);
    float tempF = sensor.temperature_c * 9.0 / 5.0 + 32.0;
    sprintf(buf, "(%.0fF)", tempF);
    tft.drawString(buf, 320, yPos + 25);

    // RSSI bars
    drawRSSIBars(380, yPos + 8, sensor.rssi);

    // RSSI value
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);
    sprintf(buf, "%ddBm", sensor.rssi);
    tft.drawString(buf, 380, yPos + 35);

    // Last seen time
    unsigned long age = millis() - sensor.lastSeen;
    String ageStr = formatTime(age) + " ago";
    tft.setTextColor(COLOR_DIVIDER);
    tft.drawString(ageStr.c_str(), 430, yPos + 45);

    // Detection count
    sprintf(buf, "x%lu", sensor.detectionCount);
    tft.drawString(buf, 430, yPos + 8);

    // Battery indicator (if available)
    if (sensor.battery_low) {
      tft.setTextColor(COLOR_PRESSURE_LOW);
      tft.drawString("BAT LOW", 430, yPos + 20);
    }
  }

  void drawRSSIBars(int x, int y, int8_t rssi) {
    int bars = 0;
    if (rssi >= -50) bars = 5;
    else if (rssi >= -60) bars = 4;
    else if (rssi >= -70) bars = 3;
    else if (rssi >= -80) bars = 2;
    else if (rssi >= -90) bars = 1;

    int barWidth = 6;
    int barGap = 2;
    int maxHeight = 20;

    for (int i = 0; i < 5; i++) {
      int barHeight = (i + 1) * 4;
      int barX = x + i * (barWidth + barGap);
      int barY = y + (maxHeight - barHeight);

      uint16_t color;
      if (i < bars) {
        if (bars >= 4) color = COLOR_RSSI_HIGH;
        else if (bars >= 2) color = COLOR_RSSI_MED;
        else color = COLOR_RSSI_LOW;
      } else {
        color = COLOR_DIVIDER;
      }

      tft.fillRect(barX, barY, barWidth, barHeight, color);
    }
  }

  void drawPressureBar(int x, int y, float pressure) {
    int barWidth = 80;
    int barHeight = 8;

    tft.drawRect(x, y, barWidth, barHeight, COLOR_TEXT);

    float normalized = constrain((pressure - 20) / 20.0, 0, 1);
    int fillWidth = normalized * (barWidth - 2);

    uint16_t fillColor = getPressureColor(pressure);

    if (fillWidth > 0) {
      tft.fillRect(x + 1, y + 1, fillWidth, barHeight - 2, fillColor);
    }

    int targetX = x + (int)((32 - 20) / 20.0 * barWidth);
    tft.drawLine(targetX, y - 2, targetX, y + barHeight + 2, COLOR_TEXT);
  }

  void drawStatsScreenTFT(TPMSDecoder& decoder, unsigned long totalPackets,
                          unsigned long startTime, int8_t currentRSSI) {
    int yPos = HEADER_HEIGHT + 20;
    int lineHeight = 35;

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextDatum(TL_DATUM);

    // Title
    tft.setTextColor(TFT_CYAN, COLOR_BG);
    tft.setTextSize(3);
    tft.drawString("STATISTICS", SCREEN_WIDTH / 2 - 80, yPos);
    yPos += lineHeight + 10;

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, COLOR_BG);

    // Uptime
    tft.drawString("Uptime:", 30, yPos);
    tft.setTextColor(TFT_GREEN, COLOR_BG);
    tft.drawString(formatTime(millis() - startTime), 200, yPos);
    yPos += lineHeight;

    // Total packets
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Total Packets:", 30, yPos);
    tft.setTextColor(TFT_YELLOW, COLOR_BG);
    tft.drawString(String(totalPackets), 200, yPos);
    yPos += lineHeight;

    // Valid readings
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Valid Readings:", 30, yPos);
    tft.setTextColor(TFT_GREEN, COLOR_BG);
    tft.drawString(String(decoder.getValidPackets()), 200, yPos);
    yPos += lineHeight;

    // Unique sensors
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Unique Sensors:", 30, yPos);
    tft.setTextColor(TFT_CYAN, COLOR_BG);
    tft.drawString(String(decoder.getUniqueSensors()), 200, yPos);
    yPos += lineHeight;

    // Active sensors
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Active Sensors:", 30, yPos);
    tft.setTextColor(TFT_GREEN, COLOR_BG);
    tft.drawString(String(decoder.getSensorCount()), 200, yPos);
    yPos += lineHeight;

    // Current RSSI
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Current RSSI:", 30, yPos);
    if (currentRSSI > -80) tft.setTextColor(TFT_GREEN, COLOR_BG);
    else if (currentRSSI > -95) tft.setTextColor(TFT_YELLOW, COLOR_BG);
    else tft.setTextColor(TFT_RED, COLOR_BG);
    tft.drawString(String(currentRSSI) + " dBm", 200, yPos);
  }
#endif  // USE_TFT_DISPLAY

#ifdef USE_OLED_DISPLAY
  // ============================================================================
  // OLED Display Functions (128x64)
  // ============================================================================

  void drawScatterPlotOLED(TPMSDecoder& decoder) {
    // Stats in yellow zone
    oled.setCursor(2, 0);
    oled.setTextSize(2);
    oled.print(decoder.getUniqueSensors());

    oled.setTextSize(2);
    int digits = (decoder.getValidPackets() >= 1000) ? 4 :
                 (decoder.getValidPackets() >= 100) ? 3 :
                 (decoder.getValidPackets() >= 10) ? 2 : 1;
    int xPos = 128 - (digits * 12) - 2;
    oled.setCursor(xPos, 0);
    oled.print(decoder.getValidPackets());

    #define GRAPH_LEFT 8
    #define GRAPH_RIGHT 120
    #define GRAPH_TOP 18
    #define GRAPH_BOTTOM 54
    #define GRAPH_WIDTH (GRAPH_RIGHT - GRAPH_LEFT)
    #define GRAPH_HEIGHT (GRAPH_BOTTOM - GRAPH_TOP)

    // Draw axes
    oled.drawFastVLine(GRAPH_LEFT - 1, GRAPH_TOP, GRAPH_HEIGHT + 2, SSD1306_WHITE);
    oled.drawFastHLine(GRAPH_LEFT - 1, GRAPH_BOTTOM + 1, GRAPH_WIDTH + 2, SSD1306_WHITE);

    // X-axis labels
    oled.setTextSize(1);
    oled.setCursor(GRAPH_LEFT - 2, 56);
    oled.print(PSI_MIN);
    oled.setCursor(GRAPH_LEFT + GRAPH_WIDTH/2 - 6, 56);
    oled.print((PSI_MIN + PSI_MAX) / 2);
    oled.setCursor(GRAPH_RIGHT - 8, 56);
    oled.print(PSI_MAX);

    // Draw scatter dots
    uint8_t* histogram = decoder.getHistogram();
    for (int bin = 0; bin < PSI_BINS; bin++) {
      if (histogram[bin] > 0) {
        int x = GRAPH_LEFT + (bin * GRAPH_WIDTH) / (PSI_BINS - 1);
        int count = min((int)histogram[bin], GRAPH_HEIGHT / 3);
        for (int d = 0; d < count; d++) {
          int y = GRAPH_BOTTOM - 2 - (d * 3);
          if (y >= GRAPH_TOP) {
            oled.fillRect(x, y, 2, 2, SSD1306_WHITE);
          }
        }
      }
    }

    // Waiting indicator
    if (decoder.getValidPackets() == 0) {
      static int waitAnim = 0;
      waitAnim = (waitAnim + 1) % 4;
      int cx = GRAPH_LEFT + GRAPH_WIDTH/2;
      int cy = GRAPH_TOP + GRAPH_HEIGHT/2;
      oled.drawCircle(cx, cy, 4 + waitAnim, SSD1306_WHITE);
    }
  }

  void drawSensorListOLED(TPMSDecoder& decoder) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("SENSORS (");
    oled.print(decoder.getSensorCount());
    oled.print(")");

    oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);

    if (decoder.getSensorCount() == 0) {
      oled.setCursor(20, 28);
      oled.print("No sensors");
      return;
    }

    TPMSSensor* sensors = decoder.getSensors();
    int startIdx = max(0, decoder.getSensorCount() - 4);
    int y = 16;

    for (int i = decoder.getSensorCount() - 1; i >= startIdx && y < 64; i--) {
      TPMSSensor& s = sensors[i];

      oled.setCursor(0, y);
      oled.printf("%06X", s.id & 0xFFFFFF);

      oled.setCursor(44, y);
      oled.printf("%2.0f", s.pressure_psi);

      float tempF = s.temperature_c * 9.0 / 5.0 + 32.0;
      oled.setCursor(68, y);
      oled.printf("%3.0fF", tempF);

      oled.setCursor(100, y);
      if (s.rssi >= -70) oled.print("+++");
      else if (s.rssi >= -85) oled.print("++ ");
      else if (s.rssi >= -100) oled.print("+  ");
      else oled.print("-  ");

      if (s.isNew) {
        oled.setCursor(120, y);
        oled.print("*");
      }

      y += 12;
    }
  }

  void drawStatsScreenOLED(TPMSDecoder& decoder, unsigned long totalPackets,
                           unsigned long startTime, int8_t currentRSSI) {
    oled.setTextSize(1);

    oled.setCursor(30, 0);
    oled.print("STATISTICS");
    oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);

    oled.setCursor(0, 16);
    oled.print("Uptime: ");
    oled.print(formatTime(millis() - startTime));

    oled.setCursor(0, 26);
    oled.print("Packets: ");
    oled.print(totalPackets);

    oled.setCursor(0, 36);
    oled.print("Valid:   ");
    oled.print(decoder.getValidPackets());

    oled.setCursor(0, 46);
    oled.print("Sensors: ");
    oled.print(decoder.getUniqueSensors());
    oled.print(" (");
    oled.print(decoder.getSensorCount());
    oled.print(" active)");

    oled.setCursor(0, 56);
    oled.print("RSSI: ");
    oled.print(currentRSSI);
    oled.print(" dBm");
  }
#endif  // USE_OLED_DISPLAY
};

#endif // DISPLAY_H
