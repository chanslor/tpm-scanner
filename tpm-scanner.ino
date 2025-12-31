/*
 * TPMS Scanner - Tire Pressure Monitoring System Reader
 *
 * Scans for TPMS (Tire Pressure Monitoring System) sensor transmissions
 * on 315 MHz using a CC1101 wireless module (North America).
 *
 * Hardware:
 *   ESP32 DevKit (30-pin)
 *   CC1101 Wireless Module with SMA Antenna (315 MHz)
 *   ILI9488 TFT Display (3.5" 480x320 SPI) - or 0.96" OLED (see below)
 *
 * Features:
 *   - 315 MHz scanning (North America - USA, Canada, Mexico)
 *   - Real-time display of detected TPMS sensors
 *   - Sensor ID, pressure, temperature decoding
 *   - Signal strength (RSSI) display
 *   - Detection history and logging
 *
 * CC1101 Wiring (VSPI):
 *   CC1101 Pin 1 (GND)   -> GND
 *   CC1101 Pin 2 (VCC)   -> 3.3V
 *   CC1101 Pin 3 (GDO0)  -> GPIO 21 (interrupt)
 *   CC1101 Pin 4 (CSN)   -> GPIO 5  (chip select)
 *   CC1101 Pin 5 (SCK)   -> GPIO 18 (VSPI CLK)
 *   CC1101 Pin 6 (MOSI)  -> GPIO 23 (VSPI MOSI)
 *   CC1101 Pin 7 (MISO)  -> GPIO 19 (VSPI MISO)
 *   CC1101 Pin 8 (GDO2)  -> Not connected
 *
 * ILI9488 TFT Wiring (HSPI - separate from CC1101):
 *   TFT_SCLK  -> GPIO 14 (HSPI CLK)  - Green wire
 *   TFT_MISO  -> GPIO 12 (HSPI MISO) - Purple wire
 *   TFT_MOSI  -> GPIO 13 (HSPI MOSI) - White wire
 *   TFT_CS    -> GPIO 15 (chip select) - Red wire
 *   TFT_DC    -> GPIO 2  (data/command) - Yellow wire
 *   TFT_RST   -> GPIO 4  (reset) - Orange wire
 *   TFT_BL    -> GPIO 22 (backlight) - Blue wire
 *
 * OLED Wiring (I2C - alternative to TFT):
 *   VCC -> 3.3V, GND -> GND, SDA -> GPIO 4, SCL -> GPIO 22
 *   To use OLED: uncomment USE_OLED_DISPLAY, comment USE_TFT_DISPLAY
 *
 * Author: TPMS Scanner Project
 * Date: December 2024
 */

#include <SPI.h>
#include <Wire.h>

// Display options - uncomment ONE:
// #define USE_OLED_DISPLAY   // 0.96" I2C OLED (SSD1306 128x64)
#define USE_TFT_DISPLAY       // 3.5" SPI TFT (ST7796 480x320)

#ifdef USE_TFT_DISPLAY
#include <TFT_eSPI.h>
#endif

#ifdef USE_OLED_DISPLAY
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

// OLED Display pins (I2C) - only used if USE_OLED_DISPLAY is defined
#define OLED_SDA  4   // D4 (GPIO 4)
#define OLED_SCL  22  // D22 (GPIO 22) - standard ESP32 I2C SCL
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// TFT Display pins (HSPI) - only used if USE_TFT_DISPLAY is defined
// These match User_Setup.h - wired to ILI9488 3.5" 480x320 module
#define TFT_BL_PIN    22    // Backlight (GPIO 22 - Blue wire)
// Other TFT pins defined in User_Setup.h:
//   TFT_MISO  = 12 (HSPI) - Purple wire
//   TFT_MOSI  = 13 (HSPI) - White wire
//   TFT_SCLK  = 14 (HSPI) - Green wire
//   TFT_CS    = 15 - Red wire
//   TFT_DC    = 2 - Yellow wire
//   TFT_RST   = 4 - Orange wire

// ============================================================================
// Pin Definitions - CC1101 Module
// ============================================================================

// CC1101 connections for standard ESP32 DevKit (30-pin):
//   All pins directly accessible on pin headers - no soldering needed!
//
// Using VSPI bus (directly on headers):
//   IO18 = SCK, IO19 = MISO, IO23 = MOSI, IO5 = CS
//   IO21 = GDO0 (interrupt)

#define CC1101_CS     5     // Chip Select (directly on header)
#define CC1101_GDO0   21    // Interrupt (directly on header)
#define CC1101_SCK    18    // SPI Clock (VSPI default)
#define CC1101_MOSI   23    // SPI MOSI (VSPI default)
#define CC1101_MISO   19    // SPI MISO (VSPI default)

// ============================================================================
// Button Configuration
// ============================================================================

#define BOOT_BUTTON   0     // GPIO 0 - BOOT button (directly on ESP32 DevKit)

// Display modes (cycle with BOOT button)
#define DISPLAY_MODE_SCATTER  0   // PSI scatter plot (default)
#define DISPLAY_MODE_SENSORS  1   // Sensor list with details
#define DISPLAY_MODE_STATS    2   // Statistics screen
#define DISPLAY_MODE_COUNT    3   // Total number of modes

// ============================================================================
// Display Configuration
// ============================================================================

#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320
#define HEADER_HEIGHT 40
#define SENSOR_ROW_HEIGHT 55
#define MAX_VISIBLE_SENSORS 5
#define MAX_TRACKED_SENSORS 20

// Colors (RGB565)
#define COLOR_BG           0x0000    // Black
#define COLOR_HEADER_BG    0x000F    // Dark blue
#define COLOR_TEXT         0xFFFF    // White
#define COLOR_PRESSURE_OK  0x07E0    // Green - normal pressure
#define COLOR_PRESSURE_LOW 0xF800    // Red - low pressure
#define COLOR_PRESSURE_HIGH 0xFFE0   // Yellow - high pressure
#define COLOR_TEMP_NORMAL  0x07E0    // Green
#define COLOR_TEMP_HOT     0xF800    // Red
#define COLOR_RSSI_HIGH    0x07E0    // Green - strong signal
#define COLOR_RSSI_MED     0xFFE0    // Yellow - medium signal
#define COLOR_RSSI_LOW     0xF800    // Red - weak signal
#define COLOR_DIVIDER      0x3186    // Dark gray
#define COLOR_FREQ_315     0x07FF    // Cyan - 315 MHz
#define COLOR_FREQ_433     0xF81F    // Magenta - 433 MHz
#define COLOR_NEW_SENSOR   0xFFE0    // Yellow - newly detected

// ============================================================================
// CC1101 Register Definitions
// ============================================================================

// Configuration registers
#define CC1101_IOCFG2      0x00
#define CC1101_IOCFG1      0x01
#define CC1101_IOCFG0      0x02
#define CC1101_FIFOTHR     0x03
#define CC1101_SYNC1       0x04
#define CC1101_SYNC0       0x05
#define CC1101_PKTLEN      0x06
#define CC1101_PKTCTRL1    0x07
#define CC1101_PKTCTRL0    0x08
#define CC1101_ADDR        0x09
#define CC1101_CHANNR      0x0A
#define CC1101_FSCTRL1     0x0B
#define CC1101_FSCTRL0     0x0C
#define CC1101_FREQ2       0x0D
#define CC1101_FREQ1       0x0E
#define CC1101_FREQ0       0x0F
#define CC1101_MDMCFG4     0x10
#define CC1101_MDMCFG3     0x11
#define CC1101_MDMCFG2     0x12
#define CC1101_MDMCFG1     0x13
#define CC1101_MDMCFG0     0x14
#define CC1101_DEVIATN     0x15
#define CC1101_MCSM2       0x16
#define CC1101_MCSM1       0x17
#define CC1101_MCSM0       0x18
#define CC1101_FOCCFG      0x19
#define CC1101_BSCFG       0x1A
#define CC1101_AGCCTRL2    0x1B
#define CC1101_AGCCTRL1    0x1C
#define CC1101_AGCCTRL0    0x1D
#define CC1101_WOREVT1     0x1E
#define CC1101_WOREVT0     0x1F
#define CC1101_WORCTRL     0x20
#define CC1101_FREND1      0x21
#define CC1101_FREND0      0x22
#define CC1101_FSCAL3      0x23
#define CC1101_FSCAL2      0x24
#define CC1101_FSCAL1      0x25
#define CC1101_FSCAL0      0x26
#define CC1101_RCCTRL1     0x27
#define CC1101_RCCTRL0     0x28
#define CC1101_FSTEST      0x29
#define CC1101_PTEST       0x2A
#define CC1101_AGCTEST     0x2B
#define CC1101_TEST2       0x2C
#define CC1101_TEST1       0x2D
#define CC1101_TEST0       0x2E

// Command strobes
#define CC1101_SRES        0x30
#define CC1101_SFSTXON     0x31
#define CC1101_SXOFF       0x32
#define CC1101_SCAL        0x33
#define CC1101_SRX         0x34
#define CC1101_STX         0x35
#define CC1101_SIDLE       0x36
#define CC1101_SWOR        0x38
#define CC1101_SPWD        0x39
#define CC1101_SFRX        0x3A
#define CC1101_SFTX        0x3B
#define CC1101_SWORRST     0x3C
#define CC1101_SNOP        0x3D

// Status registers (burst read)
#define CC1101_PARTNUM     0x30
#define CC1101_VERSION     0x31
#define CC1101_FREQEST     0x32
#define CC1101_LQI         0x33
#define CC1101_RSSI        0x34
#define CC1101_MARCSTATE   0x35
#define CC1101_WORTIME1    0x36
#define CC1101_WORTIME0    0x37
#define CC1101_PKTSTATUS   0x38
#define CC1101_VCO_VC_DAC  0x39
#define CC1101_TXBYTES     0x3A
#define CC1101_RXBYTES     0x3B

// FIFO access
#define CC1101_TXFIFO      0x3F
#define CC1101_RXFIFO      0x3F

// Read/Write flags
#define CC1101_WRITE_BURST 0x40
#define CC1101_READ_SINGLE 0x80
#define CC1101_READ_BURST  0xC0

// ============================================================================
// TPMS Data Structures
// ============================================================================

struct TPMSSensor {
  uint32_t id;                    // Sensor ID (unique per tire)
  float pressure_psi;             // Tire pressure in PSI
  float pressure_kpa;             // Tire pressure in kPa
  float temperature_c;            // Temperature in Celsius
  int8_t rssi;                    // Signal strength in dBm
  uint8_t battery_pct;            // Battery percentage (if available)
  bool battery_low;               // Battery low flag
  uint16_t frequency;             // Detected frequency (315 or 433)
  unsigned long firstSeen;        // First detection timestamp
  unsigned long lastSeen;         // Last detection timestamp
  uint32_t detectionCount;        // Number of times detected
  bool isNew;                     // Recently detected flag
  String protocol;                // Detected protocol type
};

// ============================================================================
// Global Variables
// ============================================================================

// Display (optional)
#ifdef USE_TFT_DISPLAY
TFT_eSPI tft = TFT_eSPI();
#endif

#ifdef USE_OLED_DISPLAY
// SSD1306 128x64 OLED via I2C (Adafruit library)
Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RESET);
#endif

// SPI for CC1101 (VSPI)
SPIClass cc1101SPI(VSPI);

// Sensor tracking
TPMSSensor sensors[MAX_TRACKED_SENSORS];
int sensorCount = 0;
int scrollOffset = 0;

// Scanning state
volatile bool packetReceived = false;
bool scanning = true;
const uint16_t FREQUENCY = 315;  // 315 MHz for North America (USA, Canada, Mexico)

// Statistics
unsigned long totalPackets = 0;
unsigned long validPackets = 0;
unsigned long startTime = 0;
unsigned long lastDisplayUpdate = 0;

// Pressure histogram for scatter plot
#define PSI_MIN 20
#define PSI_MAX 60
#define PSI_BINS (PSI_MAX - PSI_MIN + 1)  // 41 bins (20-60 PSI)
uint8_t psiHistogram[PSI_BINS];  // Count of readings at each PSI
uint16_t uniqueSensors = 0;      // Count of unique sensor IDs seen

// Timing
#define DISPLAY_UPDATE_MS 500  // Slower updates to reduce flicker
#define SENSOR_TIMEOUT_MS 300000   // 5 minutes timeout
#define NEW_SENSOR_THRESHOLD 30000 // 30 seconds "new" indicator

// Histogram decay settings (prevents bars from maxing out over long runs)
#define HISTOGRAM_DECAY_INTERVAL_MS 300000 // Decay every 5 minutes
#define HISTOGRAM_DECAY_PERCENT 3          // Reduce by 3% each interval
#define HISTOGRAM_MIN_VALUE 1              // Floor value (keeps "ghost" of past readings)

// Display mode (cycle with BOOT button)
uint8_t displayMode = DISPLAY_MODE_SCATTER;
unsigned long lastButtonPress = 0;
bool lastButtonState = HIGH;  // Button is active LOW
#define BUTTON_DEBOUNCE_MS 250

// Histogram decay timer
unsigned long lastHistogramDecay = 0;

// ============================================================================
// SPI Timeout Helper
// ============================================================================

// Wait for MISO to go low with timeout (returns true if ready, false if timeout)
#define SPI_TIMEOUT_MS 100

inline bool waitMISO() {
  uint32_t timeout = millis() + SPI_TIMEOUT_MS;
  while (digitalRead(CC1101_MISO)) {
    if (millis() >= timeout) {
      Serial.println("[ERROR] SPI timeout waiting for MISO");
      return false;
    }
  }
  return true;
}

// ============================================================================
// CC1101 Configuration for TPMS
// ============================================================================

// Configuration for 315 MHz TPMS reception (2-FSK) - North America
// Based on working TPMS receiver projects (andi38/TPMS, etc.)
// Toyota PMV-107J compatible settings:
//   - 315 MHz carrier
//   - 2-FSK modulation (NOT OOK!)
//   - 19.2 kbps data rate
//   - 19 kHz deviation
//   - 135 kHz bandwidth
//   - Sync word 0x001A

const uint8_t CC1101_CONFIG_315[] = {
  0x06,  // IOCFG2   - GDO2: Asserts when sync word received
  0x2E,  // IOCFG1   - High-Z (not used)
  0x06,  // IOCFG0   - GDO0: Asserts when sync word received, deasserts at end of packet
  0x47,  // FIFOTHR  - RX FIFO threshold = 33 bytes, ADC retention
  0x00,  // SYNC1    - Sync word high byte (0x001A)
  0x1A,  // SYNC0    - Sync word low byte - TPMS preamble pattern!
  0xFF,  // PKTLEN   - Max packet length 255 bytes (variable length mode)
  0x00,  // PKTCTRL1 - No address check, no status append
  0x00,  // PKTCTRL0 - Fixed packet length, no CRC, no whitening
  0x00,  // ADDR     - Device address (not used)
  0x00,  // CHANNR   - Channel 0
  0x06,  // FSCTRL1  - IF frequency 152.3 kHz
  0x00,  // FSCTRL0  - Frequency offset
  0x0C,  // FREQ2    - 315 MHz (0x0C1D89 = 315000000 Hz)
  0x1D,  // FREQ1
  0x89,  // FREQ0
  0x8A,  // MDMCFG4  - Bandwidth 135 kHz, DRATE_E = 10
  0x83,  // MDMCFG3  - Data rate 19.2 kbps (DRATE_M = 131)
  0x02,  // MDMCFG2  - 2-FSK modulation, 16/16 sync word, no Manchester
  0x22,  // MDMCFG1  - 2 preamble bytes, no FEC
  0xF8,  // MDMCFG0  - Channel spacing
  0x44,  // DEVIATN  - FSK deviation 19.05 kHz
  0x07,  // MCSM2    - RX timeout
  0x30,  // MCSM1    - Stay in RX after packet, CCA always
  0x18,  // MCSM0    - Auto calibrate on idle to RX/TX
  0x16,  // FOCCFG   - Frequency offset compensation
  0x6C,  // BSCFG    - Bit synchronization
  0x43,  // AGCCTRL2 - AGC control (medium settings)
  0x40,  // AGCCTRL1 - Carrier sense relative threshold
  0x91,  // AGCCTRL0 - AGC control
  0x87,  // WOREVT1  - Wake on radio
  0x6B,  // WOREVT0
  0xFB,  // WORCTRL
  0x56,  // FREND1   - Front end RX config
  0x10,  // FREND0   - Front end TX config (for FSK)
  0xE9,  // FSCAL3   - Frequency synthesizer cal
  0x2A,  // FSCAL2
  0x00,  // FSCAL1
  0x1F,  // FSCAL0
  0x41,  // RCCTRL1  - RC oscillator
  0x00,  // RCCTRL0
};

// ============================================================================
// Function Prototypes
// ============================================================================

void initDisplay();
bool initCC1101();
void cc1101WriteReg(uint8_t addr, uint8_t value);
uint8_t cc1101ReadReg(uint8_t addr);
uint8_t cc1101ReadStatus(uint8_t addr);
void cc1101Strobe(uint8_t strobe);
void cc1101Reset();
void cc1101SetFrequency(uint16_t freqMHz);
void cc1101StartRX();
int8_t cc1101GetRSSI();
void processPacket();
void decodeTPMSPacket(uint8_t* data, uint8_t len, int8_t rssi);
int manchesterDecode(uint8_t* input, int inputLen, uint8_t* output, int maxOutput);
void tryDirectDecode(uint8_t* data, int len, int8_t rssi);
void updateSensor(uint32_t id, float pressure, float temp, int8_t rssi, uint16_t freq, const char* protocol);
void pruneOldSensors();
void decayHistogram();
void drawDisplay();
void drawScatterPlot();
void drawSensorList();
void drawStatsScreen();
void drawHeader();
void drawSensorRow(int index, int yPos);
void drawRSSIBars(int x, int y, int8_t rssi);
void drawPressureBar(int x, int y, float pressure);
void handleTouch();
void handleButton();
String formatTime(unsigned long ms);
uint16_t getPressureColor(float psi);
uint16_t getTempColor(float temp);

// Interrupt handler
void IRAM_ATTR onPacketReceived() {
  packetReceived = true;
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize histogram
  memset(psiHistogram, 0, sizeof(psiHistogram));

  // Initialize BOOT button for display mode switching
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  Serial.println("\n=================================");
  Serial.println("    TPMS Scanner v1.0");
  Serial.println("    ESP32 DevKit + CC1101");
  Serial.println("=================================\n");

  // Initialize display (if enabled)
  initDisplay();

#ifdef USE_TFT_DISPLAY
  // Show splash screen on TFT
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
  // Show splash screen on OLED
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

  Serial.println("Initializing CC1101 radio module...");

  // Initialize CC1101
  if (!initCC1101()) {
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
    Serial.println("ERROR: CC1101 initialization failed!");
    Serial.println("Check wiring:");
    Serial.println("  VCC  -> 3.3V");
    Serial.println("  GND  -> GND");
    Serial.println("  CSN  -> GPIO 5");
    Serial.println("  SCK  -> GPIO 18");
    Serial.println("  MOSI -> GPIO 23");
    Serial.println("  MISO -> GPIO 19");
    Serial.println("  GDO0 -> GPIO 21");
    while (1) delay(1000);  // Halt
  }

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

  // Setup interrupt - trigger on RISING (carrier detected) or FALLING (carrier lost)
  pinMode(CC1101_GDO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), onPacketReceived, CHANGE);

  // Start receiving
  cc1101SetFrequency(FREQUENCY);
  cc1101StartRX();

  startTime = millis();

  delay(500);

#ifdef USE_TFT_DISPLAY
  // Draw main display
  tft.fillScreen(COLOR_BG);
#endif
  drawDisplay();

  Serial.println("TPMS Scanner ready!");
  Serial.println("Scanning 315 MHz (North America)...");
  Serial.println("Watch Serial Monitor for detected sensors.\n");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  unsigned long currentTime = millis();

  // Process received packets
  if (packetReceived) {
    packetReceived = false;
    processPacket();
  }


  // Poll FIFO for data every 50ms
  // With sync word detection enabled, we only receive real TPMS packets
  static unsigned long lastFIFOPoll = 0;
  if (currentTime - lastFIFOPoll >= 50) {
    lastFIFOPoll = currentTime;

    uint8_t rxBytes = cc1101ReadStatus(CC1101_RXBYTES) & 0x7F;
    bool overflow = cc1101ReadStatus(CC1101_RXBYTES) & 0x80;

    if (overflow) {
      // FIFO overflow - flush and restart
      Serial.println("[WARN] FIFO overflow, flushing...");
      cc1101Strobe(CC1101_SFRX);
      cc1101Strobe(CC1101_SRX);
      return;
    }

    // With sync word 0x001A configured, we should only get data when
    // a valid TPMS preamble is detected - no more noise packets!
    if (rxBytes >= 6) {  // Minimum TPMS packet size
      int8_t rssi = cc1101GetRSSI();

      // Filter extremely weak signals (below noise floor)
      // Note: TPMS signals can be weak (-100 to -110 dBm) depending on distance
      if (rssi < -115) {
        // Below noise floor - definitely not a real signal
        cc1101Strobe(CC1101_SFRX);
        cc1101Strobe(CC1101_SRX);
        return;
      }

      totalPackets++;

      Serial.printf("\n>>> TPMS RX [%d MHz] %d bytes, RSSI: %d dBm\n", FREQUENCY, rxBytes, rssi);
      Serial.print("    RAW: ");

      // Read all bytes from FIFO
      uint8_t packet[64];
      int len = min((int)rxBytes, 64);

      digitalWrite(CC1101_CS, LOW);
      if (!waitMISO()) {
        digitalWrite(CC1101_CS, HIGH);
        cc1101Strobe(CC1101_SFRX);
        cc1101Strobe(CC1101_SRX);
        return;
      }
      cc1101SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
      cc1101SPI.transfer(CC1101_RXFIFO | CC1101_READ_BURST);
      for (int i = 0; i < len; i++) {
        packet[i] = cc1101SPI.transfer(0);
      }
      cc1101SPI.endTransaction();
      digitalWrite(CC1101_CS, HIGH);

      // Print hex dump
      for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", packet[i]);
      }
      Serial.println();

      // Try Manchester decoding (TPMS typically uses Manchester encoding)
      uint8_t decoded[32];
      int decodedLen = manchesterDecode(packet, len, decoded, 32);

      if (decodedLen > 0) {
        Serial.print("    MANCHESTER: ");
        for (int i = 0; i < decodedLen; i++) {
          Serial.printf("%02X ", decoded[i]);
        }
        Serial.println();

        // Try to decode the Manchester-decoded data
        decodeTPMSPacket(decoded, decodedLen, rssi);
      } else {
        // Try raw decode if Manchester fails
        decodeTPMSPacket(packet, len, rssi);
      }

      // Flush and restart RX
      cc1101Strobe(CC1101_SFRX);
      cc1101Strobe(CC1101_SRX);
    }
  }

  // Debug: Print RSSI every 5 seconds
  static unsigned long lastRSSIPrint = 0;
  if (currentTime - lastRSSIPrint >= 5000) {
    lastRSSIPrint = currentTime;
    int8_t rssi = cc1101GetRSSI();
    uint8_t rxBytes = cc1101ReadStatus(CC1101_RXBYTES) & 0x7F;
    Serial.printf("[STATUS] %d MHz | RSSI: %d dBm | FIFO: %d bytes | Pkts: %lu\n",
                  FREQUENCY, rssi, rxBytes, totalPackets);
  }

  // Update display
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdate = currentTime;

    // Prune old sensors
    pruneOldSensors();

    // Redraw display
    drawDisplay();
  }

  // Decay histogram periodically (prevents bars from maxing out over long runs)
  if (currentTime - lastHistogramDecay >= HISTOGRAM_DECAY_INTERVAL_MS) {
    lastHistogramDecay = currentTime;
    decayHistogram();
  }

  // Handle BOOT button for display mode switching
  handleButton();

  // Handle touch input
  handleTouch();

  yield();
}

// ============================================================================
// Display Functions
// ============================================================================

void initDisplay() {
#ifdef USE_TFT_DISPLAY
  // Initialize backlight pin
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);  // Turn on backlight

  tft.init();
  tft.setRotation(1);  // Landscape (480x320)
  tft.fillScreen(COLOR_BG);
  Serial.println("TFT Display initialized (480x320) on HSPI");
  Serial.println("TFT pins: MISO=12, MOSI=13, SCLK=14, CS=15, DC=2, RST=4, BL=22");
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
  if(!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("ERROR: SSD1306 allocation failed!");
    // Don't halt - continue with serial only
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

// Track if we need full redraw (reduces flicker)
static int lastSensorCount = -1;
static bool needsFullRedraw = true;

void drawDisplay() {
#ifdef USE_TFT_DISPLAY
  // Only do full clear when sensor count changes or on first draw
  bool sensorCountChanged = (sensorCount != lastSensorCount);

  drawHeader();

  // Draw sensor list
  int visibleCount = min(sensorCount - scrollOffset, MAX_VISIBLE_SENSORS);
  int yPos = HEADER_HEIGHT + 5;

  // Only clear sensor area when needed (reduces flicker)
  bool didFullRedraw = false;
  if (needsFullRedraw || sensorCountChanged) {
    tft.fillRect(0, HEADER_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - HEADER_HEIGHT, COLOR_BG);
    lastSensorCount = sensorCount;
    needsFullRedraw = false;
    didFullRedraw = true;
  }

  if (sensorCount == 0) {
    // Show scanning message (only redraw on first frame or after sensor count change)
    if (didFullRedraw) {
      tft.setTextColor(COLOR_TEXT);
      tft.setTextSize(2);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Scanning for TPMS sensors...", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
      tft.setTextSize(1);
      tft.drawString("315 MHz (North America)", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 30);
    }

    // Animated scanning indicator - clear just the dots area
    static int scanAnim = 0;
    scanAnim = (scanAnim + 1) % 4;
    tft.fillRect(SCREEN_WIDTH / 2 - 20, SCREEN_HEIGHT / 2 + 45, 40, 15, COLOR_BG);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    String dots = "";
    for (int i = 0; i <= scanAnim; i++) dots += ".";
    tft.drawString(dots, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 50);
  } else {
    for (int i = 0; i < visibleCount; i++) {
      int sensorIdx = i + scrollOffset;
      drawSensorRow(sensorIdx, yPos);
      yPos += SENSOR_ROW_HEIGHT;

      // Draw divider
      if (i < visibleCount - 1) {
        tft.drawLine(10, yPos - 2, SCREEN_WIDTH - 10, yPos - 2, COLOR_DIVIDER);
      }
    }
  }

  // Scroll indicators
  if (scrollOffset > 0) {
    tft.fillTriangle(SCREEN_WIDTH - 20, HEADER_HEIGHT + 10,
                     SCREEN_WIDTH - 10, HEADER_HEIGHT + 10,
                     SCREEN_WIDTH - 15, HEADER_HEIGHT + 5, COLOR_TEXT);
  }
  if (scrollOffset + MAX_VISIBLE_SENSORS < sensorCount) {
    tft.fillTriangle(SCREEN_WIDTH - 20, SCREEN_HEIGHT - 15,
                     SCREEN_WIDTH - 10, SCREEN_HEIGHT - 15,
                     SCREEN_WIDTH - 15, SCREEN_HEIGHT - 10, COLOR_TEXT);
  }
#elif defined(USE_OLED_DISPLAY)
  // OLED display (128x64) - Multiple display modes
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // Draw the appropriate screen based on display mode
  switch (displayMode) {
    case DISPLAY_MODE_SCATTER:
      drawScatterPlot();
      break;
    case DISPLAY_MODE_SENSORS:
      drawSensorList();
      break;
    case DISPLAY_MODE_STATS:
      drawStatsScreen();
      break;
  }

  oled.display();

#else
  // Serial-only mode: print status periodically
  static unsigned long lastSerialPrint = 0;
  if (millis() - lastSerialPrint > 2000) {
    lastSerialPrint = millis();
    Serial.printf("\n=== TPMS Scanner Status [%d MHz] ===\n", FREQUENCY);
    Serial.printf("Packets: %lu | Sensors: %d | Uptime: %s\n",
                  totalPackets, sensorCount, formatTime(millis() - startTime).c_str());

    if (sensorCount > 0) {
      Serial.println("─────────────────────────────────────────────────────");
      for (int i = 0; i < sensorCount; i++) {
        Serial.printf("ID: %08X | %.1f PSI (%.0f kPa) | %.0fC | %ddBm | %s\n",
                      sensors[i].id, sensors[i].pressure_psi, sensors[i].pressure_kpa,
                      sensors[i].temperature_c, sensors[i].rssi, sensors[i].protocol.c_str());
      }
    }
  }
#endif
}

void drawHeader() {
#ifdef USE_TFT_DISPLAY
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
  sprintf(freqStr, "%d MHz", FREQUENCY);
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
#endif
}

#ifdef USE_TFT_DISPLAY
void drawSensorRow(int index, int yPos) {
  if (index < 0 || index >= sensorCount) return;

  TPMSSensor& sensor = sensors[index];
  char buf[32];

  // New sensor indicator
  if (sensor.isNew) {
    tft.fillCircle(12, yPos + SENSOR_ROW_HEIGHT / 2, 6, COLOR_NEW_SENSOR);
  }

  // Frequency indicator
  uint16_t freqColor = COLOR_FREQ_315;  // 315 MHz only
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
  // Convert RSSI to 0-5 bars
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
  // Typical tire pressure range: 20-40 PSI
  int barWidth = 80;
  int barHeight = 8;

  // Draw outline
  tft.drawRect(x, y, barWidth, barHeight, COLOR_TEXT);

  // Calculate fill (20-40 PSI range)
  float normalized = constrain((pressure - 20) / 20.0, 0, 1);
  int fillWidth = normalized * (barWidth - 2);

  // Color based on pressure
  uint16_t fillColor = getPressureColor(pressure);

  if (fillWidth > 0) {
    tft.fillRect(x + 1, y + 1, fillWidth, barHeight - 2, fillColor);
  }

  // Target zone marker (around 32 PSI)
  int targetX = x + (int)((32 - 20) / 20.0 * barWidth);
  tft.drawLine(targetX, y - 2, targetX, y + barHeight + 2, COLOR_TEXT);
}
#endif  // USE_TFT_DISPLAY

uint16_t getPressureColor(float psi) {
  if (psi < 25) return COLOR_PRESSURE_LOW;    // Too low
  if (psi > 38) return COLOR_PRESSURE_HIGH;   // Too high
  return COLOR_PRESSURE_OK;                    // Normal
}

uint16_t getTempColor(float temp) {
  if (temp > 60) return COLOR_TEMP_HOT;       // Hot tire
  return COLOR_TEMP_NORMAL;
}

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

// ============================================================================
// CC1101 Functions
// ============================================================================

bool initCC1101() {
  // Initialize SPI
  cc1101SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

  pinMode(CC1101_CS, OUTPUT);
  digitalWrite(CC1101_CS, HIGH);

  // Reset CC1101
  cc1101Reset();
  delay(100);

  // Verify CC1101 is present
  uint8_t partnum = cc1101ReadStatus(CC1101_PARTNUM);
  uint8_t version = cc1101ReadStatus(CC1101_VERSION);

  Serial.printf("CC1101 Part: 0x%02X, Version: 0x%02X\n", partnum, version);

  // CC1101 should return 0x00 for partnum, version varies (0x04, 0x14, etc.)
  if (partnum != 0x00 || version == 0x00 || version == 0xFF) {
    Serial.println("ERROR: Invalid CC1101 response!");
    return false;
  }

  // Load 315 MHz configuration (North America)
  for (int i = 0; i < sizeof(CC1101_CONFIG_315); i++) {
    cc1101WriteReg(i, CC1101_CONFIG_315[i]);
  }

  Serial.println("CC1101 initialized successfully");
  return true;
}

void cc1101Reset() {
  digitalWrite(CC1101_CS, LOW);
  delayMicroseconds(10);
  digitalWrite(CC1101_CS, HIGH);
  delayMicroseconds(40);

  digitalWrite(CC1101_CS, LOW);
  if (!waitMISO()) {  // Wait for MISO to go low
    digitalWrite(CC1101_CS, HIGH);
    return;
  }

  cc1101SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  cc1101SPI.transfer(CC1101_SRES);
  cc1101SPI.endTransaction();

  if (!waitMISO()) {  // Wait for reset to complete
    digitalWrite(CC1101_CS, HIGH);
    return;
  }
  digitalWrite(CC1101_CS, HIGH);
}

void cc1101WriteReg(uint8_t addr, uint8_t value) {
  digitalWrite(CC1101_CS, LOW);
  if (!waitMISO()) {
    digitalWrite(CC1101_CS, HIGH);
    return;
  }

  cc1101SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  cc1101SPI.transfer(addr);
  cc1101SPI.transfer(value);
  cc1101SPI.endTransaction();

  digitalWrite(CC1101_CS, HIGH);
}

uint8_t cc1101ReadReg(uint8_t addr) {
  uint8_t value = 0;

  digitalWrite(CC1101_CS, LOW);
  if (!waitMISO()) {
    digitalWrite(CC1101_CS, HIGH);
    return 0xFF;  // Return error value
  }

  cc1101SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  cc1101SPI.transfer(addr | CC1101_READ_SINGLE);
  value = cc1101SPI.transfer(0);
  cc1101SPI.endTransaction();

  digitalWrite(CC1101_CS, HIGH);
  return value;
}

uint8_t cc1101ReadStatus(uint8_t addr) {
  uint8_t value = 0;

  digitalWrite(CC1101_CS, LOW);
  if (!waitMISO()) {
    digitalWrite(CC1101_CS, HIGH);
    return 0xFF;  // Return error value
  }

  cc1101SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  cc1101SPI.transfer(addr | CC1101_READ_BURST);
  value = cc1101SPI.transfer(0);
  cc1101SPI.endTransaction();

  digitalWrite(CC1101_CS, HIGH);
  return value;
}

void cc1101Strobe(uint8_t strobe) {
  digitalWrite(CC1101_CS, LOW);
  if (!waitMISO()) {
    digitalWrite(CC1101_CS, HIGH);
    return;
  }

  cc1101SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  cc1101SPI.transfer(strobe);
  cc1101SPI.endTransaction();

  digitalWrite(CC1101_CS, HIGH);
}

void cc1101SetFrequency(uint16_t freqMHz) {
  // Go to idle state first
  cc1101Strobe(CC1101_SIDLE);

  // Load 315 MHz config (North America)
  for (int i = 0; i < sizeof(CC1101_CONFIG_315); i++) {
    cc1101WriteReg(i, CC1101_CONFIG_315[i]);
  }

  Serial.printf("CC1101 configured for %d MHz\n", freqMHz);
}

void cc1101StartRX() {
  cc1101Strobe(CC1101_SFRX);   // Flush RX FIFO
  cc1101Strobe(CC1101_SRX);    // Enter RX mode
}

int8_t cc1101GetRSSI() {
  uint8_t rssi_raw = cc1101ReadStatus(CC1101_RSSI);
  int16_t rssi_dbm;

  if (rssi_raw >= 128) {
    rssi_dbm = ((int16_t)rssi_raw - 256) / 2 - 74;
  } else {
    rssi_dbm = rssi_raw / 2 - 74;
  }

  return (int8_t)rssi_dbm;
}

// ============================================================================
// Manchester Decoding
// ============================================================================

// Manchester decode: convert 2 bits to 1 bit
// Convention: 01 = 0, 10 = 1 (IEEE 802.3)
// Returns decoded length, or 0 if decoding fails
int manchesterDecode(uint8_t* input, int inputLen, uint8_t* output, int maxOutput) {
  int outIdx = 0;
  int bitPos = 0;
  uint8_t outByte = 0;

  // Process input as bit pairs
  for (int i = 0; i < inputLen && outIdx < maxOutput; i++) {
    uint8_t inByte = input[i];

    for (int b = 7; b >= 1; b -= 2) {
      uint8_t bit1 = (inByte >> b) & 1;
      uint8_t bit0 = (inByte >> (b - 1)) & 1;

      uint8_t decoded;
      if (bit1 == 0 && bit0 == 1) {
        decoded = 0;  // 01 -> 0
      } else if (bit1 == 1 && bit0 == 0) {
        decoded = 1;  // 10 -> 1
      } else {
        // Invalid Manchester pair - might indicate packet boundary
        // Continue anyway for TPMS which may have preamble/sync issues
        decoded = bit1;  // Take first bit as fallback
      }

      outByte = (outByte << 1) | decoded;
      bitPos++;

      if (bitPos == 8) {
        output[outIdx++] = outByte;
        outByte = 0;
        bitPos = 0;
      }
    }
  }

  // Return decoded length (will be roughly half of input)
  return outIdx;
}

// Alternative: Try to decode without Manchester (some sensors don't use it)
// This tries direct byte interpretation
void tryDirectDecode(uint8_t* data, int len, int8_t rssi) {
  if (len < 5) return;

  // Look for Toyota PMV-107J format (direct FSK, no Manchester):
  // [ID:4 bytes] [Status:1] [Pressure:1] [Temp:1] [Checksum:1]

  uint32_t id = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                ((uint32_t)data[2] << 8) | data[3];

  if (id != 0 && id != 0xFFFFFFFF) {
    uint8_t status = (len > 4) ? data[4] : 0;
    uint8_t pressureRaw = (len > 5) ? data[5] : 0;
    uint8_t tempRaw = (len > 6) ? data[6] : 0;

    // Toyota pressure: value * 2.5 kPa (or direct kPa depending on variant)
    float pressure_kpa = pressureRaw * 2.5;

    // Toyota temp: value - 40 (or value - 50 for some)
    float temp_c = tempRaw - 40.0;

    // Validate
    if (pressure_kpa > 100 && pressure_kpa < 450 &&
        temp_c > -30 && temp_c < 100) {
      Serial.printf(">>> DIRECT DECODE: ID=%08X P=%.0f kPa T=%.0f C\n",
                    id, pressure_kpa, temp_c);
      updateSensor(id, pressure_kpa, temp_c, rssi, FREQUENCY, "Toyota");
      validPackets++;
    }
  }
}

// ============================================================================
// Packet Processing
// ============================================================================

void processPacket() {
  totalPackets++;

  // Read number of bytes in FIFO
  uint8_t rxBytes = cc1101ReadStatus(CC1101_RXBYTES);

  if (rxBytes & 0x80) {  // Overflow
    cc1101Strobe(CC1101_SFRX);
    cc1101Strobe(CC1101_SRX);
    return;
  }

  if (rxBytes == 0) {
    cc1101Strobe(CC1101_SRX);
    return;
  }

  // Read RSSI first
  int8_t rssi = cc1101GetRSSI();

  // Read packet data
  uint8_t packet[64];
  int len = min((int)rxBytes, 64);

  digitalWrite(CC1101_CS, LOW);
  if (!waitMISO()) {
    digitalWrite(CC1101_CS, HIGH);
    cc1101Strobe(CC1101_SFRX);
    cc1101Strobe(CC1101_SRX);
    return;
  }

  cc1101SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  cc1101SPI.transfer(CC1101_RXFIFO | CC1101_READ_BURST);
  for (int i = 0; i < len; i++) {
    packet[i] = cc1101SPI.transfer(0);
  }
  cc1101SPI.endTransaction();

  digitalWrite(CC1101_CS, HIGH);

  // Debug output
  Serial.printf("RX [%d MHz] %d bytes, RSSI: %d dBm: ", FREQUENCY, len, rssi);
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();

  // Try to decode TPMS data
  decodeTPMSPacket(packet, len, rssi);

  // Restart RX
  cc1101Strobe(CC1101_SFRX);
  cc1101Strobe(CC1101_SRX);
}

void decodeTPMSPacket(uint8_t* data, uint8_t len, int8_t rssi) {
  // TPMS protocols vary by manufacturer. Common formats:
  // - FSK modulated, Manchester encoded
  // - Packet typically contains: sync, ID (4 bytes), pressure, temp, checksum

  // Minimum viable packet length
  if (len < 4) return;

  uint32_t sensorId = 0;
  float pressure_kpa = 0;
  float temperature_c = 0;
  const char* protocol = "Unknown";
  bool decoded = false;

  // ===== Toyota PMV-107J format (2008+ Corolla, Prius, RAV4, etc.) =====
  // Format: [ID:28 bits] [Status:4 bits] [Pressure:8 bits] [Temp:8 bits] [CRC:8 bits]
  // Total: 7 bytes after Manchester decoding
  // Pressure: raw * 0.25 kPa (some variants use * 2.5)
  // Temp: raw - 40 C

  if (len >= 7 && !decoded) {
    sensorId = ((uint32_t)data[0] << 20) | ((uint32_t)data[1] << 12) |
               ((uint32_t)data[2] << 4) | ((data[3] >> 4) & 0x0F);

    uint8_t pressureRaw = data[4];
    uint8_t tempRaw = data[5];

    // Toyota uses pressure * 0.25 kPa (range 0-255 = 0-64 kPa, but actually 0-637.5 kPa)
    // Some variants multiply differently
    pressure_kpa = pressureRaw * 2.5;  // Try common 2.5 multiplier first
    temperature_c = tempRaw - 40.0;

    // Validation: realistic tire pressure 150-350 kPa (22-50 PSI)
    // Temperature: -10°C to 60°C (14°F to 140°F) for cold weather
    if (sensorId != 0 && sensorId != 0x0FFFFFFF &&
        pressure_kpa >= 150 && pressure_kpa <= 350 &&
        temperature_c >= -10 && temperature_c <= 60) {
      validPackets++;
      protocol = "Toyota";
      decoded = true;

      float temp_f = temperature_c * 9.0 / 5.0 + 32.0;
      Serial.printf("TPMS [Toyota]: ID=%07X, P=%.1f kPa (%.1f PSI), T=%.0fF\n",
                    sensorId, pressure_kpa, pressure_kpa * 0.145038, temp_f);

      updateSensor(sensorId, pressure_kpa, temperature_c, rssi, FREQUENCY, protocol);
    }
  }

  // ===== Schrader format (very common, used by many OEMs) =====
  // Format: [ID:32 bits] [Status:8 bits] [Pressure:8 bits] [Temp:8 bits] [CRC:8 bits]
  if (len >= 8 && !decoded) {
    sensorId = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
               ((uint32_t)data[2] << 8) | data[3];

    if (sensorId != 0 && sensorId != 0xFFFFFFFF) {
      uint8_t pressureRaw = data[5];
      uint8_t tempRaw = data[6];

      pressure_kpa = pressureRaw * 2.5;
      temperature_c = tempRaw - 40.0;  // Changed from -50 to -40

      // Validation: realistic tire pressure 150-350 kPa (22-50 PSI)
      // Temperature: -10°C to 60°C (14°F to 140°F) for cold weather
      if (pressure_kpa >= 150 && pressure_kpa <= 350 &&
          temperature_c >= -10 && temperature_c <= 60) {
        validPackets++;
        protocol = "Schrader";
        decoded = true;

        float temp_f = temperature_c * 9.0 / 5.0 + 32.0;
        Serial.printf("TPMS [Schrader]: ID=%08X, P=%.1f kPa (%.1f PSI), T=%.0fF\n",
                      sensorId, pressure_kpa, pressure_kpa * 0.145038, temp_f);

        updateSensor(sensorId, pressure_kpa, temperature_c, rssi, FREQUENCY, protocol);
      }
    }
  }

  // ===== Generic/Aftermarket format (byte-aligned, simple) =====
  if (len >= 6 && !decoded) {
    // Try big-endian ID
    sensorId = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
               ((uint32_t)data[2] << 8) | data[3];

    if (sensorId == 0 || sensorId == 0xFFFFFFFF) {
      // Try little-endian
      sensorId = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) |
                 ((uint32_t)data[1] << 8) | data[0];
    }

    if (sensorId != 0 && sensorId != 0xFFFFFFFF) {
      uint8_t pressureRaw = data[4];
      uint8_t tempRaw = data[5];

      // Try different pressure scalings
      pressure_kpa = pressureRaw * 2.5;
      temperature_c = tempRaw - 40.0;  // Try -40 offset

      if (pressure_kpa < 100) {
        pressure_kpa = pressureRaw * 4.0;  // Some use *4
      }
      if (temperature_c < -30) {
        temperature_c = tempRaw - 50.0;  // Try -50 offset
      }

      // Validation: -10°C to 60°C (14°F to 140°F) for cold weather
      if (pressure_kpa >= 150 && pressure_kpa <= 350 &&
          temperature_c >= -10 && temperature_c <= 60) {
        validPackets++;
        protocol = "Generic";
        decoded = true;

        float temp_f = temperature_c * 9.0 / 5.0 + 32.0;
        Serial.printf("TPMS [Generic]: ID=%08X, P=%.1f kPa (%.1f PSI), T=%.0fF\n",
                      sensorId, pressure_kpa, pressure_kpa * 0.145038, temp_f);

        updateSensor(sensorId, pressure_kpa, temperature_c, rssi, FREQUENCY, protocol);
      }
    }
  }

  // Note: Not using tryDirectDecode as fallback - too many false positives
}

void updateSensor(uint32_t id, float pressure_kpa, float temp_c, int8_t rssi, uint16_t freq, const char* protocol) {
  unsigned long now = millis();
  float psi = pressure_kpa * 0.145038;

  // Update histogram for scatter plot (clamp to PSI_MIN-PSI_MAX range)
  int psiBin = constrain((int)psi - PSI_MIN, 0, PSI_BINS - 1);
  if (psiHistogram[psiBin] < 255) {  // Prevent overflow
    psiHistogram[psiBin]++;
  }

  // Check if sensor already exists
  bool isNewSensor = true;
  for (int i = 0; i < sensorCount; i++) {
    if (sensors[i].id == id) {
      // Update existing sensor
      sensors[i].pressure_kpa = pressure_kpa;
      sensors[i].pressure_psi = psi;
      sensors[i].temperature_c = temp_c;
      sensors[i].rssi = rssi;
      sensors[i].frequency = freq;
      sensors[i].lastSeen = now;
      sensors[i].detectionCount++;
      sensors[i].isNew = (now - sensors[i].firstSeen) < NEW_SENSOR_THRESHOLD;
      sensors[i].protocol = protocol;
      isNewSensor = false;
      return;
    }
  }

  // Add new sensor
  if (sensorCount < MAX_TRACKED_SENSORS) {
    TPMSSensor& newSensor = sensors[sensorCount];
    newSensor.id = id;
    newSensor.pressure_kpa = pressure_kpa;
    newSensor.pressure_psi = psi;
    newSensor.temperature_c = temp_c;
    newSensor.rssi = rssi;
    newSensor.frequency = freq;
    newSensor.firstSeen = now;
    newSensor.lastSeen = now;
    newSensor.detectionCount = 1;
    newSensor.isNew = true;
    newSensor.battery_low = false;
    newSensor.battery_pct = 100;
    newSensor.protocol = protocol;

    sensorCount++;
    uniqueSensors++;  // Track total unique sensors ever seen

    Serial.printf("New sensor added: ID=%08X (total: %d)\n", id, sensorCount);
  }
}

void pruneOldSensors() {
  unsigned long now = millis();
  int i = 0;

  while (i < sensorCount) {
    if (now - sensors[i].lastSeen > SENSOR_TIMEOUT_MS) {
      Serial.printf("Removing stale sensor: %08X\n", sensors[i].id);

      // Shift remaining sensors
      for (int j = i; j < sensorCount - 1; j++) {
        sensors[j] = sensors[j + 1];
      }
      sensorCount--;
    } else {
      // Update isNew flag
      sensors[i].isNew = (now - sensors[i].firstSeen) < NEW_SENSOR_THRESHOLD;
      i++;
    }
  }
}

void decayHistogram() {
  // Reduce all histogram bins by HISTOGRAM_DECAY_PERCENT
  // but keep a minimum floor so past readings leave a "ghost"
  for (int i = 0; i < PSI_BINS; i++) {
    if (psiHistogram[i] > HISTOGRAM_MIN_VALUE) {
      // Calculate decay: reduce by X%
      uint8_t decay = (psiHistogram[i] * HISTOGRAM_DECAY_PERCENT) / 100;
      if (decay < 1) decay = 1;  // Ensure we always decay by at least 1

      // Apply decay but maintain floor
      if (psiHistogram[i] - decay >= HISTOGRAM_MIN_VALUE) {
        psiHistogram[i] -= decay;
      } else {
        psiHistogram[i] = HISTOGRAM_MIN_VALUE;
      }
    }
    // Bins at 0 stay at 0 (never had readings)
    // Bins at HISTOGRAM_MIN_VALUE stay there (had readings, now at floor)
  }

  Serial.println("[DECAY] Histogram decayed by 3%");
}

// ============================================================================
// Touch Handling
// ============================================================================

void handleTouch() {
#ifdef USE_TFT_DISPLAY
  // Touch handling would go here for displays with touch
  // For now, this is a placeholder
#endif
}

// ============================================================================
// Button Handling
// ============================================================================

void handleButton() {
  bool buttonState = digitalRead(BOOT_BUTTON);
  unsigned long now = millis();

  // Check for button press (transition from HIGH to LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Debounce check
    if (now - lastButtonPress >= BUTTON_DEBOUNCE_MS) {
      lastButtonPress = now;

      // Cycle to next display mode
      displayMode = (displayMode + 1) % DISPLAY_MODE_COUNT;

      const char* modeNames[] = {"Scatter Plot", "Sensor List", "Statistics"};
      Serial.printf("[MODE] Display mode: %s\n", modeNames[displayMode]);

      // Force immediate display update
      drawDisplay();
    }
  }

  lastButtonState = buttonState;
}

// ============================================================================
// OLED Display Modes
// ============================================================================

#ifdef USE_OLED_DISPLAY

void drawScatterPlot() {
  // ===== YELLOW ZONE (Y 0-15) - Stats =====
  // Unique sensor count (large, left side)
  oled.setCursor(2, 0);
  oled.setTextSize(2);
  oled.print(uniqueSensors);

  // Total readings (large, right side) - matching font
  oled.setTextSize(2);
  // Right-align the number: calculate position based on digit count
  int digits = (validPackets >= 1000) ? 4 : (validPackets >= 100) ? 3 : (validPackets >= 10) ? 2 : 1;
  int xPos = 128 - (digits * 12) - 2;  // 12 pixels per char at size 2
  oled.setCursor(xPos, 0);
  oled.print(validPackets);

  // ===== BLUE ZONE (Y 16-63) - Scatter Plot =====
  // Graph area: X 8-120, Y 18-55
  // X-axis labels at bottom (Y 56-63)

  #define GRAPH_LEFT 8
  #define GRAPH_RIGHT 120
  #define GRAPH_TOP 18
  #define GRAPH_BOTTOM 54
  #define GRAPH_WIDTH (GRAPH_RIGHT - GRAPH_LEFT)
  #define GRAPH_HEIGHT (GRAPH_BOTTOM - GRAPH_TOP)

  // Draw axes
  oled.drawFastVLine(GRAPH_LEFT - 1, GRAPH_TOP, GRAPH_HEIGHT + 2, SSD1306_WHITE);  // Y-axis
  oled.drawFastHLine(GRAPH_LEFT - 1, GRAPH_BOTTOM + 1, GRAPH_WIDTH + 2, SSD1306_WHITE);  // X-axis

  // X-axis labels (PSI)
  oled.setTextSize(1);
  oled.setCursor(GRAPH_LEFT - 2, 56);
  oled.print(PSI_MIN);
  oled.setCursor(GRAPH_LEFT + GRAPH_WIDTH/2 - 6, 56);
  oled.print((PSI_MIN + PSI_MAX) / 2);
  oled.setCursor(GRAPH_RIGHT - 8, 56);
  oled.print(PSI_MAX);

  // Draw scatter dots from histogram
  // Each PSI bin maps to X position, height = count
  for (int bin = 0; bin < PSI_BINS; bin++) {
    if (psiHistogram[bin] > 0) {
      // Map bin index to X position
      int x = GRAPH_LEFT + (bin * GRAPH_WIDTH) / (PSI_BINS - 1);

      // Draw stacked dots (each dot is 2 pixels, stack up to fill graph)
      int count = min((int)psiHistogram[bin], GRAPH_HEIGHT / 3);  // Max dots that fit
      for (int d = 0; d < count; d++) {
        int y = GRAPH_BOTTOM - 2 - (d * 3);  // Stack from bottom up
        if (y >= GRAPH_TOP) {
          oled.fillRect(x, y, 2, 2, SSD1306_WHITE);  // 2x2 dot
        }
      }
    }
  }

  // If no data yet, show subtle "waiting" indicator
  if (validPackets == 0) {
    static int waitAnim = 0;
    waitAnim = (waitAnim + 1) % 4;
    int cx = GRAPH_LEFT + GRAPH_WIDTH/2;
    int cy = GRAPH_TOP + GRAPH_HEIGHT/2;
    oled.drawCircle(cx, cy, 4 + waitAnim, SSD1306_WHITE);
  }
}

void drawSensorList() {
  // Title
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("SENSORS (");
  oled.print(sensorCount);
  oled.print(")");

  // Draw horizontal line under title
  oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  if (sensorCount == 0) {
    oled.setCursor(20, 28);
    oled.print("No sensors");
    return;
  }

  // Show up to 4 most recent sensors (start at Y=16, clear of the line)
  int startIdx = max(0, sensorCount - 4);
  int y = 16;

  for (int i = sensorCount - 1; i >= startIdx && y < 64; i--) {
    TPMSSensor& s = sensors[i];

    // Sensor ID (short form)
    oled.setCursor(0, y);
    oled.printf("%06X", s.id & 0xFFFFFF);  // Last 6 hex digits

    // Pressure
    oled.setCursor(44, y);
    oled.printf("%2.0f", s.pressure_psi);

    // Temperature (Fahrenheit)
    float tempF = s.temperature_c * 9.0 / 5.0 + 32.0;
    oled.setCursor(68, y);
    oled.printf("%3.0fF", tempF);

    // RSSI indicator (simple)
    oled.setCursor(100, y);
    if (s.rssi >= -70) oled.print("+++");
    else if (s.rssi >= -85) oled.print("++ ");
    else if (s.rssi >= -100) oled.print("+  ");
    else oled.print("-  ");

    // New sensor indicator
    if (s.isNew) {
      oled.setCursor(120, y);
      oled.print("*");
    }

    y += 12;
  }
}

void drawStatsScreen() {
  oled.setTextSize(1);

  // Title
  oled.setCursor(30, 0);
  oled.print("STATISTICS");
  oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  // Uptime (start at Y=16, clear of the line)
  oled.setCursor(0, 16);
  oled.print("Uptime: ");
  oled.print(formatTime(millis() - startTime));

  // Total packets received
  oled.setCursor(0, 26);
  oled.print("Packets: ");
  oled.print(totalPackets);

  // Valid TPMS readings
  oled.setCursor(0, 36);
  oled.print("Valid:   ");
  oled.print(validPackets);

  // Unique sensors
  oled.setCursor(0, 46);
  oled.print("Sensors: ");
  oled.print(uniqueSensors);
  oled.print(" (");
  oled.print(sensorCount);
  oled.print(" active)");

  // Current RSSI
  oled.setCursor(0, 56);
  oled.print("RSSI: ");
  int8_t rssi = cc1101GetRSSI();
  oled.print(rssi);
  oled.print(" dBm");
}

#endif  // USE_OLED_DISPLAY
