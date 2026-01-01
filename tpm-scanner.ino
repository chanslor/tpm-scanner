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

// ============================================================================
// Display Selection - uncomment ONE:
// ============================================================================
// #define USE_OLED_DISPLAY   // 0.96" I2C OLED (SSD1306 128x64)
#define USE_TFT_DISPLAY       // 3.5" SPI TFT (ILI9488 480x320)

// ============================================================================
// Includes
// ============================================================================

#include <SPI.h>
#include "cc1101.h"
#include "tpms_decoders.h"
#include "display.h"

// ============================================================================
// Button Configuration
// ============================================================================

#define BOOT_BUTTON        0      // GPIO 0 - BOOT button (built into ESP32 DevKit)
#define BUTTON_DEBOUNCE_MS 250    // Button debounce time

// ============================================================================
// Timing Configuration
// ============================================================================

#define DISPLAY_UPDATE_MS  500    // Display refresh interval

// ============================================================================
// Global Objects
// ============================================================================

CC1101 radio;
TPMSDecoder decoder;
DisplayManager display;

// ============================================================================
// Global State
// ============================================================================

const uint16_t FREQUENCY = 315;   // 315 MHz for North America

volatile bool packetReceived = false;
bool scanning = true;

unsigned long totalPackets = 0;
unsigned long startTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastHistogramDecay = 0;

// Button state
unsigned long lastButtonPress = 0;
bool lastButtonState = HIGH;  // Button is active LOW

// ============================================================================
// Interrupt Handler
// ============================================================================

void IRAM_ATTR onPacketReceived() {
  packetReceived = true;
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize BOOT button for display mode switching
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  Serial.println("\n=================================");
  Serial.println("    TPMS Scanner v1.0");
  Serial.println("    ESP32 DevKit + CC1101");
  Serial.println("=================================\n");

  // Initialize display
  display.init();
  display.showSplash();

  Serial.println("Initializing CC1101 radio module...");

  // Initialize CC1101
  if (!radio.init()) {
    display.showInitFailed();
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

  display.showInitSuccess();

  // Setup interrupt - trigger on RISING (carrier detected) or FALLING (carrier lost)
  pinMode(CC1101_GDO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), onPacketReceived, CHANGE);

  // Start receiving
  radio.setFrequency(FREQUENCY);
  radio.startRX();

  startTime = millis();
  lastHistogramDecay = millis();

  delay(500);

  // Draw main display
  display.clearForMain();
  display.draw(decoder, totalPackets, startTime, FREQUENCY, scanning, radio.getRSSI());

  Serial.println("TPMS Scanner ready!");
  Serial.println("Scanning 315 MHz (North America)...");
  Serial.println("Watch Serial Monitor for detected sensors.\n");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  unsigned long currentTime = millis();

  // Process received packets (from interrupt)
  if (packetReceived) {
    packetReceived = false;
    processInterruptPacket();
  }

  // Poll FIFO for data every 50ms
  static unsigned long lastFIFOPoll = 0;
  if (currentTime - lastFIFOPoll >= 50) {
    lastFIFOPoll = currentTime;
    pollFIFO();
  }

  // Debug: Print RSSI every 5 seconds
  static unsigned long lastRSSIPrint = 0;
  if (currentTime - lastRSSIPrint >= 5000) {
    lastRSSIPrint = currentTime;
    int8_t rssi = radio.getRSSI();
    uint8_t rxBytes = radio.readStatus(CC1101_RXBYTES) & 0x7F;
    Serial.printf("[STATUS] %d MHz | RSSI: %d dBm | FIFO: %d bytes | Pkts: %lu\n",
                  FREQUENCY, rssi, rxBytes, totalPackets);
  }

  // Update display
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdate = currentTime;

    // Prune old sensors
    decoder.pruneOldSensors();

    // Redraw display
    display.draw(decoder, totalPackets, startTime, FREQUENCY, scanning, radio.getRSSI());
  }

  // Decay histogram periodically
  if (currentTime - lastHistogramDecay >= HISTOGRAM_DECAY_INTERVAL_MS) {
    lastHistogramDecay = currentTime;
    decoder.decayHistogram();
  }

  // Handle BOOT button for display mode switching
  handleButton();

  yield();
}

// ============================================================================
// Packet Processing
// ============================================================================

void processInterruptPacket() {
  totalPackets++;

  // Read number of bytes in FIFO
  uint8_t rxBytes = radio.readStatus(CC1101_RXBYTES);

  if (rxBytes & 0x80) {  // Overflow
    radio.strobe(CC1101_SFRX);
    radio.strobe(CC1101_SRX);
    return;
  }

  if (rxBytes == 0) {
    radio.strobe(CC1101_SRX);
    return;
  }

  // Read RSSI first
  int8_t rssi = radio.getRSSI();

  // Read packet data
  uint8_t packet[64];
  int len = min((int)rxBytes, 64);
  radio.readFIFO(packet, len);

  // Debug output
  Serial.printf("RX [%d MHz] %d bytes, RSSI: %d dBm: ", FREQUENCY, len, rssi);
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();

  // Try to decode TPMS data
  decoder.decodePacket(packet, len, rssi, FREQUENCY);

  // Restart RX
  radio.strobe(CC1101_SFRX);
  radio.strobe(CC1101_SRX);
}

void pollFIFO() {
  uint8_t rxBytes = radio.readStatus(CC1101_RXBYTES) & 0x7F;
  bool overflow = radio.readStatus(CC1101_RXBYTES) & 0x80;

  if (overflow) {
    Serial.println("[WARN] FIFO overflow, flushing...");
    radio.strobe(CC1101_SFRX);
    radio.strobe(CC1101_SRX);
    return;
  }

  // With sync word 0x001A configured, we should only get data when
  // a valid TPMS preamble is detected
  if (rxBytes >= 6) {  // Minimum TPMS packet size
    int8_t rssi = radio.getRSSI();

    // Filter extremely weak signals (below noise floor)
    if (rssi < RSSI_NOISE_FLOOR) {
      radio.strobe(CC1101_SFRX);
      radio.strobe(CC1101_SRX);
      return;
    }

    totalPackets++;

    Serial.printf("\n>>> TPMS RX [%d MHz] %d bytes, RSSI: %d dBm\n", FREQUENCY, rxBytes, rssi);
    Serial.print("    RAW: ");

    // Read all bytes from FIFO
    uint8_t packet[64];
    int len = min((int)rxBytes, 64);
    radio.readFIFO(packet, len);

    // Print hex dump
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", packet[i]);
    }
    Serial.println();

    // Try Manchester decoding (TPMS typically uses Manchester encoding)
    uint8_t decoded[32];
    int decodedLen = decoder.manchesterDecode(packet, len, decoded, 32);

    if (decodedLen > 0) {
      Serial.print("    MANCHESTER: ");
      for (int i = 0; i < decodedLen; i++) {
        Serial.printf("%02X ", decoded[i]);
      }
      Serial.println();

      // Try to decode the Manchester-decoded data
      decoder.decodePacket(decoded, decodedLen, rssi, FREQUENCY);
    } else {
      // Try raw decode if Manchester fails
      decoder.decodePacket(packet, len, rssi, FREQUENCY);
    }

    // Flush and restart RX
    radio.strobe(CC1101_SFRX);
    radio.strobe(CC1101_SRX);
  }
}

// ============================================================================
// Button Handling
// ============================================================================

void handleButton() {
  bool buttonState = digitalRead(BOOT_BUTTON);
  unsigned long now = millis();

  // Trigger on button RELEASE (LOW to HIGH transition) - more reliable
  if (buttonState == HIGH && lastButtonState == LOW) {
    if (now - lastButtonPress >= BUTTON_DEBOUNCE_MS) {
      lastButtonPress = now;

      // Cycle to next display mode
      display.nextMode();

      // Force immediate display update
      display.forceRedraw();
      display.draw(decoder, totalPackets, startTime, FREQUENCY, scanning, radio.getRSSI());
    }
  }

  lastButtonState = buttonState;
}
