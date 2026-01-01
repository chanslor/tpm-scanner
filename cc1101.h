/*
 * CC1101 Radio Driver
 *
 * Low-level driver for the CC1101 wireless transceiver module.
 * Configured for 315 MHz TPMS reception with 2-FSK modulation.
 */

#ifndef CC1101_H
#define CC1101_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================================
// Pin Definitions - CC1101 Module
// ============================================================================

// CC1101 connections for standard ESP32 DevKit (30-pin):
// Using VSPI bus (directly on headers):
//   IO18 = SCK, IO19 = MISO, IO23 = MOSI, IO5 = CS
//   IO21 = GDO0 (interrupt)

#define CC1101_CS     5     // Chip Select (directly on header)
#define CC1101_GDO0   21    // Interrupt (directly on header)
#define CC1101_SCK    18    // SPI Clock (VSPI default)
#define CC1101_MOSI   23    // SPI MOSI (VSPI default)
#define CC1101_MISO   19    // SPI MISO (VSPI default)

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
// SPI Timeout Configuration
// ============================================================================

#define SPI_TIMEOUT_MS 100

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

static const uint8_t CC1101_CONFIG_315[] = {
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
// CC1101 Driver Class
// ============================================================================

class CC1101 {
public:
  CC1101() : _spi(VSPI) {}

  // Initialize the CC1101 module
  bool init() {
    _spi.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    pinMode(CC1101_CS, OUTPUT);
    digitalWrite(CC1101_CS, HIGH);

    // Reset CC1101
    reset();
    delay(100);

    // Verify CC1101 is present
    uint8_t partnum = readStatus(CC1101_PARTNUM);
    uint8_t version = readStatus(CC1101_VERSION);

    Serial.printf("CC1101 Part: 0x%02X, Version: 0x%02X\n", partnum, version);

    // CC1101 should return 0x00 for partnum, version varies (0x04, 0x14, etc.)
    if (partnum != 0x00 || version == 0x00 || version == 0xFF) {
      Serial.println("ERROR: Invalid CC1101 response!");
      return false;
    }

    // Load 315 MHz configuration (North America)
    for (size_t i = 0; i < sizeof(CC1101_CONFIG_315); i++) {
      writeReg(i, CC1101_CONFIG_315[i]);
    }

    Serial.println("CC1101 initialized successfully");
    return true;
  }

  // Reset the CC1101 module
  void reset() {
    digitalWrite(CC1101_CS, LOW);
    delayMicroseconds(10);
    digitalWrite(CC1101_CS, HIGH);
    delayMicroseconds(40);

    digitalWrite(CC1101_CS, LOW);
    if (!waitMISO()) {
      digitalWrite(CC1101_CS, HIGH);
      return;
    }

    _spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    _spi.transfer(CC1101_SRES);
    _spi.endTransaction();

    if (!waitMISO()) {
      digitalWrite(CC1101_CS, HIGH);
      return;
    }
    digitalWrite(CC1101_CS, HIGH);
  }

  // Write a register value
  void writeReg(uint8_t addr, uint8_t value) {
    digitalWrite(CC1101_CS, LOW);
    if (!waitMISO()) {
      digitalWrite(CC1101_CS, HIGH);
      return;
    }

    _spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    _spi.transfer(addr);
    _spi.transfer(value);
    _spi.endTransaction();

    digitalWrite(CC1101_CS, HIGH);
  }

  // Read a register value
  uint8_t readReg(uint8_t addr) {
    uint8_t value = 0;

    digitalWrite(CC1101_CS, LOW);
    if (!waitMISO()) {
      digitalWrite(CC1101_CS, HIGH);
      return 0xFF;
    }

    _spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    _spi.transfer(addr | CC1101_READ_SINGLE);
    value = _spi.transfer(0);
    _spi.endTransaction();

    digitalWrite(CC1101_CS, HIGH);
    return value;
  }

  // Read a status register
  uint8_t readStatus(uint8_t addr) {
    uint8_t value = 0;

    digitalWrite(CC1101_CS, LOW);
    if (!waitMISO()) {
      digitalWrite(CC1101_CS, HIGH);
      return 0xFF;
    }

    _spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    _spi.transfer(addr | CC1101_READ_BURST);
    value = _spi.transfer(0);
    _spi.endTransaction();

    digitalWrite(CC1101_CS, HIGH);
    return value;
  }

  // Send a strobe command
  void strobe(uint8_t cmd) {
    digitalWrite(CC1101_CS, LOW);
    if (!waitMISO()) {
      digitalWrite(CC1101_CS, HIGH);
      return;
    }

    _spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    _spi.transfer(cmd);
    _spi.endTransaction();

    digitalWrite(CC1101_CS, HIGH);
  }

  // Set frequency (loads 315 MHz config)
  void setFrequency(uint16_t freqMHz) {
    // Go to idle state first
    strobe(CC1101_SIDLE);

    // Load 315 MHz config (North America)
    for (size_t i = 0; i < sizeof(CC1101_CONFIG_315); i++) {
      writeReg(i, CC1101_CONFIG_315[i]);
    }

    Serial.printf("CC1101 configured for %d MHz\n", freqMHz);
  }

  // Start receiving
  void startRX() {
    strobe(CC1101_SFRX);   // Flush RX FIFO
    strobe(CC1101_SRX);    // Enter RX mode
  }

  // Get RSSI in dBm
  int8_t getRSSI() {
    uint8_t rssi_raw = readStatus(CC1101_RSSI);
    int16_t rssi_dbm;

    if (rssi_raw >= 128) {
      rssi_dbm = ((int16_t)rssi_raw - 256) / 2 - 74;
    } else {
      rssi_dbm = rssi_raw / 2 - 74;
    }

    return (int8_t)rssi_dbm;
  }

  // Read bytes from RX FIFO
  int readFIFO(uint8_t* buffer, int maxLen) {
    digitalWrite(CC1101_CS, LOW);
    if (!waitMISO()) {
      digitalWrite(CC1101_CS, HIGH);
      return 0;
    }

    _spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    _spi.transfer(CC1101_RXFIFO | CC1101_READ_BURST);

    int len = min(maxLen, 64);
    for (int i = 0; i < len; i++) {
      buffer[i] = _spi.transfer(0);
    }

    _spi.endTransaction();
    digitalWrite(CC1101_CS, HIGH);

    return len;
  }

  // Get SPI object for direct access if needed
  SPIClass& getSPI() { return _spi; }

private:
  SPIClass _spi;

  // Wait for MISO to go low with timeout
  bool waitMISO() {
    uint32_t timeout = millis() + SPI_TIMEOUT_MS;
    while (digitalRead(CC1101_MISO)) {
      if (millis() >= timeout) {
        Serial.println("[ERROR] SPI timeout waiting for MISO");
        return false;
      }
    }
    return true;
  }
};

#endif // CC1101_H
