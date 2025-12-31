# CLAUDE.md

This file provides guidance to Claude Code when working with the TPMS Scanner project.

---
## IMPORTANT: ALWAYS USE deploy.sh

**NEVER run arduino-cli commands directly.** Always use the deploy.sh script:

```bash
./deploy.sh          # Compile and upload (DEFAULT)
./deploy.sh compile  # Compile only
./deploy.sh upload   # Upload only
./deploy.sh monitor  # Serial monitor (run in separate terminal)
./deploy.sh libs     # Install required libraries
```

---

## Project Overview

ESP32-based TPMS (Tire Pressure Monitoring System) scanner using CC1101 wireless module. Scans for tire pressure sensor transmissions on **315 MHz** (North America - USA, Canada, Mexico).

**Hardware:**
- ESP32 DevKit (30-pin) - standard board with all GPIO accessible
- CC1101 Wireless Module with SMA Antenna (315 MHz)
- 0.96" OLED Display (SSD1306 128x64, I2C) - for portable viewing

**Purpose:** Detect and decode TPMS sensor transmissions to display tire pressure (PSI) and temperature (Fahrenheit) data in real-time.

## Wiring Diagram

Using standard **ESP32 DevKit 30-pin** board with all GPIO pins accessible on headers.
No soldering required - just use jumper wires!

See **WIRING.md** for detailed step-by-step instructions with diagrams.

### CC1101 Quick Reference

```
CC1101 Pin    ESP32 Pin    Wire Color
────────────────────────────────────────
1. GND    →   GND          Black
2. VCC    →   3V3          Red
3. GDO0   →   GPIO 21      Brown
4. CSN    →   GPIO 5       White
5. SCK    →   GPIO 18      Grey
6. MOSI   →   GPIO 23      Purple
7. MISO   →   GPIO 19      Blue
8. GDO2   →   (not used)   -
```

### OLED Display Quick Reference

```
OLED Pin      ESP32 Pin    Wire Color
────────────────────────────────────────
VCC       →   3V3          Red
GND       →   GND          Black
SDA       →   GPIO 4       Yellow
SCL       →   GPIO 22      Orange
```

### CC1101 Module Pin Layout

```
    ┌──────────────────┐
    │ ①  ②            │
    │ ③  ④    ○ ○     │
    │ ⑤  ⑥            │
    │ ⑦  ⑧   [ANT]    │
    └──────────────────┘

1. GND     - Ground
2. VCC     - Power (3.3V)
3. GDO0    - Interrupt (GPIO 21)
4. CSN     - Chip Select (GPIO 5)
5. SCK     - SPI Clock (GPIO 18)
6. MOSI    - SPI Data In (GPIO 23)
7. MISO    - SPI Data Out (GPIO 19)
8. GDO2    - Not connected
```

## Features

- **315 MHz Scanning** - Dedicated to North American TPMS frequency
- **2-FSK Demodulation** - Proper FSK modulation for TPMS signals
- **Manchester Decoding** - Decodes Manchester-encoded sensor data
- **Scatter Plot Display** - Real-time PSI distribution visualization
  - Yellow zone (top 16px): Unique sensor count + total readings
  - Blue zone (bottom 48px): PSI scatter plot (20-60 PSI)
- **Visual Traffic Monitor** - Watch tire pressures from passing cars build up
- **Sensor Tracking** - Tracks up to 20 unique sensors
- **Multi-Protocol Support** - Toyota, Schrader, and Generic decoders
- **SPI Timeout Protection** - All SPI operations have 100ms timeout to prevent hangs
- **BOOT Button Display Modes** - Press BOOT button (GPIO 0) to cycle through 3 display modes

## TPMS Background

### What is TPMS?

Tire Pressure Monitoring Systems are mandatory in many countries. Each tire has a sensor that periodically transmits:
- Unique Sensor ID (28-32 bits typically)
- Tire Pressure (PSI or kPa)
- Temperature (Celsius, displayed as Fahrenheit)
- Battery Status (some sensors)

### Frequencies

| Frequency | Region | Notes |
|-----------|--------|-------|
| **315 MHz** | **North America** | **USA, Canada, Mexico (this scanner)** |
| 433.92 MHz | Europe, Asia, Australia | EU standard (not supported) |

### Supported Protocols

- **Toyota PMV-107J** - 2008+ Corolla, Prius, RAV4, etc.
- **Schrader** - Most common OEM supplier
- **Generic/Aftermarket** - Various sensors

## Development Commands

### Using deploy.sh (REQUIRED)

```bash
./deploy.sh          # Compile and upload (most common)
./deploy.sh compile  # Compile only
./deploy.sh upload   # Upload only
./deploy.sh monitor  # Serial monitor (run in separate terminal)
./deploy.sh libs     # Install Adafruit SSD1306 and GFX libraries
```

**Note:** User should run `./deploy.sh monitor` in their own terminal.

## CC1101 Configuration

The CC1101 is configured for **2-FSK modulation** (NOT OOK!) which is what TPMS sensors actually use:

| Setting | Value | Purpose |
|---------|-------|---------|
| Modulation | 2-FSK | Frequency Shift Keying |
| Sync Word | 0x001A | TPMS preamble pattern |
| Data Rate | 19.2 kbps | Standard TPMS rate |
| Bandwidth | 135 kHz | Narrow for better sensitivity |
| Deviation | 19 kHz | FSK frequency deviation |
| Frequency | 315 MHz | North America |

### Key Registers

| Register | Value | Purpose |
|----------|-------|---------|
| FREQ2/1/0 | 0x0C1D89 | 315 MHz carrier |
| MDMCFG2 | 0x02 | 2-FSK, 16/16 sync word |
| MDMCFG4 | 0x8A | 135 kHz bandwidth |
| MDMCFG3 | 0x83 | 19.2 kbps data rate |
| DEVIATN | 0x44 | 19 kHz deviation |
| SYNC1/0 | 0x001A | TPMS sync word |

### Frequency Calculation

```
Frequency = (FREQ2 << 16 + FREQ1 << 8 + FREQ0) * Fxtal / 2^16
Fxtal = 26 MHz (CC1101 crystal)

For 315 MHz: FREQ2=0x0C, FREQ1=0x1D, FREQ0=0x89
```

## Pin Mappings

### CC1101 (VSPI)

| Function | GPIO | Notes |
|----------|------|-------|
| MISO | 19 | CC1101 SPI |
| MOSI | 23 | CC1101 SPI |
| SCK | 18 | CC1101 SPI |
| CS | 5 | CC1101 chip select |
| GDO0 | 21 | Sync word interrupt |

### OLED Display (I2C)

| Function | GPIO | Notes |
|----------|------|-------|
| SDA | 4 | I2C Data |
| SCL | 22 | I2C Clock |
| Address | 0x3C | Default I2C address |

**Display Zones (0.96" SSD1306 128x64):**

The display has a dual-color layout with yellow at the top and blue below:

```
Y=0  ┌────────────────────────────┐
     │      YELLOW ZONE           │  ← Y=0 to Y=15 (16 pixels)
     │  Title text (Y=0-7)        │     Header area
     │  Separator line (Y=10)     │
Y=15 ├────────────────────────────┤
     │                            │
     │       BLUE ZONE            │  ← Y=16 to Y=63 (48 pixels)
     │   Content/data area        │     Main content area
     │                            │
Y=63 └────────────────────────────┘
```

- **Yellow zone (Y=0-15)**: Title and separator line only
- **Blue zone (Y=16-63)**: All data content starts at Y=16 to avoid overlap with yellow

### Button

| Function | GPIO | Notes |
|----------|------|-------|
| BOOT | 0 | Display mode button (built into ESP32 DevKit) |

## Architecture

### Core Components

1. **CC1101 Driver**
   - Raw SPI communication with CC1101 chip
   - 2-FSK demodulation at 315 MHz
   - Sync word detection (0x001A)
   - RSSI reading and filtering
   - FIFO buffer management
   - SPI timeout protection (100ms) with graceful recovery

2. **Manchester Decoder**
   - Converts Manchester-encoded bits to raw bytes
   - IEEE 802.3 convention (01=0, 10=1)

3. **TPMS Decoder**
   - Toyota PMV-107J format
   - Schrader format
   - Generic/Aftermarket format
   - Pressure scaling (raw * 2.5 kPa)
   - Temperature offset (raw - 40°C)

4. **Sensor Manager**
   - Tracks detected sensors
   - Merges duplicate detections
   - Prunes stale sensors (5 min timeout)
   - New sensor highlighting

5. **Display Renderer**
   - OLED scatter plot visualization
   - Yellow zone: unique sensor count + reading count
   - Blue zone: PSI histogram (20-60 PSI on X-axis)
   - Dots stack vertically as readings accumulate
   - Visual representation of "RF traffic" around you

### Data Flow

```
CC1101 Radio (315 MHz, 2-FSK)
    │
    ▼ (sync word 0x001A detected)
┌──────────────────┐
│  Read FIFO       │ ─── Get raw bytes + RSSI
└──────────────────┘
    │
    ▼
┌──────────────────┐
│ manchesterDecode │ ─── Convert to raw data
└──────────────────┘
    │
    ▼
┌──────────────────┐
│decodeTPMSPacket  │ ─── Try Toyota/Schrader/Generic
└──────────────────┘
    │
    ▼
┌──────────────────┐
│  updateSensor    │ ─── Add to sensor list + histogram
└──────────────────┘
    │
    ▼
┌──────────────────┐
│  drawDisplay     │ ─── Render scatter plot
└──────────────────┘
```

### Scatter Plot Display

```
┌────────────────────────────────┐
│ 141                       203  │  ← Yellow: sensors + readings
├────────────────────────────────┤
│ │          ▪▪                  │
│ │         ▪▪▪▪                 │
│ │        ▪▪▪▪▪▪  ▪             │  ← Blue: PSI scatter plot
│ │       ▪▪▪▪▪▪▪▪▪▪             │     Dots stack as readings
│ │      ▪▪▪▪▪▪▪▪▪▪▪▪            │     accumulate at each PSI
│ ├──────────────────────────────│
│ 20         40         60       │  ← X-axis: PSI (20-60)
└────────────────────────────────┘
```

- **X-axis**: Tire pressure 20-60 PSI (configurable via PSI_MIN/PSI_MAX)
- **Y-axis**: Count of readings at that pressure
- **Dots**: 2x2 pixels, stacked vertically
- Most readings cluster around 30-35 PSI (properly inflated passenger cars)
- Higher readings (42-50 PSI) typically from trucks/SUVs

### Histogram Decay

To prevent the scatter plot bars from maxing out during long runs, the histogram uses a **very slow decay** mechanism:

- **Decay Interval**: Every 5 minutes (HISTOGRAM_DECAY_INTERVAL_MS)
- **Decay Rate**: 3% reduction per interval (HISTOGRAM_DECAY_PERCENT)
- **Minimum Floor**: Bars never drop below 1 (HISTOGRAM_MIN_VALUE)

This creates a "rolling window" effect where:
- Active traffic keeps bars growing
- When away from cars, bars very slowly shrink over 1-2 hours
- Past readings leave a "ghost" (minimum of 1 dot) showing where traffic was detected

## Troubleshooting

### CC1101 Not Detected

**Symptom:** "CC1101 INIT FAILED" on screen

**Check:**
1. Wiring connections (especially VCC=3.3V, not 5V!)
2. SPI pins correct (GPIO 18, 19, 23 for VSPI)
3. CS pin (GPIO 5) connected
4. GDO0 pin (GPIO 21) connected
5. Module antenna connected

### SPI Timeout Errors

**Symptom:** `[ERROR] SPI timeout waiting for MISO` in serial output

**Cause:** The CC1101 module is not responding to SPI commands within 100ms.

**Check:**
1. Verify all SPI wiring connections are secure
2. Check for loose jumper wires or poor contact
3. Ensure VCC is 3.3V (5V will damage the module!)
4. Try power cycling the ESP32 and CC1101
5. Check if the CC1101 module is damaged or counterfeit

**Recovery:** The firmware automatically recovers from SPI timeouts by:
- Releasing the CS line to reset SPI state
- Flushing the RX FIFO
- Restarting receive mode

If timeouts persist, the module may need replacement.

### OLED Display Not Working

**Symptom:** OLED stays off or blank

**Check:**
1. I2C address is 0x3C (most common)
2. SDA connected to GPIO 4
3. SCL connected to GPIO 22
4. VCC is 3.3V (not 5V)
5. Try swapping SDA/SCL wires if no I2C device found

**Serial output should show:**
```
OLED pins: SDA=GPIO4, SCL=GPIO22
Scanning I2C bus...
  I2C device found at 0x3C
OLED Display initialized (128x64)
```

### No Sensors Detected

**Possible causes:**
1. **No TPMS nearby** - Need an actual car tire sensor within 10-20 feet
2. **Sensors asleep** - Start the car or drive briefly to wake sensors
3. **Antenna issue** - Connect the SMA antenna (315 MHz)!
4. **Weak signal** - Real sensors should give RSSI > -95 dBm

**Tips:**
- TPMS sensors transmit every 30-60 seconds when stationary
- Driving the car activates sensors (every 15-30 seconds)
- Tapping tires may wake motion-activated sensors
- Car should be within 10-20 feet for reliable reception

### Serial Debug

Use serial monitor to see raw packet data:
```bash
./deploy.sh monitor
```

**Normal output (no sensors):**
```
[STATUS] 315 MHz | RSSI: -105 dBm | FIFO: 0 bytes | Pkts: 0
```

**When TPMS signal received:**
```
>>> TPMS RX [315 MHz] 17 bytes, RSSI: -78 dBm
    RAW: C7 61 18 03 26 C8 3A 52 33 9E ED 33 B9 13 E7 7F 18
    MANCHESTER: 94 21 5A 71 5B E5 E1 D7
TPMS [Toyota]: ID=9421A57, P=227.5 kPa (33.0 PSI), T=55F
```

## Validation Ranges

The decoder validates sensor data to filter noise:

| Parameter | Valid Range | Notes |
|-----------|-------------|-------|
| Pressure | 150-350 kPa | 22-50 PSI |
| Temperature | -10°C to 60°C | 14°F to 140°F |
| RSSI | > -115 dBm | Filters signals below noise floor |

## Display Modes

Press the **BOOT button** (GPIO 0) to cycle through 3 display modes:

### Mode 1: Scatter Plot (default)

```
┌────────────────────────────────┐
│ 141                       203  │  ← Sensors / Readings
├────────────────────────────────┤
│ │        ▪▪▪▪▪▪                │
│ │       ▪▪▪▪▪▪▪▪               │  ← PSI distribution
│ ├──────────────────────────────│
│ 20         40         60       │  ← PSI scale
└────────────────────────────────┘
```

- **Yellow zone**: Two large numbers - unique sensors (left) and total readings (right)
- **Blue zone**: Histogram where X = PSI (20-60), dots stack up as counts increase
- Perfect for visualizing "RF traffic" while driving

### Mode 2: Sensor List

```
┌────────────────────────────────┐
│ SENSORS (4)                    │
│────────────────────────────────│
│ 9421A5  33  75F  +++  *        │  ← ID, PSI, Temp, Signal, New
│ 8B3C22  35  68F  ++            │
│ 1F7E91  31  72F  ++            │
│ A42B10  44  80F  +             │
└────────────────────────────────┘
```

- Shows last 4 detected sensors (most recent first)
- **ID**: Last 6 hex digits of sensor ID
- **PSI**: Tire pressure
- **Temp**: Temperature in Fahrenheit
- **Signal**: +++ (strong), ++ (medium), + (weak), - (very weak)
- **\***: New sensor indicator (detected within 30 seconds)

### Mode 3: Statistics

```
┌────────────────────────────────┐
│      STATISTICS                │
│────────────────────────────────│
│ Uptime: 23m45s                 │
│ Packets: 1247                  │
│ Valid:   203                   │
│ Sensors: 141 (4 active)        │
│ RSSI: -78 dBm                  │
└────────────────────────────────┘
```

- **Uptime**: Time since boot
- **Packets**: Total RF packets received
- **Valid**: Successfully decoded TPMS readings
- **Sensors**: Unique sensors ever seen (active = not timed out)
- **RSSI**: Current signal strength (live)

## Future Enhancements

- [ ] SD card logging
- [ ] WiFi upload to server
- [ ] More protocol support (Continental, Pacific)
- [ ] Sensor assignment to tire positions (FL, FR, RL, RR)
- [ ] Pressure alerts (low/high threshold)
- [ ] Historical data graphs

## Safety Note

This is a **passive receiver only**. It does not transmit or interfere with any vehicle systems. TPMS sensors broadcast openly on ISM bands.

## Project Files

```
tpm-scanner/
├── tpm-scanner.ino      # Main firmware
├── deploy.sh            # Build/deploy script
├── CLAUDE.md            # This documentation
├── WIRING.md            # Detailed wiring guide
├── pins-for-CC1101.jpg  # CC1101 pinout reference
└── build/               # Compiled output (gitignored)
```

## Library Dependencies

- **Adafruit_SSD1306** - OLED display driver (install via `./deploy.sh libs`)
- **Adafruit_GFX** - Graphics library for OLED
- **SPI** - Built into ESP32 core
- **Wire** - I2C for OLED (built into ESP32 core)

## Build Info

| Metric | Value |
|--------|-------|
| Board FQBN | esp32:esp32:esp32 |
| Frequency | 315 MHz (North America) |
| Modulation | 2-FSK (19.2 kbps) |
| Display | 0.96" OLED SSD1306 (128x64) |
| Temperature | Fahrenheit |
| Binary Size | ~322 KB |
| Compile Time | ~30-45 seconds |
| SPI Timeout | 100ms (SPI_TIMEOUT_MS) |
| PSI Range | 20-60 (PSI_MIN/PSI_MAX) |
| Button Debounce | 250ms (BUTTON_DEBOUNCE_MS) |
| Display Modes | 3 (Scatter, Sensors, Stats) |
| Histogram Decay | 3% every 5min (HISTOGRAM_DECAY_*) |
| Histogram Floor | 1 (keeps ghost of past readings) |
