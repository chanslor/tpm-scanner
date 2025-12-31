# TPMS Scanner Wiring Guide - ESP32 DevKit

## Overview

Using a standard **ESP32 DevKit 30-pin** board with all GPIO pins accessible on headers.
**No soldering required!** Just use jumper wires.

This scanner receives **315 MHz TPMS signals** (North America) using **2-FSK modulation** and displays a real-time **scatter plot** of tire pressures on an OLED display - watch the invisible RF "aura" of traffic around you!

## Components

1. **CC1101 Wireless Module** - 315 MHz RF transceiver for TPMS signals (2-FSK)
2. **0.96" OLED Display (SSD1306)** - 128x64 I2C display for portable viewing
3. **315 MHz Antenna** - SMA antenna (~24cm / 9.5 inches for quarter wavelength)

---

## Wiring Diagram

```
    ╔═══════════════════════════════════════════════════════════════════════╗
    ║                     ESP32 DevKit 30-Pin                               ║
    ║                                                                       ║
    ║              ┌───────────────────────────┐                            ║
    ║              │  ┌─────────────────────┐  │                            ║
    ║              │  │    ESP32-WROOM-32   │  │                            ║
    ║              │  │                     │  │                            ║
    ║              │  └─────────────────────┘  │                            ║
    ║              │                           │                            ║
    ║    3V3  [ ]──┼───────────────────────────┼──[ ] VIN                   ║
    ║    GND  [ ]──┼───────────────────────────┼──[ ] GND                   ║
    ║    D15  [ ]  │                           │  [ ] D13                   ║
    ║    D2   [ ]  │                           │  [ ] D12                   ║
    ║    D4   [ ]  │                           │  [ ] D14                   ║
    ║    RX2  [ ]  │                           │  [ ] D27                   ║
    ║    TX2  [ ]  │                           │  [ ] D26                   ║
    ║    D5   [ ]──┼── White (CS) ─────────────┼──[ ] D25                   ║
    ║    D18  [ ]──┼── Grey (SCK) ──────────────┼──[ ] D33                   ║
    ║    D19  [ ]──┼── Blue (MISO) ─────────────┼──[ ] D32                   ║
    ║    D21  [ ]──┼── Brown (GDO0) ────────────┼──[ ] D35                   ║
    ║    RX0  [ ]  │                           │  [ ] D34                   ║
    ║    TX0  [ ]  │                           │  [ ] VN                    ║
    ║    D22  [ ]  │                           │  [ ] VP                    ║
    ║    D23  [ ]──┼── Purple (MOSI) ───────────┼──[ ] EN                    ║
    ║              │                           │                            ║
    ║              │      [  USB  ]            │                            ║
    ║              └───────────────────────────┘                            ║
    ║                                                                       ║
    ╚═══════════════════════════════════════════════════════════════════════╝
                            │ │ │ │ │ │
                            │ │ │ │ │ │
    ╔═══════════════════════╧═╧═╧═╧═╧═╧═════════════════════════════════════╗
    ║                                                                       ║
    ║                       CC1101 Module                                   ║
    ║                                                                       ║
    ║    ┌─────────────────────────────────────────────────────────────┐    ║
    ║    │                                                             │    ║
    ║    │    ①  ②         [○ ○]                                      │    ║
    ║    │    ③  ④                                                     │    ║
    ║    │    ⑤  ⑥                                                     │    ║
    ║    │    ⑦  ⑧        [ANTENNA]                                   │    ║
    ║    │    │  │             ║                                       │    ║
    ║    └────┼──┼─────────────╨───────────────────────────────────────┘    ║
    ║         │  │                                                          ║
    ║         │  │                                                          ║
    ║    ┌────┴──┴─────────────────────────────────────────────────────┐    ║
    ║    │                                                             │    ║
    ║    │   Pin 1 (GND)  ◄─── Black ─────  GND                        │    ║
    ║    │   Pin 2 (VCC)  ◄─── Red ───────  3V3 (3.3V)                 │    ║
    ║    │   Pin 3 (GDO0) ◄─── Brown ─────  D21 (GPIO 21)              │    ║
    ║    │   Pin 4 (CSN)  ◄─── White ─────  D5  (GPIO 5)               │    ║
    ║    │   Pin 5 (SCK)  ◄─── Grey ──────  D18 (GPIO 18)              │    ║
    ║    │   Pin 6 (MOSI) ◄─── Purple ────  D23 (GPIO 23)              │    ║
    ║    │   Pin 7 (MISO) ◄─── Blue ──────  D19 (GPIO 19)              │    ║
    ║    │   Pin 8 (GDO2)      (not connected)                         │    ║
    ║    │                                                             │    ║
    ║    └─────────────────────────────────────────────────────────────┘    ║
    ║                                                                       ║
    ╚═══════════════════════════════════════════════════════════════════════╝
```

---

## Connection Table

| CC1101 Pin | Function | ESP32 Pin | Wire Color |
|------------|----------|-----------|------------|
| 1 | GND | GND | Black |
| 2 | VCC | 3V3 | Red |
| 3 | GDO0 | GPIO 21 (D21) | Brown |
| 4 | CSN | GPIO 5 (D5) | White |
| 5 | SCK | GPIO 18 (D18) | Grey |
| 6 | MOSI | GPIO 23 (D23) | Purple |
| 7 | MISO | GPIO 19 (D19) | Blue |
| 8 | GDO2 | (not used) | - |

---

## CC1101 Module Pinout

```
    CC1101 Module (looking at pins)
    ┌──────────────────────────────┐
    │                              │
    │  ① GND        ② VCC         │
    │                              │
    │  ③ GDO0       ④ CSN         │
    │                              │
    │  ⑤ SCK        ⑥ MOSI        │
    │                              │
    │  ⑦ MISO       ⑧ GDO2        │
    │                              │
    │         [SMA ANTENNA]        │
    │              ║               │
    └──────────────╨───────────────┘
```

---

## Step-by-Step Wiring

### 1. Power Connections (CRITICAL - DO FIRST)

```
CC1101 Pin 1 (GND) ──── Black wire ────► ESP32 GND
CC1101 Pin 2 (VCC) ──── Red wire ──────► ESP32 3V3

⚠️  WARNING: VCC must be 3.3V, NOT 5V!
    Using 5V will damage the CC1101 module!
```

### 2. SPI Bus Connections

```
CC1101 Pin 5 (SCK)  ──── Grey wire ────► ESP32 D18 (GPIO 18)
CC1101 Pin 6 (MOSI) ──── Purple wire ──► ESP32 D23 (GPIO 23)
CC1101 Pin 7 (MISO) ──── Blue wire ────► ESP32 D19 (GPIO 19)
CC1101 Pin 4 (CSN)  ──── White wire ───► ESP32 D5  (GPIO 5)
```

### 3. Interrupt Connection

```
CC1101 Pin 3 (GDO0) ──── Brown wire ───► ESP32 D21 (GPIO 21)
```

### 4. Attach Antenna

```
⚠️  IMPORTANT: Connect SMA antenna before powering on!
    Operating without antenna can damage the RF module.
```

---

## OLED Display Wiring (Optional)

The 0.96" SSD1306 OLED (128x64) uses I2C and only needs 4 wires:

```
    OLED Module               ESP32 DevKit
    ┌───────────┐
    │  SSD1306  │
    │  128x64   │
    │           │
    │ SDA ──────┼───── Yellow ────► D4 (GPIO 4)
    │ SCL ──────┼───── Orange ────► D22 (GPIO 22)
    │ VCC ──────┼───── Red    ────► 3V3
    │ GND ──────┼───── Black  ────► GND
    │           │
    └───────────┘
```

### OLED Connection Table

| OLED Pin | Function | ESP32 Pin | Wire Color |
|----------|----------|-----------|------------|
| SDA | I2C Data | GPIO 4 (D4) | Yellow |
| SCL | I2C Clock | GPIO 22 (D22) | Orange |
| VCC | Power | 3V3 | Red |
| GND | Ground | GND | Black |

---

## Quick Visual Reference

```
    ESP32 Pin      Wire        CC1101 Pin
    ─────────────────────────────────────
       GND    ════ Black  ═══   Pin 1 (GND)
       3V3    ════ Red    ═══   Pin 2 (VCC)
       D21    ════ Brown  ═══   Pin 3 (GDO0)
       D5     ════ White  ═══   Pin 4 (CSN)
       D18    ════ Grey   ═══   Pin 5 (SCK)
       D23    ════ Purple ═══   Pin 6 (MOSI)
       D19    ════ Blue   ═══   Pin 7 (MISO)
                                Pin 8 (not used)
```

### OLED Display (I2C)
```
    ESP32 Pin      Wire        OLED Pin
    ─────────────────────────────────────
       GND    ════ Black  ═══   GND
       3V3    ════ Red    ═══   VCC
       D4     ════ Yellow ═══   SDA
       D22    ════ Orange ═══   SCL
```

---

## Verification Checklist

Before powering on:

**CC1101 Module:**
- [ ] VCC connected to **3.3V** (NOT 5V!)
- [ ] GND connected to GND
- [ ] All 5 signal wires connected (GDO0, CSN, SCK, MOSI, MISO)
- [ ] Antenna attached to CC1101 SMA connector
- [ ] No short circuits between adjacent pins
- [ ] Wires firmly seated in headers

**OLED Display (if using):**
- [ ] VCC connected to **3.3V**
- [ ] GND connected to GND
- [ ] SDA connected to GPIO 4 (D4)
- [ ] SCL connected to GPIO 22 (D22)

---

## Troubleshooting

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| "CC1101 INIT FAILED" | SPI wiring wrong | Double-check SCK, MOSI, MISO, CS pins |
| Part number 0xFF | No power or bad ground | Check VCC and GND connections |
| Part number wrong | Wrong SPI pins | Verify GPIO 18, 19, 23, 5 |
| No sensors detected | Normal - need TPMS nearby | Drive near a car, or wait |
| Weak/no signal | Antenna not connected | Attach SMA antenna! |

---

## Serial Monitor Output

Once working, you'll see output like:

```
=================================
    TPMS Scanner v1.0
    ESP32 DevKit + CC1101
=================================

OLED pins: SDA=GPIO4, SCL=GPIO22
Scanning I2C bus...
  I2C device found at 0x3C
OLED Display initialized (128x64)
Initializing CC1101 radio module...
CC1101 Part: 0x00, Version: 0x14
CC1101 initialized successfully
CC1101 configured for 315 MHz
TPMS Scanner ready!
Scanning 315 MHz (North America)...

[STATUS] 315 MHz | RSSI: -105 dBm | FIFO: 0 bytes | Pkts: 0

>>> TPMS RX [315 MHz] 17 bytes, RSSI: -78 dBm
    RAW: C7 61 18 03 26 C8 3A 52 33 9E ED 33 B9 13 E7 7F 18
    MANCHESTER: 94 21 5A 71 5B E5 E1 D7
TPMS [Toyota]: ID=9421A57, P=227.5 kPa (33.0 PSI), T=55F
New sensor added: ID=09421A57 (total: 1)
```

---

## Technical Notes

### RF Configuration

The CC1101 is configured for **2-FSK modulation** (NOT OOK):

| Parameter | Value |
|-----------|-------|
| Frequency | 315 MHz |
| Modulation | 2-FSK |
| Data Rate | 19.2 kbps |
| Bandwidth | 135 kHz |
| Deviation | 19 kHz |
| Sync Word | 0x001A |

### Display Output - Scatter Plot

The OLED shows a real-time **scatter plot** of tire pressures:

```
┌────────────────────────────────┐
│ 12        47 rdgs              │  ← Unique sensors + readings
├────────────────────────────────┤
│ │          ▪▪                  │
│ │         ▪▪▪▪                 │
│ │        ▪▪▪▪▪▪  ▪             │  ← Dots stack as readings
│ │       ▪▪▪▪▪▪▪▪▪▪             │     accumulate at each PSI
│ ├──────────────────────────────│
│ 20         40         60       │  ← PSI scale (20-60)
└────────────────────────────────┘
```

- **Yellow zone**: Large number = unique sensors, right side = total readings
- **Blue zone**: PSI histogram - X axis is pressure (20-60 PSI)
- **Dots**: Stack vertically as more readings come in at each PSI level
- **Decay**: Bars very slowly shrink (3% every 5 min) to prevent maxing out, keeping a "ghost" of past readings
- Drive in traffic to see the distribution build up around 30-35 PSI!
