/*
 * TPMS Protocol Decoders
 *
 * Decodes various TPMS sensor protocols including:
 *   - Toyota PMV-107J
 *   - Schrader
 *   - Generic/Aftermarket
 *
 * Also includes Manchester decoder and sensor management.
 */

#ifndef TPMS_DECODERS_H
#define TPMS_DECODERS_H

#include <Arduino.h>

// ============================================================================
// Validation Constants
// ============================================================================

#define MIN_PRESSURE_KPA   150    // Minimum valid pressure (22 PSI)
#define MAX_PRESSURE_KPA   350    // Maximum valid pressure (50 PSI)
#define MIN_TEMPERATURE_C  -10    // Minimum valid temperature
#define MAX_TEMPERATURE_C  60     // Maximum valid temperature
#define RSSI_NOISE_FLOOR   -115   // Below this is noise

// ============================================================================
// Sensor Configuration
// ============================================================================

#define MAX_TRACKED_SENSORS 20
#define SENSOR_TIMEOUT_MS   300000   // 5 minutes timeout
#define NEW_SENSOR_THRESHOLD 30000   // 30 seconds "new" indicator

// Pressure histogram for scatter plot
#define PSI_MIN 20
#define PSI_MAX 60
#define PSI_BINS (PSI_MAX - PSI_MIN + 1)  // 41 bins (20-60 PSI)

// Histogram decay settings (prevents bars from maxing out over long runs)
#define HISTOGRAM_DECAY_INTERVAL_MS 300000  // Decay every 5 minutes
#define HISTOGRAM_DECAY_PERCENT     3       // Reduce by 3% each interval
#define HISTOGRAM_MIN_VALUE         1       // Floor value (keeps "ghost" of past readings)

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
// TPMS Decoder Class
// ============================================================================

class TPMSDecoder {
public:
  TPMSDecoder() : sensorCount(0), uniqueSensors(0), validPackets(0) {
    memset(psiHistogram, 0, sizeof(psiHistogram));
  }

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

  // Main packet decoder - tries multiple protocols
  void decodePacket(uint8_t* data, uint8_t len, int8_t rssi, uint16_t frequency) {
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

      // Validation: realistic tire pressure and temperature
      if (sensorId != 0 && sensorId != 0x0FFFFFFF &&
          pressure_kpa >= MIN_PRESSURE_KPA && pressure_kpa <= MAX_PRESSURE_KPA &&
          temperature_c >= MIN_TEMPERATURE_C && temperature_c <= MAX_TEMPERATURE_C) {
        validPackets++;
        protocol = "Toyota";
        decoded = true;

        float temp_f = temperature_c * 9.0 / 5.0 + 32.0;
        Serial.printf("TPMS [Toyota]: ID=%07X, P=%.1f kPa (%.1f PSI), T=%.0fF\n",
                      sensorId, pressure_kpa, pressure_kpa * 0.145038, temp_f);

        updateSensor(sensorId, pressure_kpa, temperature_c, rssi, frequency, protocol);
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
        temperature_c = tempRaw - 40.0;

        // Validation
        if (pressure_kpa >= MIN_PRESSURE_KPA && pressure_kpa <= MAX_PRESSURE_KPA &&
            temperature_c >= MIN_TEMPERATURE_C && temperature_c <= MAX_TEMPERATURE_C) {
          validPackets++;
          protocol = "Schrader";
          decoded = true;

          float temp_f = temperature_c * 9.0 / 5.0 + 32.0;
          Serial.printf("TPMS [Schrader]: ID=%08X, P=%.1f kPa (%.1f PSI), T=%.0fF\n",
                        sensorId, pressure_kpa, pressure_kpa * 0.145038, temp_f);

          updateSensor(sensorId, pressure_kpa, temperature_c, rssi, frequency, protocol);
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
        temperature_c = tempRaw - 40.0;

        if (pressure_kpa < 100) {
          pressure_kpa = pressureRaw * 4.0;  // Some use *4
        }
        if (temperature_c < -30) {
          temperature_c = tempRaw - 50.0;  // Try -50 offset
        }

        // Validation
        if (pressure_kpa >= MIN_PRESSURE_KPA && pressure_kpa <= MAX_PRESSURE_KPA &&
            temperature_c >= MIN_TEMPERATURE_C && temperature_c <= MAX_TEMPERATURE_C) {
          validPackets++;
          protocol = "Generic";
          decoded = true;

          float temp_f = temperature_c * 9.0 / 5.0 + 32.0;
          Serial.printf("TPMS [Generic]: ID=%08X, P=%.1f kPa (%.1f PSI), T=%.0fF\n",
                        sensorId, pressure_kpa, pressure_kpa * 0.145038, temp_f);

          updateSensor(sensorId, pressure_kpa, temperature_c, rssi, frequency, protocol);
        }
      }
    }
  }

  // Update or add a sensor
  void updateSensor(uint32_t id, float pressure_kpa, float temp_c, int8_t rssi,
                    uint16_t freq, const char* protocol) {
    unsigned long now = millis();
    float psi = pressure_kpa * 0.145038;

    // Update histogram for scatter plot (clamp to PSI_MIN-PSI_MAX range)
    int psiBin = constrain((int)psi - PSI_MIN, 0, PSI_BINS - 1);
    if (psiHistogram[psiBin] < 255) {  // Prevent overflow
      psiHistogram[psiBin]++;
    }

    // Check if sensor already exists
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

  // Remove sensors that haven't been seen recently
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

  // Decay histogram to prevent maxing out over long runs
  void decayHistogram() {
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

  // Accessors
  TPMSSensor* getSensors() { return sensors; }
  int getSensorCount() const { return sensorCount; }
  uint16_t getUniqueSensors() const { return uniqueSensors; }
  unsigned long getValidPackets() const { return validPackets; }
  uint8_t* getHistogram() { return psiHistogram; }

private:
  TPMSSensor sensors[MAX_TRACKED_SENSORS];
  int sensorCount;
  uint16_t uniqueSensors;
  unsigned long validPackets;
  uint8_t psiHistogram[PSI_BINS];
};

#endif // TPMS_DECODERS_H
