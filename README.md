# LTC2944 / INA228 BMS

An Arduino library for the **LTC2944 battery gas gauge IC** / **INA228 Power Monitor**, supporting Li-ion,
LiFePO4, AGM, and GEL battery packs across a wide range of series-cell configurations.

Versions 1.0.x Designed for **ATmega328P** boards (Arduino Pro Mini, Nano, Uno). with the LTC2944 board

Versions 1.1.x Designed for **ATmega328P** boards (Arduino Pro Mini, Nano, Uno). with support for the [Adafruit INA228 board](https://www.adafruit.com/product/5832)
For the INA228, I will use parts of Rob Tillaart's [INA228 library](https://github.com/RobTillaart/INA228).

### Update 16-4-2026: The INA228 version is currently under development.

Upcoming versions 2.x.x Designed for the Raspberry RP2040-Zero.

All versions are based on functionality found in commercial BMS systems from Victron, Mastervolt, and so on.
The excellent article ["A Closer Look at State of Charge (SOC) and State of Health (SOH) Estimation Techniques for Batteries from Analog Devices"](https://www.analog.com/en/resources/technical-articles/a-closer-look-at-state-of-charge-and-state-health-estimation-tech.html) has also inspired me.

[![Arduino Library Manager](https://img.shields.io/badge/Arduino-Library%20Manager-blue)](https://www.arduino.cc/reference/en/libraries/ltc2944_bms/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
![Platform: AVR](https://img.shields.io/badge/platform-AVR%20%2F%20ATmega328P-orange)

---

## Features

### ✅ SOC estimation using a piecewise voltage curve
Fully implemented. _socFromVoltageCurve() and _socFromOcvCurve() handle this. Li-ion and LiFePO4 use 11-point PROGMEM OCV tables; lead-acid falls back to a two-segment piecewise linear curve through vEmpty → vNominal → vFull.

### ✅ IR compensation so SOC is accurate under load
Fully implemented. _socFromVoltageCurve() removes the signed IR drop via compV = cellV - (currentA * irOhm * relax) where irOhm = _effectiveResistance(). A smooth interpolated relax factor (0 → 1 between 50 mA and 500 mA) prevents abrupt switching at low currents.

### ✅ Adaptive coulomb counting once calibrated
Fully implemented. _estimateCoulombSoc() uses _etaAcrAccum mapped against the calibrated empty/full ACR anchors. _adaptiveBlend() then weights coulomb vs. voltage SOC as a function of current magnitude — more current → more weight on coulomb counting.

###  ✅ Automatic internal-resistance learning
Fully implemented. _updateResistanceLearning() uses a dV/dI estimator with an EMA (α = 0.10), only firing on discharge step transitions (charge transitions are explicitly rejected). Confidence saturates at 12 samples. _effectiveResistance() returns 0 until confidence ≥ 50%.

### ✅ EEPROM persistence for calibration data and the learned resistance model (wear-leveled, CRC-verified)
Fully implemented. The learned model uses two alternating slots (EE_LEARNED_SLOT_A / B) with a monotonic sequence counter — this is the classic wear-leveling ping-pong pattern. Each LearnedRecord is protected by a CRC-8. Calibration data (empty/full voltage and raw ACR anchors) is stored with its own magic words. Load/save on begin() and periodically every 30 minutes.

### ✅ Battery-disconnect detection with EEPROM flag
Fully implemented. _checkDisconnectOnMeasurement() detects low voltage + near-zero current and writes DISCONNECT_MARKER (0xD1) to EE_DISCONNECT. _readDisconnectFlag() reads it back on boot. _clearDisconnectFlag() clears it when a valid voltage is seen again.

### ✅ Calibration state machine: zero offset → discharge to empty → charge to full
Fully implemented. The CalPhase enum covers CAL_NONE → CAL_ZERO → CAL_WAIT → CAL_DISCHARGE → CAL_CHARGE → CAL_DONE (plus CAL_ABORT). The zero phase averages 32 samples of the current register at rest to derive _currentOffset_mA. Discharge watches for vbat ≤ vEmpty. The charge phase detects full via voltage threshold, peak-drop detection, and charger-cutoff sensing.

### ✅ Alarm bitfield for over/under voltage, temperature, over-current, and profile mismatch
Fully implemented. _makeAlarms() produces an 11-bit field including ALARM_UNDER_VOLTAGE, ALARM_OVER_VOLTAGE, ALARM_ABS_UNDER_V, ALARM_ABS_OVER_V, ALARM_TEMPERATURE, ALARM_OVER_CURRENT, ALARM_CHARGE_CURRENT, ALARM_PROFILE_MISMATCH, ALARM_ACR_ROLLOVER, ALARM_SENSOR_FAIL, and ALARM_NO_PROFILE.

### ✅ Battery profiles stored in PROGMEM
Fully implemented. kProfiles[], kOcvLiion[], kOcvLfp[], and all profile name strings are declared with PROGMEM. _resolveProfile() uses memcpy_P and pgm_read_word to read them safely.

### ✅ Temperature capacity derating
Fully implemented and matches the exact spec. _tempDerateFactor() returns 0.60 at −20 °C, 1.00 at 25 °C, and 0.95 at 60 °C via two linear segments. Only remainingMah and runtimeMinutes are scaled; soc is not.

### ✅ SOH estimation
Fully implemented. At the end of CAL_CHARGE, the ACR span between the empty and full anchors is converted to mAh and compared to _capacityMah. The result is persisted to EE_SOH_VALUE (with its own magic word) and exposed via getSoh() and BmsMeasurement.soh.

### ✅ Coulombic efficiency (η) correction
Fully implemented. _updateEtaAccum() applies _etaCharge to positive ACR deltas and _etaDischarge to negative ones. The shadow register _etaAcrAccum is used everywhere in place of raw ACR for SOC calculations. Defaults: ηc = 0.99, ηd = 1.00. Configurable via setEfficiency().

### ✅ Self-discharge correction
Fully implemented. A configurable bleed rate (mAh/hour, default 0.05) is subtracted from the η-corrected ACR accumulator on every update tick. On boot, the elapsed time since the last EEPROM timestamp is used to back-apply the loss before any measurement is taken. Disabled by passing 0.0 to setSelfDischargeRate().

### ✅ ACR rollover detection
Fully implemented. The 16-bit ACR register wraps from 0xFFFF back to 0x0000 on a sufficiently discharged or long-running pack. The library detects this by watching for a large downward jump (_lastRawACR > 0x8000 && rawACR < 0x1000) and sets ALARM_ACR_ROLLOVER in the alarm bitfield. For that cycle, SOC falls back to voltage-only estimation to avoid a spurious reading from the corrupted coulomb count.

### ✅ Compile-time debug mode
Fully implemented. Setting LTC2944_BMS_ENABLE_DEBUG to 1 enables a BmsDebugState struct that captures per-cycle internals: final, voltage, and coulomb SOC with source label (VOLTAGE_BOOT, VOLTAGE_ROLLOVER, VOLTAGE_ONLY, BLEND), IR-compensated pack and cell OCV estimates, effective resistance, temperature derate factor, raw register values, and ACR boot-tick and rollover flags. The struct is retrieved via getDebugState() and printed via printDebugStatus(). When the flag is 0, the struct collapses to an empty type, adding zero RAM or flash overhead.

---

![LTC2944 bridge testing](https://raw.githubusercontent.com/rleusden/ESP32-BatteryGauge/main/images/ltc2944_bridge_testing.png)

---

## Installation

1. Download or clone this repository.
2. Copy the `LTC2944_BMS` folder into your Arduino `libraries` directory.
3. Restart the Arduino IDE.
4. Open **File → Examples → LTC2944_BMS → BasicBMS** to get started.

---

## Quick start

```cpp
#include <LTC2944_BMS.h>

LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);

    bms.setProfile(CHEM_LIION, 2);   // 3S Li-ion (11.1 V nominal)
    bms.begin();
}

void loop() {
    if (bms.update()) {
        Serial.print("SOC: "); Serial.print(bms.getSoc()); Serial.println("%");
        Serial.print("V:   "); Serial.println(bms.getVoltage());
    }
}
```

---

## Battery profiles

### Li-ion / Li-polymer (`CHEM_LIION`)

| classCode | Config | Nominal voltage |
|-----------|--------|-----------------|
| 0 | 1S | 3.7 V |
| 1 | 2S | 7.4 V |
| 2 | 3S | 11.1 V |
| 3 | 4S | 14.8 V |
| 4 | 6S | 22.2 V |
| 5 | 8S | 29.6 V |
| 6 | 12S | 44.4 V |
| 7 | 14S | 51.8 V |

### LiFePO4 (`CHEM_LIFEPO4`)

| classCode | Config | Nominal voltage |
|-----------|--------|-----------------|
| 0 | 1S | 3.2 V |
| 1 | 2S | 6.4 V |
| 2 | 3S | 9.6 V |
| 3 | 4S | 12.8 V |
| 4 | 6S | 19.2 V |
| 5 | 8S | 25.6 V |
| 6 | 12S | 38.4 V |
| 7 | 14S | 44.8 V |

### AGM lead-acid (`CHEM_AGM`) · GEL lead-acid (`CHEM_GEL`)

| classCode | Pack voltage |
|-----------|-------------|
| 0 | 12 V |
| 1 | 24 V |
| 2 | 48 V |

---

## API reference

### Configuration (call before `begin()`)

```cpp
void setProfile(BatteryChem chem, uint8_t classCode);
void setShuntResistor(float ohms);     // default 0.010 (10 mΩ)
void setPinLoadEnable(uint8_t pin);    // default 5
void setPinStatusLed(uint8_t pin);     // default 13
void setCurrentDeadband(float amps);   // default 0.030 A
```

### Initialisation

```cpp
bool begin();        // returns false if LTC2944 not found
bool reinitChip();   // re-init after I²C error
```

### Main loop

```cpp
bool update();       // call every loop(); returns true when a new reading is ready (≤ 1 Hz)
```

### Measurements

```cpp
int      getSoc();           // 0–100 %
float    getVoltage();       // Volts
float    getCurrent();       // Amperes (positive = charging)
float    getTemperature();   // °C
uint16_t getAlarms();        // bitfield of ALARM_* constants
bool     isCharging();
BmsMeasurement getMeasurement();  // all values in one struct
```

### Alarm constants

```
ALARM_SENSOR_FAIL     ALARM_NO_PROFILE     ALARM_UNDER_VOLTAGE
ALARM_OVER_VOLTAGE    ALARM_ABS_UNDER_V    ALARM_ABS_OVER_V
ALARM_TEMPERATURE     ALARM_OVER_CURRENT   ALARM_CHARGE_CURRENT
ALARM_PROFILE_MISMATCH
```

### Battery disconnect

```cpp
bool isBatteryDisconnected();  // true after disconnect is confirmed
```

### Calibration

```cpp
void     startCalibration();   // starts the zero→discharge→charge sequence
void     stopCalibration();
CalPhase getCalPhase();        // CAL_NONE, CAL_ZERO, CAL_WAIT, CAL_DISCHARGE, CAL_CHARGE, CAL_DONE, CAL_ABORT
bool     isCalibrated();       // true if EEPROM holds a valid empty/full ACR window
void     resetCalibration();   // erase calibration from EEPROM
```

### New battery / learned model

```cpp
void  resetForNewBattery();        // clears learned resistance and calibration
float getLearnedResistance();      // Ohms
float getLearningConfidence();     // 0.0–1.0
```

### Utility

```cpp
const char* getProfileName();      // e.g. "LIION_3S"
bool        isProfilePlausible();  // false if measured voltage is outside profile bounds
```

---

## EEPROM map

| Address | Size | Content |
|---------|------|---------|
| 0x00 | 4 | Current-offset magic |
| 0x04 | 4 | Current offset (float, mA) |
| 0x08 | 1 | Battery disconnect flag |
| 0x20 | 4 | Calibration magic |
| 0x24 | 4 | Empty voltage (float) |
| 0x28 | 4 | Full voltage (float) |
| 0x2C | 4 | Empty ACR raw + magic |
| 0x30 | 4 | Full ACR raw + magic |
| 0xA0 | ~28 | Learned model slot A |
| 0xBC | ~28 | Learned model slot B |

Total EEPROM used: ~220 bytes of the ATmega328P's 1024 bytes.

---

## License

MIT — see [LICENSE](LICENSE).
