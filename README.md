# LTC2944_BMS

An Arduino library for the **LTC2944 battery gas gauge IC**, supporting Li-ion,
LiFePO4, AGM, and GEL battery packs across a wide range of series-cell configurations.

Versions 1.x.x Designed for **ATmega328P** boards (Arduino Pro Mini, Nano, Uno).

Upcoming versions 2.x.x Designed for the Raspberry RP2040-Zero.

All versions are based on functionality found in commercial BMS systems from Victron, Mastervolt, and so on.
The excellent article ["A Closer Look at State of Charge (SOC) and State of Health (SOH) Estimation Techniques for Batteries from Analog Devices"](https://www.analog.com/en/resources/technical-articles/a-closer-look-at-state-of-charge-and-state-health-estimation-tech.html) has also inspired me.

[![Arduino Library Manager](https://img.shields.io/badge/Arduino-Library%20Manager-blue)](https://www.arduino.cc/reference/en/libraries/ltc2944_bms/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
![Platform: AVR](https://img.shields.io/badge/platform-AVR%20%2F%20ATmega328P-orange)

---

## Features

- State-of-charge (SOC) estimation using a piecewise voltage curve
- IR (internal-resistance) compensation so SOC is accurate under load
- Adaptive coulomb counting once the battery has been calibrated
- Automatic internal-resistance learning — improves accuracy over time without user intervention
- EEPROM persistence for calibration data and the learned resistance model (wear-leveled, CRC-verified)
- Battery-disconnect detection with EEPROM flag (survives power loss)
- Calibration state machine: zero offset → discharge to empty → charge to full
- Alarm bitfield for over/under voltage, temperature, over-current, and profile mismatch
- On the ATmega328P-based boards, all battery profiles are stored in flash (PROGMEM) to conserve RAM

## Currently being tested, part of v1.9.x

- Temperature capacity derating
RemainingMah and runtimeMinutes are scaled by a piecewise-linear derating factor: −20 °C → 60 %, 25 °C → 100 %, 60 °C → 95 %. SOC itself is unaffected; only derived energy outputs are derated.
- SOH (State of Health) estimation
Measured Cmax (from the ACR span between empty and full calibration anchors) is compared to rated capacity at the end of each full calibration cycle. SOH % persisted to EEPROM. Exposed via getSoh() and BmsMeasurement.soh
- Coulombic efficiency (η) correction
Separate charge and discharge efficiency factors (default ηc = 0.99, ηd = 1.00) are applied to a floating-point ACR shadow register. Prevents SOC drift from round-trip losses over many cycles. Configurable via setEfficiency().

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
