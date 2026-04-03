# LTC2944_BMS Library Reference

Version 1.0.8 — Arduino library for the LTC2944 battery gas gauge IC.

Designed for **ATmega328P** boards (Arduino Pro Mini, Nano, Uno).

---

## How the library works

The LTC2944 is a battery gas gauge IC that measures voltage, current, temperature, and integrates current over time in a 16-bit accumulated charge register (ACR). The library wraps all chip communication and builds several layers of intelligence on top.

### SOC estimation

State of charge is estimated using two complementary methods that are blended together:

**Voltage curve SOC** maps the measured pack voltage to a piecewise curve calibrated for each battery chemistry. IR compensation is applied using the learned internal resistance — under load the voltage sag is corrected back to the open-circuit equivalent before reading the curve. This method works immediately at boot but is imprecise on flat-curve chemistries (LiFePO4) and under heavy load.

**Coulomb SOC** uses the LTC2944 ACR register as a hardware integrator. Current flows through the shunt resistor, the chip integrates it continuously, and the register position within the calibrated empty–full window gives a direct mAh-accurate SOC. This method requires a completed calibration run to establish the empty and full ACR anchor points.

At runtime both methods are blended adaptively. At low current the voltage curve gets more weight (the pack is relaxed and voltage is accurate). At high current the coulomb count dominates (voltage is depressed by IR drop and less reliable).

### Calibration

A full discharge–charge cycle is required once to establish the ACR anchors. The calibration state machine runs through five phases automatically: zero current offset → settle → discharge to empty → charge to full → done. Results are saved to EEPROM and survive power cycles.

### Internal resistance learning

After calibration, the library observes voltage steps when current changes (e.g. load connects or disconnects) and uses these to build a running estimate of the battery's internal resistance. Confidence grows with each sample. Once confidence exceeds 50% the learned resistance is used for IR compensation, replacing the initial zero assumption. The model is saved to EEPROM every 30 minutes using a wear-levelled two-slot scheme with CRC verification.

### EEPROM layout

| Address | Size | Content |
|---------|------|---------|
| 0x00 | 4 | Offset magic |
| 0x04 | 4 | Current offset (float, mA) |
| 0x08 | 1 | Battery disconnect flag |
| 0x20 | 4 | Calibration magic |
| 0x24 | 4 | Empty voltage (float, V) |
| 0x28 | 4 | Full voltage (float, V) |
| 0x2C | 2 | Empty ACR raw |
| 0x2E | 2 | Empty ACR magic |
| 0x30 | 2 | Full ACR raw |
| 0x32 | 2 | Full ACR magic |
| 0xA0 | 27 | Learned model slot A |
| 0xBB | 27 | Learned model slot B |

Total EEPROM used: ~214 bytes of the ATmega328P's 1024 bytes.

---

## Quick start

```cpp
#include <LTC2944_BMS.h>

LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);
    bms.setProfile(CHEM_LIION, 2);   // 2S Li-ion (7.4 V nominal)
    bms.setCapacity(4200);           // 4200 mAh
    bms.setShuntResistor(0.001f);    // 1 mΩ shunt
    bms.begin();
}

void loop() {
    if (bms.update()) bms.print(Serial);
}
```

---

## Configuration

Call all configuration methods before `begin()`.

### `void setProfile(BatteryChem chem, uint8_t classCode)`

Selects the battery chemistry and pack configuration. Must be called before `begin()`.

For Li-ion and LiFePO4, `classCode` is the number of series cells. For AGM and GEL, `classCode` is the pack voltage in volts.

| Chemistry | classCode | Example |
|-----------|-----------|---------|
| `CHEM_LIION` | 1, 2, 3, 4, 6, 8, 12, 14 | `setProfile(CHEM_LIION, 2)` → 2S, 7.4 V |
| `CHEM_LIFEPO4` | 1, 2, 3, 4, 6, 8, 12, 14 | `setProfile(CHEM_LIFEPO4, 4)` → 4S, 12.8 V |
| `CHEM_AGM` | 12, 24, 48 | `setProfile(CHEM_AGM, 24)` → 24 V |
| `CHEM_GEL` | 12, 24, 48 | `setProfile(CHEM_GEL, 48)` → 48 V |

### `void setCapacity(uint16_t capacityMah)`

Sets the nominal battery capacity in mAh. Enables remaining capacity and runtime estimation, and selects the correct LTC2944 prescaler for accurate coulomb counting. Leave unset (0) if capacity is unknown — SOC will still work but remaining mAh and runtime will not be shown.

### `void setShuntResistor(float ohms)`

Sets the shunt resistor value in Ohms. Default: `0.001` (1 mΩ). Must match the physical shunt on the hardware. This value is used for current measurement and coulomb counting scaling.

### `void setFullVoltage(float volts)`

Overrides the full-charge voltage for the pack. Useful when a charger terminates below the profile default (e.g. 8.30 V instead of 8.40 V on a 2S pack). Set to `0` to use the profile default.

### `void setEmptyVoltage(float volts)`

Overrides the empty voltage for the pack. Set to `0` to use the profile default.

### `void setOutputMode(OutputMode mode)`

Selects the serial output format used by `print()`.

| Mode | Format |
|------|--------|
| `OUTPUT_SERIAL` | Human-readable (default): `SOC=65%  V=7.4V  I=-748mA ...` |
| `OUTPUT_CSV` | Comma-separated, header printed once in `begin()` |
| `OUTPUT_ESPHOME` | Same as CSV but terminated with `\r\n` for ESPHome UART parsing |

### `void setPinLoadEnable(uint8_t pin)`

Sets the digital output pin used to enable the discharge load during calibration. Default: pin 5.

### `void setPinStatusLed(uint8_t pin)`

Sets the digital output pin used for the status LED. The LED is on during active calibration phases. Default: pin 13.

### `void setCurrentDeadband(float amps)`

Sets the current deadband in Amperes. Readings below this threshold are treated as zero. Default: `0.030` A (30 mA). Increase if noise causes false charging/discharging detection.

### `void setCalibrationTimeout(uint32_t minutes)`

Sets the maximum time allowed for each calibration phase (discharge and charge). Default: 240 minutes. Increase for large packs with low discharge current. Rule of thumb: `(capacityMah / dischargeMa) * 60 * 1.2`.

### `void setCalibrationFullCurrent(float amps)`

Sets the current taper threshold for PATH A full-charge detection during calibration. The library considers the battery full when charge current drops below this value for five consecutive seconds. Default: `0.10` A. Increase for chargers that terminate at a higher current.

### `void setCalibrationChargerOnCurrent(float amps)`

Sets the minimum current that indicates a charger is connected during calibration. Default: `0.030` A. Used to detect if a charger is accidentally connected during the discharge phase.

### `void setMaxDischargeCurrent(float amps)`

Sets the discharge current threshold for the `ALARM_OVER_CURRENT` alarm. Default: `5.5` A.

### `void setMaxChargeCurrent(float amps)`

Sets the charge current threshold for the `ALARM_CHARGE_CURRENT` alarm. Default: `3.0` A.

---

## Initialisation

### `bool begin()`

Initialises the library. Loads calibration and offset data from EEPROM, resolves the battery profile, configures the LTC2944 prescaler, and seeds the ACR register to a voltage-estimated position so SOC is immediately meaningful. Returns `false` if the LTC2944 is not found on the I2C bus.

### `bool reinitChip()`

Re-initialises the LTC2944 after an I2C error. Rewrites the prescaler and control register. Returns `false` if the chip does not respond. Also called internally after two consecutive I2C failures.

---

## Main loop

### `bool update()`

Reads all LTC2944 registers, updates SOC, alarms, IR learning, and the learned model. Returns `true` once per second when a new measurement is ready. Call this every `loop()` iteration. Returns `false` during the three-tick boot-settle window, on I2C failure, or after battery disconnect is confirmed.

---

## Output

### `void print(Stream& port)`

Prints the latest measurement to any Stream (Serial, SoftwareSerial, etc.) in the format selected by `setOutputMode()`. Only meaningful after `update()` returns `true`.

### `void printCsvHeader(Stream& port)`

Prints the CSV column header line. Called automatically by `begin()` when `OUTPUT_CSV` is selected.

### `void printDebug(Stream& port)`

Prints a full diagnostic dump including EEPROM calibration values, RAM calibration state, live LTC2944 register reads, ACR seed value, and boot-tick counter. Useful for verifying calibration integrity after a first flash or troubleshooting unexpected SOC behaviour. Call once in `setup()` after `begin()`.

---

## Measurement accessors

All accessors reflect the most recent `update()` call.

| Method | Returns | Description |
|--------|---------|-------------|
| `int getSoc()` | 0–100 | State of charge in percent |
| `float getVoltage()` | Volts | Pack voltage |
| `float getCurrent()` | Amperes | Charge/discharge current. Positive = charging, negative = discharging |
| `float getTemperature()` | °C | LTC2944 die temperature |
| `uint16_t getAlarms()` | bitfield | Active alarm flags — see alarm constants below |
| `bool isCharging()` | bool | `true` when charge current is detected |
| `uint16_t getRemainingMah()` | mAh | Estimated remaining capacity. Zero if capacity not set |
| `int16_t getRuntimeMinutes()` | minutes | Estimated runtime at current discharge rate. `-1` if unknown |
| `uint16_t getCapacityMah()` | mAh | Configured capacity as set by `setCapacity()` |
| `uint8_t getPrescaler()` | M | LTC2944 prescaler value currently in use |
| `BmsMeasurement getMeasurement()` | struct | All values in a single struct |

---

## Alarm constants

The alarm bitfield returned by `getAlarms()` is a combination of these flags:

| Constant | Bit | Condition |
|----------|-----|-----------|
| `ALARM_SENSOR_FAIL` | 0x0001 | LTC2944 I2C read failed |
| `ALARM_NO_PROFILE` | 0x0002 | Battery profile not found in table |
| `ALARM_UNDER_VOLTAGE` | 0x0004 | Pack voltage below calibrated empty voltage |
| `ALARM_OVER_VOLTAGE` | 0x0008 | Pack voltage above calibrated full voltage + margin |
| `ALARM_ABS_UNDER_V` | 0x0010 | Pack voltage below absolute minimum (hardware damage risk) |
| `ALARM_ABS_OVER_V` | 0x0020 | Pack voltage above absolute maximum (hardware damage risk) |
| `ALARM_TEMPERATURE` | 0x0040 | Die temperature outside –10 °C to +60 °C |
| `ALARM_OVER_CURRENT` | 0x0080 | Discharge current exceeds `setMaxDischargeCurrent()` |
| `ALARM_CHARGE_CURRENT` | 0x0100 | Charge current exceeds `setMaxChargeCurrent()` |
| `ALARM_PROFILE_MISMATCH` | 0x0200 | Measured voltage is outside the plausible range for the selected profile |
| `ALARM_ACR_ROLLOVER` | 0x0400 | ACR counter rolled over — coulomb SOC is unreliable until recalibration |

---

## Battery disconnect detection

### `bool isBatteryDisconnected()`

Returns `true` after battery disconnection has been confirmed. Disconnection is detected when pack voltage drops below the absolute minimum threshold and current is near zero for two consecutive readings. A disconnect flag is written to EEPROM so the condition survives a power cycle. Once `true`, `update()` stops returning new readings. Reconnect and power-cycle to resume normal operation.

---

## Calibration

A calibration run measures the current offset at rest and establishes the empty and full ACR anchor points for the coulomb counter. Run the Calibration sketch once per battery. Results are stored in EEPROM and persist across power cycles.

### `void startCalibration()`

Starts the calibration state machine. Clears any previous calibration data and current offset, then enters the zero phase. Ensure no load and no charger are connected when this is called.

### `void stopCalibration()`

Aborts calibration and returns to normal operation. Calibration data is not saved.

### `CalPhase getCalPhase()`

Returns the current calibration phase.

| Value | Meaning |
|-------|---------|
| `CAL_NONE` | Normal operation |
| `CAL_ZERO` | Sampling current offset (5 s settle + 32 samples) |
| `CAL_WAIT` | Settling before discharge begins |
| `CAL_DISCHARGE` | Discharging to empty voltage |
| `CAL_CHARGE` | Waiting for full charge |
| `CAL_DONE` | Calibration complete, data saved to EEPROM |
| `CAL_ABORT` | Aborted — over-temperature, timeout, or charger connected during discharge |

### `bool isCalibrated()`

Returns `true` if EEPROM contains a valid calibration with both empty and full ACR anchors. When `true`, coulomb-blended SOC is active. When `false`, voltage-curve SOC only.

### `void resetCalibration()`

Erases calibration data from EEPROM and clears the in-RAM state. Use before recalibrating a different battery. Also called automatically by `startCalibration()`.

---

## Learned model

### `float getLearnedResistance()`

Returns the current internal resistance estimate in Ohms. This value is used for IR compensation in the voltage-SOC curve. Returns the default (0.08 Ω) until confidence reaches 50%.

### `float getLearningConfidence()`

Returns the confidence of the learned resistance model as a value from 0.0 to 1.0. Confidence grows with the number of valid resistance samples observed. IR compensation is applied once confidence reaches 0.5.

### `void resetForNewBattery()`

Clears both the learned resistance model and the calibration data from EEPROM. Use when replacing the battery with a different cell — the new battery will have a different internal resistance and different voltage anchors.

---

## Utility

### `const char* getProfileName()`

Returns the resolved profile name as a null-terminated string, e.g. `"LIION_2S"` or `"LFP_4S"`. Returns `"INVALID"` if the chemistry and classCode combination is not in the profile table.

### `bool isProfilePlausible()`

Returns `false` if the measured pack voltage is outside the plausible range for the selected profile. This indicates a wrong profile selection or a seriously out-of-spec battery. Also sets `ALARM_PROFILE_MISMATCH`.

---

## Battery profiles

### Li-ion / Li-polymer (`CHEM_LIION`)

| classCode | Config | Nominal | Empty | Full |
|-----------|--------|---------|-------|------|
| 1 | 1S | 3.7 V | 3.0 V | 4.2 V |
| 2 | 2S | 7.4 V | 6.0 V | 8.4 V |
| 3 | 3S | 11.1 V | 9.0 V | 12.6 V |
| 4 | 4S | 14.8 V | 12.0 V | 16.8 V |
| 6 | 6S | 22.2 V | 18.0 V | 25.2 V |
| 8 | 8S | 29.6 V | 24.0 V | 33.6 V |
| 12 | 12S | 44.4 V | 36.0 V | 50.4 V |
| 14 | 14S | 51.8 V | 42.0 V | 58.8 V |

### LiFePO4 (`CHEM_LIFEPO4`)

| classCode | Config | Nominal | Empty | Full |
|-----------|--------|---------|-------|------|
| 1 | 1S | 3.2 V | 2.8 V | 3.6 V |
| 2 | 2S | 6.4 V | 5.6 V | 7.2 V |
| 3 | 3S | 9.6 V | 8.4 V | 10.8 V |
| 4 | 4S | 12.8 V | 11.2 V | 14.4 V |
| 6 | 6S | 19.2 V | 16.8 V | 21.6 V |
| 8 | 8S | 25.6 V | 22.4 V | 28.8 V |
| 12 | 12S | 38.4 V | 33.6 V | 43.2 V |
| 14 | 14S | 44.8 V | 39.2 V | 50.4 V |

### AGM lead-acid (`CHEM_AGM`) · GEL lead-acid (`CHEM_GEL`)

| classCode | Pack voltage | Empty | Full |
|-----------|-------------|-------|------|
| 12 | 12 V | 10.5 V | 14.4 V (AGM) / 14.2 V (GEL) |
| 24 | 24 V | 21.0 V | 28.8 V (AGM) / 28.4 V (GEL) |
| 48 | 48 V | 42.0 V | 57.6 V (AGM) / 56.8 V (GEL) |

---

## Notes on prescaler selection

The LTC2944 prescaler M determines coulomb counting resolution. The library computes the optimal M automatically based on capacity and shunt value using the formula from the datasheet:

```
M ≥ 4096 × (capacity_mAh / (65535 × 0.340 mAh)) × (shunt_mΩ / 50 mΩ)
```

The result is rounded up to the next power of two from {1, 4, 16, 64, 256, 1024, 4096}. The selected value is printed at startup and accessible via `getPrescaler()`.

For large packs (100 Ah) with a 1 mΩ shunt, M = 1024 is selected automatically, giving a qLSB of approximately 4.25 mAh per count and a full-scale range of about 278 Ah — well suited for 100 Ah packs.
