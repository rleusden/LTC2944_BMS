# LTC2944 Current Measurement Calibration

## Overview

This document describes the practical calibration of the current measurement path using the **LTC2944** in combination with a **4-terminal shunt resistor**.

The goal was to determine the **effective shunt resistance** of the complete measurement chain and validate the accuracy of the LTC2944 current readings.

---

## Test Setup

### Hardware

- Battery: 4S 2.2Ah 70C LiPo pack (Turnigy Nano-Tech) used as a stable energy source
- Measurement Board
  - Custom LTC2944 PCB with:
  - 4-terminal (Kelvin) shunt resistor (nominal: **1 mΩ**, 1%)
  - Arduino Pro Mini interface
- HP 3456A (voltage measurement across Kelvin sense terminals)
- ZT-703S (current reference up to 10 A)
- Owon OEL1530T (current source)

### Method

1. The electronic load was configured to draw fixed currents.
2. Current was verified using the ZT-703S (up to 10 A).
3. Voltage across the shunt was measured using the HP3456A directly at the Kelvin sense terminals.
4. The LTC2944 raw current was read using a minimal sketch (no library influence).
5. The effective shunt resistance was derived and used to calibrate the LTC2944.

---

## Shunt Measurement Results

Measured voltage across the shunt:

| Current (A) | Voltage (mV) | Calculated R (mΩ) |
|------------|-------------|-------------------|
| 22.00      | 21.594      | 0.9815            |
| 20.00      | 19.634      | 0.9817            |
| 15.00      | 14.725      | 0.9817            |
| 10.00      | 9.818       | 0.9818            |
| 5.00       | 4.910       | 0.9820            |
| 1.00       | 0.988       | 0.9880            |

### Observation

- Above ~5 A, the effective shunt resistance is highly consistent:
  - **~0.982 mΩ**
- The 1 A point shows slightly higher deviation due to measurement resolution and setup limitations.
- The shunt and measurement path are **linear and stable**.

---

## LTC2944 Calibration

Initial configuration used:

RSENSE_OHM = 0.001000f;

This resulted in a systematic underestimation of current.

After calibration, the effective value was determined as:

RSENSE_OHM = 0.000978f;

---

## Validation Results

|Reference Current (A)|	LTC2944 (A)|	Error (mA)|	Error (%)|
|---------------------|------------|------------|----------|
|1.00	|-0.984548|	15.5|	1.55%|
|2.00	|-1.971086|	28.9|	1.45%|
|5.00	|-4.962521|	37.5|	0.75%|
|8.00	|-7.985781|	14.2|	0.18%|
|10.00|-9.999587|	0.4|	0.004%|
|20.00|-20.033123|	33.1|	0.17%|
|25.00|-25.049892|	49.8|	0.20%|
|30.00|-30.066659|	66.6|	0.22%|

Observations:
- Measurement error is small and consistent
- Accuracy improves with higher current
- No significant non-linearity observed
- The LTC2944 slightly underestimates current at lower values
- Overall accuracy is within:
  - ~1.5% at low current
  - <0.25% above ~5 A

---

## Conclusion

The LTC2944 current measurement is accurate and reliable for BMS use when the effective shunt resistance is correctly calibrated.

Key result:

The effective shunt resistance of the measurement chain is approximately 0.978 mΩ.

Using this value:

The LTC2944 provides highly accurate current readings
Linearity across the tested range (1–20 A) is good
The measurement is suitable for:
- SOC estimation
- runtime calculation
- load monitoring
  
Notes:
- This calibration reflects the entire measurement chain, not just the shunt resistor
- Kelvin sensing is critical for accurate results
- Further refinement below 1 A may require offset calibration
