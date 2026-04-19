# BatteryGauge RP2040 v2.x.x OLED Edition  
## Current Architecture, Implemented Functionality, and TODO Roadmap

---

# 1. Overview

BatteryGauge RP2040 v2.0.1 is a standalone battery monitor platform based on:

- **RP2040-Zero**
- **LTC2944** battery gas gauge/coulomb counter
- **SSD1306 OLED 128x32 I2C display**
- **WS2812 status LED**

The project goal is to provide a compact, low-cost, intelligent battery monitor for:

- boats
- campers
- solar storage
- backup batteries
- mobile systems
- DIY battery packs

The current firmware already uses both RP2040 cores:

- **Core 0** = hardware interfacing + UI
- **Core 1** = battery calculations

---

# 2. Hardware Architecture

## Main Components

```text
Battery Pack
   │
Shunt Resistor
   │
LTC2944
   │ I2C
RP2040-Zero
   ├── SSD1306 OLED Display
   ├── WS2812 Status LED
   └── Serial Debug
```

# 3. Software Architecture

**Core 0 Responsibilities**

Realtime/device layer:

- I2C communication
- LTC2944 raw register reads
- sample scheduling
- OLED updates
- WS2812 LED patterns
- serial debug output
- system boot logic

**Core 1 Responsibilities**

Battery intelligence layer:

- convert raw registers to engineering values
- voltage estimation
- current estimation
- temperature estimation
- SOC estimation
- remaining capacity estimate
- runtime estimate
- charging detection

# 4. Current Implemented Functionality

### 4.1 LTC2944 Communication

Implemented:

- I2C bus initialization
- device detection
- register reads:
- Voltage
- Current
- Accumulated Charge (ACR)
- Temperature

Status: ✅ Working

### 4.2 Multicore Processing

Implemented:

- Core 0 continuously samples hardware
- Core 1 processes the latest sample
- shared state via volatile structs

Status: ✅ Working

### 4.3 Battery Measurements

Implemented:

- pack voltage
- current
- temperature
- raw ACR counter

Status: ✅ Working

Notes: Current scaling still needs final calibration.

### 4.4 SOC Estimation

Current implementation:

Simple voltage-based SOC for 2S lithium pack:

- 6.20V = 0%
- 8.30V = 100%
- Linear interpolation between limits

Status: ⚠ Temporary implementation

Needs replacement by hybrid SOC engine.

### 4.5 Remaining Capacity

Current implementation:

SOC × nominal pack size

Status: ⚠ Placeholder

### 4.6 Runtime Estimate

Current implementation:

If discharging:

remaining Ah / discharge current

Status: ⚠ Functional but basic

### 4.7 OLED Display

Implemented:

Auto-detect SSD1306:

0x3C
0x3D

Rotating pages:

- Page 1
SOC %
Voltage
Current
- Page 2
Remaining Ah
Runtime
- Page 3
Temperature
Charging / Running

Status:✅ Working

### 4.8 WS2812 Status LED

Implemented:

States
- Green = normal
- Blue blinking = charging
- Orange blinking = low battery
- Red fast blinking = critical battery
- Purple blinking = sensor fault

Status:✅ Working

### 4.9 Serial Debug

Implemented:

Example:

[DBG] V=7.415 I=-0.092 T=20.2 SOC=54 REM=10.8 RUN=702

Status: ✅ Working

# 5. Current Limitations
### 5.1 SOC Logic is Basic

Currently only voltage-based.

Problems:

- inaccurate under load
- inaccurate while charging
- chemistry dependent
  
### 5.2 No Calibration Storage Yet

Missing:

- shunt calibration
- current offset
- ACR full/empty anchors
- learned battery parameters

### 5.3 No Battery Profile System Yet

Missing selectable profiles:

- Li-ion
- LiFePO4
- AGM
- GEL
- custom curves

### 5.4 No User Input Yet

Missing:

- button
- menu
- calibration controls

### 5.5 No Communications Yet

Planned but not implemented:

- RS485 Modbus
- ESP32 bridge mode
- Web UI
