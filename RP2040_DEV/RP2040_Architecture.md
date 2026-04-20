# LTC2944_BMS v2 — Architecture Reference

## Overview

LTC2944_BMS v2 is a multi-architecture Arduino library for the LTC2944 battery
gas gauge.  It supports ATmega328P (Arduino Pro Mini / Nano / Uno) and RP2040
(Waveshare RP2040-Zero, Raspberry Pi Pico) from a single codebase.  The v1.x
Pro Mini API is preserved unchanged so existing sketches recompile without
modification.

The library is divided into **four portable layers** and a **platform HAL**.
Only the HAL touches architecture-specific code.

---

## Layer diagram

```
┌──────────────────────────────────────────────────────────┐
│  Sketch  (BasicBMS.ino / RP2040_DualCore.ino / …)        │
│  #include <LTC2944_BMS.h>   — one line, always           │
└──────────────┬───────────────────────────────────────────┘
               │ public API
┌──────────────▼───────────────────────────────────────────┐
│  BoardLayer  (src/BoardLayer.h / .cpp)                   │
│  • Instantiates HAL backends                             │
│  • Wires layers together                                 │
│  • Owns pins, Wire init, EEPROM.begin()                  │
│  • Calibration FSM callbacks                             │
│  • RP2040 dual-core snapshot                             │
└──────┬───────┬──────────────┬───────────────────────────┘
       │       │              │
┌──────▼──┐ ┌──▼──────────┐ ┌▼────────────────┐
│LTC2944  │ │BatteryModel │ │PersistenceLayer │
│Device   │ │             │ │                 │
│         │ │• Chemistry  │ │• EEPROM map     │
│• Regs   │ │  profiles   │ │• Cal anchors    │
│• Conv.  │ │• SOC fusion │ │• Learned model  │
│• ACR    │ │• OCV lookup │ │• Dual-slot wear │
│• Prescl │ │• IR comp.   │ │  levelling      │
│         │ │• Eta accum  │ │• Timestamp      │
│         │ │• Resistance │ │• SOH / offset   │
│         │ │  learning   │ │                 │
│         │ │• SOH / dera │ │                 │
└──────┬──┘ └──────┬──────┘ └───────┬─────────┘
       │            │                │
       │    ┌───────▼────────────────▼─────────┐
       │    │  CalibrationFSM                  │
       │    │  (src/CalibrationFSM.h / .cpp)   │
       │    │  • Zero / wait / discharge /     │
       │    │    charge / done / abort states  │
       │    │  • Callback-based, no layer deps │
       └────┴──────────────┬───────────────────┘
                           │ interfaces only
               ┌───────────▼───────────────────┐
               │  HAL  (src/hal/)              │
               │  BMS_I2C_HAL                  │
               │  BMS_Storage_HAL  + commit()  │
               │  BMS_Progmem_HAL              │
               └───────────┬───────────────────┘
                   ┌────────┴────────┐
          ┌────────▼──┐         ┌────▼──────────┐
          │ AVR backend│         │ RP2040 backend│
          │ Wire       │         │ Wire / Wire1  │
          │ EEPROM     │         │ EEPROM (flash)│
          │ pgm_read   │         │ plain array   │
          └────────────┘         └───────────────┘
```

---

## Files

```
LTC2944_BMS.h                   Umbrella header — the only include sketches need
library.properties

src/
  LTC2944_Device.h / .cpp       Device layer — register I/O, conversions, ACR
  BatteryModel.h / .cpp         SOC / model layer — all chemistry logic
  PersistenceLayer.h / .cpp     Storage layer — EEPROM map, save/load
  CalibrationFSM.h / .cpp       Calibration state machine
  BoardLayer.h / .cpp           Integration + public API (LTC2944_BMS class)

  hal/
    BMS_HAL.h                   Three pure-virtual interface contracts
    BMS_HAL_Select.h            Compile-time backend selection (using aliases)

    avr/
      BMS_HAL_AVR.h             AVR backend (Wire + EEPROM + pgm_read_word)
      ocv_tables_progmem.h      PROGMEM OCV curves + BMS_OCV_TABLES_IN_PROGMEM guard

    rp2040/
      BMS_HAL_RP2040.h          RP2040 backend (Wire/Wire1 + EEPROM emulation)

examples/
  BasicBMS/                     Minimal, identical API on both platforms
  Calibration/                  Full discharge/charge cal cycle
  RP2040_DualCore/              Dual-core sketch replacing the v2.0.1 monolith
```

---

## Invariants — things that must NEVER be violated

### 1  Architecture-specific code is confined to `src/hal/`

The four portable layers (`LTC2944_Device`, `BatteryModel`, `PersistenceLayer`,
`CalibrationFSM`) must never include:

- `<EEPROM.h>`
- `<avr/pgmspace.h>`
- `PROGMEM`, `pgm_read_word`, `pgm_read_byte`
- `Wire.setSDA()`, `Wire.setSCL()` (RP2040-only)
- `EEPROM.begin()`, `EEPROM.commit()` (RP2040-only)
- `noInterrupts()` / `interrupts()` in portable logic

The only permitted exception is `BoardLayer.cpp`, which may use `#if defined()`
blocks for:

- `setWire()` (RP2040: `setSDA/setSCL`)
- `begin()` (RP2040: `EEPROM.begin`; AVR: `Wire.begin`)
- `_writeSnapshot()` / `getSnapshot()` (RP2040: dual-core critical section)

### 2  `_storageBackend.commit()` is always unconditional

`BMS_Storage_HAL::commit()` is a virtual no-op on AVR.  `BMS_Storage_RP2040`
overrides it to call `EEPROM.commit()`.  BoardLayer calls `_storageBackend.commit()`
without any `#if` guard.  Adding a guard is a bug — it means AVR gets a dead
call (fine) but someone adding a new platform must remember to call commit()
themselves.

### 3  The EEPROM map is frozen at v1.x offsets

```
0x00–0x03  Global magic
0x04–0x07  Current offset (float, mA)
0x08       Disconnect flag
0x20–0x33  Calibration record
0x40–0x43  Timestamp (uint32_t, seconds)
0x44–0x46  SOH magic + value
0xA0–0xDF  Learned model slots A and B (dual-slot wear levelling)
```

An AVR board running v1.x firmware that upgrades to v2.x retains its calibration
and learned model without any migration tool.

### 4  OCV tables have exactly one definition per build

- On flat-address platforms: defined in `BatteryModel.cpp` as plain `const uint16_t[]`.
- On AVR: defined in `ocv_tables_progmem.h` with `PROGMEM`.  The guard macro
  `BMS_OCV_TABLES_IN_PROGMEM` prevents `BatteryModel.cpp` from defining them again.
- Never defined in both places — the linker would error or silently pick one.

---

## Adding a new architecture

1.  Create `src/hal/<arch>/BMS_HAL_<arch>.h`.  Implement:
    - `BMS_I2C_<arch>` extending `BMS_I2C_HAL`
    - `BMS_Storage_<arch>` extending `BMS_Storage_HAL`
      — override `commit()` if writes are buffered
    - `BMS_Progmem_<arch>` extending `BMS_Progmem_HAL`
      — use `BMS_Progmem_Flat` if the platform has a flat address space

2.  Add a branch to `src/hal/BMS_HAL_Select.h`:
    ```cpp
    #elif defined(ARDUINO_ARCH_MYARCH)
        #include "myarch/BMS_HAL_MYARCH.h"
        using BMS_I2C_Backend     = BMS_I2C_MYARCH;
        using BMS_Storage_Backend = BMS_Storage_MYARCH;
        using BMS_Progmem_Backend = BMS_Progmem_Flat;   // or custom
    ```

3.  Add any one-time init (e.g. `EEPROM.begin()`) to the `begin()` method in
    `BoardLayer.cpp` inside a new `#if defined(ARDUINO_ARCH_MYARCH)` block.

4.  Add `Wire.begin()` or equivalent inside `begin()` if needed.

5.  That is all.  No other file needs to change.

---

## Calibration state machine

```
CAL_NONE
    │  startCalibration()
    ▼
CAL_ZERO ──── 32 samples at zero current ───► zeroCb(rawDeltaAvg)
    │          load enabled
    ▼
CAL_WAIT ──── wait CAL_SETTLE_MS for discharge current
    │
    ▼
CAL_DISCHARGE ── voltage ≤ vEmpty + 50 mV ──► emptyCb(v, rawACR)
    │
    ▼
CAL_CHARGE ── charger plugged in
    │         near-full detected (V ≥ 98% peak)
    │         charger cutoff confirmed ──────────► fullCb(v, rawACR, cmax)
    ▼
CAL_DONE

(any phase + timeout or stopCalibration()) → CAL_ABORT → CAL_NONE
```

The FSM calls back into `BoardLayer` via four function pointers.  It has no
knowledge of I2C, EEPROM, or BatteryModel — it only receives converted
engineering values and fires events.

---

## RP2040 dual-core usage

```
Core 0                          Core 1
──────────────────────          ────────────────────────────────────
Wire.begin() / bms.begin()      setup1(): wait for g_core0Ready
bms.update()                    loop1():  snap = bms.getSnapshot()
  → LTC2944_Device::readSample()          // use snap for display,
  → BatteryModel::update()                // Modbus, CAN, web, …
  → _writeSnapshot() [noInterrupts]
```

`getSnapshot()` wraps the struct copy in a `noInterrupts/interrupts` pair.
This is safe in the arduino-pico environment without FreeRTOS: masking
interrupts on Core 0 for the duration of a `BmsMeasurement` copy (~40 bytes)
prevents the SIO scheduler from interleaving the write with the read on Core 1
when Core 1 is also masked.  If you introduce FreeRTOS, replace the critical
section with a mutex.

---

## Thread safety summary

| Operation | Safe from Core 1? | Notes |
|---|---|---|
| `bms.getSnapshot()` | Yes | Protected by noInterrupts/interrupts |
| `bms.update()` | No | Must run on Core 0 only (owns I2C) |
| `bms.print()` | No | Uses `_meas` directly; call from Core 0 |
| `bms.startCalibration()` | No | Modifies FSM state; call from Core 0 |

---

## v1.x → v2.x migration guide

### Sketch code: no changes needed

```cpp
// v1.x sketch — compiles unchanged under v2.x on AVR
LTC2944_BMS bms;
void setup() {
    bms.setProfile(CHEM_LIION, 2);
    bms.setCapacity(4200);
    bms.setShuntResistor(0.001f);
    bms.begin();
}
void loop() {
    if (bms.update()) bms.print(Serial);
}
```

### RP2040-only additions (guarded, invisible on AVR)

```cpp
// RP2040 sketch additions
bms.setWire(Wire, /*SDA=*/4, /*SCL=*/5);   // before begin()
// …
BmsMeasurement snap = bms.getSnapshot();    // from Core 1
```

### EEPROM data: preserved automatically

The v2.x `PersistenceLayer` uses the identical byte-offset layout as the v1.x
`EEPROM.put/get` calls.  No migration tool or data-wipe is required when
upgrading a running AVR board.

---

## Version policy

| Series | Platform | Status |
|---|---|---|
| 1.x.x | AVR only | Maintenance — bug-fixes only |
| 2.x.x | AVR + RP2040 | Active development |

The 1.x and 2.x series share the same library name and EEPROM layout.  They do
not share source files.  `library.properties` targets `architectures=avr,rp2040`
for 2.x.

New architectures (ESP32, SAMD, STM32) follow the "Adding a new architecture"
steps above and do not require a 3.x series.

---

## Coulomb-counting design

### ACR LSB propagation

The LTC2944 ACR register LSB size depends on both the prescaler M and the
installed shunt resistance:

```
lsbMah = 0.085 mAh × M × (50 mΩ / Rsense)
```

The constant `0.085` lives exclusively in `LTC2944_Device` — the single
authoritative source. `BatteryModel` knows nothing about registers or shunt
values; it receives the LSB via `setAcrLsbMah()`, called by `BoardLayer::begin()`
immediately after `_device.begin(prescaler)`:

```cpp
_model.setAcrLsbMah(_device.rawACRtoMAh(1));  // rawACRtoMAh(1) == lsbMah
```

This ensures self-discharge, boot-seed voltage→ACR mapping, and coulomb SOC
all use the correct conversion factor for the installed hardware.

### Minimum resolution requirement

Coulomb counting is automatically disabled when the pack spans fewer than
200 ACR counts:

```
acrFull = capacityMah / lsbMah
if (acrFull < 200) → _estimateCoulombSoc() returns -1 → voltage-only SOC
```

This threshold is enforced in `BatteryModel::_estimateCoulombSoc()`. It
activates automatically when `setCapacity()` and `setShuntResistor()` produce
a configuration where counting resolution is too coarse to be useful.

**Practical guide:**

| Pack (mAh) | Shunt (mΩ) | lsbMah (mAh) | acrFull (counts) | Coulomb counting |
|---|---|---|---|---|
| 260 | 1 | 4.25 | 61 | ✗ disabled |
| 260 | 50 | 0.085 | 3059 | ✓ active |
| 4200 | 1 | 4.25 | 988 | ✓ active |
| 20000 | 1 | 4.25 | 4706 | ✓ active |

For small test packs on low-value shunts, the library falls back to
voltage-only SOC (`src=3`). For production packs the full hybrid SOC
(`src=4`) activates without any configuration change.

### Eta accumulator

The eta-corrected ACR shadow (`_etaAcrAccum`) tracks charge in raw ACR counts
with coulombic efficiency applied:

- On charge:    `_etaAcrAccum += delta × ηc`
- On discharge: `_etaAcrAccum += delta × ηd`

`delta` is computed as `(int32_t)rawACR − (int32_t)_lastRawACRForEta` — integer
subtraction in the raw domain, not float subtraction against the accumulator.
A separate `uint16_t _lastRawACRForEta` tracks the previous sample to keep the
delta calculation free of accumulated efficiency drift.

`update()` skips `_updateEtaAccum` whenever `hadACRRollover()` returns true,
preventing a spurious delta of ~65535 counts from corrupting the accumulator.

A plausibility guard in `_updateEtaAccum` silently discards any delta larger
than ±10% of full scale (±6554 counts) in a single sample interval. This
protects the accumulator from ACR register resets caused by chip reinitialisation
or power glitches.

### `hadACRRollover()` — read-and-clear

`LTC2944_Device::hadACRRollover()` clears `_acrRollover` on read. Call it
exactly once per `update()` cycle. The alarm bit `ALARM_ACR_ROLLOVER` captures
rollover state for reporting.

### Boot seed

On the first valid `update()` call after boot, the accumulator is seeded from
the OCV curve via `forceBootSeedFromVoltage(voltage, currentRawACR)`:

1. OCV table lookup converts pack voltage → SOC percentage
2. SOC × acrFull gives the logical accumulator position
3. `_lastRawACRForEta` is set to the **actual hardware ACR register value**,
   not the logical seed position

Step 3 is critical. The LTC2944 powers on with ACR at an arbitrary value
(often 0xFFFF or 0x7FFF). If `_lastRawACRForEta` were set to the tiny seed
count instead, the first `_updateEtaAccum` call would compute a delta of
±32768 and immediately corrupt the accumulator.

If voltage at boot is below the OCV table minimum (battery deeply discharged
or boot occurring under load), the accumulator seeds at 5% as a safe floor
rather than 0%, to avoid locking the SOC display at zero.

---

## LTC2944 control register

The control register (0x01) bit layout is:

```
[7:6]  ALCC pin config  00=disabled  01=charge-complete  10=alert  11=n/a
[5:3]  Prescaler M      000=M1  001=M2 … 111=M128
[2:1]  ADC mode         00=sleep  01=manual  10=scan  11=automatic
[0]    Shutdown         0=normal  1=chip off
```

The library writes `(0x02 << 6) | (mBits << 3) | (0x03 << 1) | 0x00` —
ALCC alert, chosen prescaler, automatic ADC, no shutdown. For M=1 this
is `0x86`.

**Common mistakes:**

- `0xFC` — sets shutdown=0 and ADC=scan(10) but M=128. Works but uses the
  wrong prescaler and a slower conversion mode.
- `0x83` — sets shutdown=**1**. Chip responds to I2C but ADC is off;
  register values are frozen at last reading.
- `0x80 | mBits<<3 | 0x03` — puts ADC bits in [1:0] instead of [2:1];
  produces shutdown=1 for any M value.

### begin() robustness

`LTC2944_Device::begin()` retries the initial status register read up to
10 times with 10 ms gaps (100 ms total). This handles slow power-rail rise
times and I2C bus contention from other devices initialising on the same bus.

If `begin()` still fails after 100 ms, `BoardLayer::begin()` continues
with `_present=false`. On the first `update()` call, `BoardLayer` calls
`_device.reinit()` automatically. A successful reinit sets `_bootSeedPending=true`
so the accumulator is re-seeded from voltage on the next sample rather than
inheriting a stale delta.

This means a `begin(): FAILED` message in the serial log does not indicate
a hardware fault if subsequent `[DBG]` lines show valid readings — the chip
self-healed on the first `update()` cycle.

---

## SOC source codes

The `BmsMeasurement::socSource` field (also accessible via `bms.getSocSource()`)
reports which path produced the final SOC value:

| Value | Name | Meaning |
|---|---|---|
| 0 | `SOC_UNKNOWN` | Not yet computed |
| 1 | `SOC_VOLTAGE_BOOT` | First estimate from OCV at boot |
| 2 | `SOC_VOLTAGE_ROLLOVER` | Voltage used after ACR rollover |
| 3 | `SOC_VOLTAGE_ONLY` | Coulomb counting disabled or unavailable |
| 4 | `SOC_BLEND` | Hybrid: voltage + coulomb fused by adaptive blend |

`src=3` is normal and correct when `acrFull < 200` (small pack / large shunt).
`src=4` indicates the full hybrid SOC engine is active.

---

## Shunt selection guide

The shunt resistor value controls both current measurement range and ACR
resolution. Choose based on the maximum expected current and minimum pack size:

```
Max current  = 64 mV / Rsense
lsbMah       = 0.085 × M × (50 mΩ / Rsense)
acrFull      = capacityMah / lsbMah
```

For coulomb counting to activate: `acrFull ≥ 200`
→ `Rsense ≤ 0.085 × M × 50 mΩ × capacityMah / 200`

At M=1: `Rsense ≤ capacityMah × 0.02125 mΩ`

| Application | Pack | Recommended shunt | Max current |
|---|---|---|---|
| Small test pack | 260 mAh | 50–100 mΩ | 0.64–1.28 A |
| LiPo / Li-ion | 1–5 Ah | 5–10 mΩ | 6.4–12.8 A |
| Boat / solar | 20–200 Ah | 0.5–1 mΩ | 64–128 A |

The LTC2944's reference shunt is 50 mΩ. Using a 50 mΩ shunt gives an ACR
LSB of exactly 0.085 mAh at M=1, matching the datasheet examples directly.
