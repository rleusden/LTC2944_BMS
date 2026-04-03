/*
 * BasicBMS.ino — LTC2944_BMS basic example
 *
 * Reads all battery measurements once per second and prints them
 * to the Serial Monitor (9600 baud).
 *
 * Demonstrates all three output modes:
 *   OUTPUT_SERIAL  — human-readable (default)
 *   OUTPUT_CSV     — comma-separated for data logging / Excel
 *   OUTPUT_ESPHOME — compact \r\n terminated CSV for ESPHome UART parsing
 *
 * Hardware:
 *   Arduino Pro Mini (ATmega328P) + LTC2944 reference PCB
 *   I2C: SDA=A4, SCL=A5  |  Shunt: 1 mOhm (R1)
 */

#include <LTC2944_BMS.h>

LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    // ── 1. Choose your battery chemistry and series-cell count ───────────────
    //   For Li-ion and LiFePO4: classCode = number of series cells.
    //   For AGM / GEL: classCode = pack voltage in Volts.
    //
    //   Li-ion examples:
    //     bms.setProfile(CHEM_LIION,   1);   // 1S  (3.7 V nominal)
    //     bms.setProfile(CHEM_LIION,   2);   // 2S  (7.4 V nominal)
    //     bms.setProfile(CHEM_LIION,   3);   // 3S  (11.1 V nominal)
    //     bms.setProfile(CHEM_LIION,   4);   // 4S  (14.8 V nominal)
    //
    //   LiFePO4 examples:
    //     bms.setProfile(CHEM_LIFEPO4, 1);   // 1S  (3.2 V nominal)
    //     bms.setProfile(CHEM_LIFEPO4, 4);   // 4S  (12.8 V nominal)
    //
    //   Lead-acid (AGM / GEL) examples:
    //     bms.setProfile(CHEM_AGM, 12);       // 12 V
    //     bms.setProfile(CHEM_AGM, 24);       // 24 V
    //     bms.setProfile(CHEM_GEL, 48);       // 48 V
    //
    bms.setProfile(CHEM_LIION, 2);      // 2S Li-ion (7.4 V nominal)

    // ── 2. Set battery capacity in mAh ──────────────────────────────────────
    //   This enables remaining-mAh and runtime estimation, and sets the
    //   correct LTC2944 prescaler for accurate coulomb counting.
    bms.setCapacity(4200);              // 4200 mAh

    // ── 3. Set shunt resistor value ─────────────────────────────────────────
    //   The reference PCB uses a 1 mOhm shunt (R1).
    //   This is already the library default, shown here for clarity.
    bms.setShuntResistor(0.001f);       // 1 mOhm

    // ── 4. Optional: override full/empty voltage to match your charger ───────
    //   If your charger terminates at a non-standard voltage, set it here.
    //   Leave commented out to use the profile default (e.g. 4.2V/cell).
    //   Example for a charger set to 4.15V/cell on a 2S pack:
    //     bms.setFullVoltage(8.30f);
    //     bms.setEmptyVoltage(6.00f);

    // ── 5. Optional: adjust alarm thresholds ────────────────────────────────
    //   The default charge current alarm fires at 3.0 A.
    //   Increase for large packs with high charge rates (e.g. 4200 mAh at 1C = 4.2 A):
    //     bms.setMaxChargeCurrent(5.0f);
    //   The default discharge alarm fires at 5.5 A:
    //     bms.setMaxDischargeCurrent(10.0f);

    // ── 6. Select output format ──────────────────────────────────────────────
    //   OUTPUT_SERIAL   Human-readable (default):
    //     SOC=65%  V=3.849V  I=-748mA  T=17.5C  CHG=NO  REM=1300mAh  RUN=1h44m
    //
    //   OUTPUT_CSV      Comma-separated, header printed once in begin():
    //     soc_%,voltage_V,current_mA,temp_C,charging,remaining_mAh,runtime_min,alarms
    //     65,3.849,-748,17.5,0,1300,104,0
    //
    //   OUTPUT_ESPHOME  Compact CSV terminated with \r\n for ESPHome sscanf:
    //     65,3.849,-748,17.5,0,1300,104,0\r\n
    //
    bms.setOutputMode(OUTPUT_SERIAL);   // <- change to OUTPUT_CSV or OUTPUT_ESPHOME

    // ── 7. Initialise ────────────────────────────────────────────────────────
    if (!bms.begin()) {
        Serial.println(F("ERROR: LTC2944 not found — check wiring!"));
        while (true) { delay(1000); }
    }

    Serial.print(F("Profile  : ")); Serial.println(bms.getProfileName());
    Serial.print(F("Capacity : ")); Serial.print(bms.getCapacityMah()); Serial.println(F(" mAh"));
    Serial.print(F("Prescaler: M=")); Serial.println(bms.getPrescaler());

    if (bms.isCalibrated()) {
        Serial.println(F("Calibration data found in EEPROM."));
    } else {
        Serial.println(F("No calibration — voltage-curve SOC only."));
        Serial.println(F("Run the Calibration example for improved accuracy."));
    }

    Serial.println();
}

void loop() {
    if (!bms.update()) return;

    if (bms.isBatteryDisconnected()) {
        Serial.println(F("Battery disconnected."));
        while (true) { delay(1000); }
    }

    // Single call outputs in the format selected above
    bms.print(Serial);
}
