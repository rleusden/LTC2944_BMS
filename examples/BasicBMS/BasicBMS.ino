/*
 * BasicBMS.ino — minimal example for LTC2944_BMS
 *
 * This sketch reads all battery measurements once per second and prints
 * them to the Serial Monitor (9600 baud).
 *
 * Hardware:
 *   - Arduino Pro Mini (or Nano / Uno) with 3.3 V / 5 V supply
 *   - LTC2944 connected via I²C (SDA = A4, SCL = A5 on Pro Mini)
 *   - 10 mΩ shunt resistor between the LTC2944 SENSE+ and SENSE- pins
 *
 * Wiring quick-start:
 *   LTC2944 SDA  -> A4
 *   LTC2944 SCL  -> A5
 *   LTC2944 GND  -> GND
 *   LTC2944 VCC  -> 3.3 V or 5 V (check your module's datasheet)
 */

#include <LTC2944_BMS.h>

// ── Create the BMS object ────────────────────────────────────────────────────
LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);
    while (!Serial) { /* wait for USB CDC on boards that need it */ }

    // ── 1. Choose your battery chemistry and series-cell count ──────────────
    //
    //   Li-ion examples:
    //     bms.setProfile(CHEM_LIION,   0);   // 1S  (3.7 V nominal)
    //     bms.setProfile(CHEM_LIION,   1);   // 2S  (7.4 V nominal)
    //     bms.setProfile(CHEM_LIION,   2);   // 3S  (11.1 V nominal)
    //     bms.setProfile(CHEM_LIION,   3);   // 4S  (14.8 V nominal)
    //
    //   LiFePO4 examples:
    //     bms.setProfile(CHEM_LIFEPO4, 0);   // 1S  (3.2 V nominal)
    //     bms.setProfile(CHEM_LIFEPO4, 3);   // 4S  (12.8 V nominal)
    //
    //   Lead-acid (AGM / GEL) examples:
    //     bms.setProfile(CHEM_AGM, 0);        // 12 V
    //     bms.setProfile(CHEM_AGM, 1);        // 24 V
    //     bms.setProfile(CHEM_GEL, 2);        // 48 V
    //
    bms.setProfile(CHEM_LIION, 0);  // ← change this to match your battery

    // ── 2. (Optional) override defaults if your hardware differs ────────────
    // bms.setShuntResistor(0.005f);        // 5 mΩ shunt instead of 10 mΩ
    // bms.setPinLoadEnable(5);             // pin driving discharge MOSFET (default 5)
    // bms.setPinStatusLed(13);             // status LED pin (default 13)

    // ── 3. Initialise the library ───────────────────────────────────────────
    if (!bms.begin()) {
        Serial.println(F("ERROR: LTC2944 not found — check wiring!"));
        while (true) { delay(1000); }
    }

    Serial.print(F("LTC2944_BMS ready — profile: "));
    Serial.println(bms.getProfileName());

    if (bms.isCalibrated()) {
        Serial.println(F("Calibration data found in EEPROM."));
    } else {
        Serial.println(F("No calibration data — using voltage-curve SOC only."));
        Serial.println(F("Call bms.startCalibration() for improved accuracy."));
    }
    Serial.println();
}

void loop() {
    // update() returns true once per second when a fresh reading is available.
    if (!bms.update()) return;

    // ── Check for disconnect ─────────────────────────────────────────────────
    if (bms.isBatteryDisconnected()) {
        Serial.println(F("Battery disconnected — halting telemetry."));
        while (true) { delay(1000); }
    }

    // ── Print measurements ───────────────────────────────────────────────────
    Serial.print(F("SOC="));
    Serial.print(bms.getSoc());
    Serial.print(F("%  V="));
    Serial.print(bms.getVoltage(), 3);
    Serial.print(F("V  I="));
    Serial.print(bms.getCurrent() * 1000.0f, 0);
    Serial.print(F("mA  T="));
    Serial.print(bms.getTemperature(), 1);
    Serial.print(F("°C  CHG="));
    Serial.print(bms.isCharging() ? F("YES") : F("NO"));

    // ── Print any active alarms ───────────────────────────────────────────────
    uint16_t al = bms.getAlarms();
    if (al) {
        Serial.print(F("  ALARM=0x"));
        Serial.print(al, HEX);
        if (al & ALARM_UNDER_VOLTAGE)    Serial.print(F(" [UV]"));
        if (al & ALARM_OVER_VOLTAGE)     Serial.print(F(" [OV]"));
        if (al & ALARM_TEMPERATURE)      Serial.print(F(" [TEMP]"));
        if (al & ALARM_PROFILE_MISMATCH) Serial.print(F(" [MISMATCH]"));
    }

    Serial.println();
}
