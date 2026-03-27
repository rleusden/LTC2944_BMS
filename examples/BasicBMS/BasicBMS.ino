/*
 * BasicBMS.ino — minimal example for LTC2944_BMS
 *
 * Reads all battery measurements once per second and prints them
 * to the Serial Monitor (9600 baud).
 *
 * Hardware:
 *   - Arduino Pro Mini (ATmega328P)
 *   - LTC2944 reference PCB with 1 mOhm shunt (R1)
 *   - LTC2944 connected via I2C (SDA = A4, SCL = A5)
 */

#include <LTC2944_BMS.h>

LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    // 1. Select battery chemistry and series-cell count
    //
    //   Li-ion:   bms.setProfile(CHEM_LIION,   0);  // 1S  (3.7 V)
    //             bms.setProfile(CHEM_LIION,   1);  // 2S  (7.4 V)
    //             bms.setProfile(CHEM_LIION,   2);  // 3S  (11.1 V)
    //             bms.setProfile(CHEM_LIION,   3);  // 4S  (14.8 V)
    //
    //   LiFePO4:  bms.setProfile(CHEM_LIFEPO4, 0);  // 1S  (3.2 V)
    //             bms.setProfile(CHEM_LIFEPO4, 3);  // 4S  (12.8 V)
    //
    //   Lead:     bms.setProfile(CHEM_AGM, 0);       // 12 V
    //             bms.setProfile(CHEM_GEL, 1);       // 24 V
    //
    bms.setProfile(CHEM_LIION, 0);      // <- change to match your battery

    // 2. Set the nominal battery capacity in mAh
    //    This enables remaining-mAh and runtime estimation, and configures
    //    the LTC2944 prescaler for correct coulomb counting.
    bms.setCapacity(2000);              // <- change to match your battery (mAh)

    // 3. Set shunt resistor value
    //    The reference PCB uses a 1 mOhm shunt (R1).
    //    This is already the library default, but shown here for clarity.
    bms.setShuntResistor(0.001f);       // 1 mOhm

    // 4. Initialise
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
        Serial.println(F("No calibration — using voltage-curve SOC."));
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

    Serial.print(F("SOC="));
    Serial.print(bms.getSoc());
    Serial.print(F("%  V="));
    Serial.print(bms.getVoltage(), 3);
    Serial.print(F("V  I="));
    Serial.print(bms.getCurrent() * 1000.0f, 0);
    Serial.print(F("mA  T="));
    Serial.print(bms.getTemperature(), 1);
    Serial.print(F("C  CHG="));
    Serial.print(bms.isCharging() ? F("YES") : F("NO"));

    // Remaining capacity and runtime (only shown when capacity is set)
    if (bms.getCapacityMah() > 0) {
        Serial.print(F("  REM="));
        Serial.print(bms.getRemainingMah());
        Serial.print(F("mAh"));

        int16_t runtime = bms.getRuntimeMinutes();
        if (runtime >= 0) {
            Serial.print(F("  RUN="));
            if (runtime >= 60) {
                Serial.print(runtime / 60);
                Serial.print(F("h"));
                Serial.print(runtime % 60);
                Serial.print(F("m"));
            } else {
                Serial.print(runtime);
                Serial.print(F("m"));
            }
        }
    }

    // Print active alarms
    uint16_t al = bms.getAlarms();
    if (al) {
        Serial.print(F("  ALARM=0x"));
        Serial.print(al, HEX);
        if (al & ALARM_UNDER_VOLTAGE)    Serial.print(F(" [UV]"));
        if (al & ALARM_OVER_VOLTAGE)     Serial.print(F(" [OV]"));
        if (al & ALARM_TEMPERATURE)      Serial.print(F(" [TEMP]"));
        if (al & ALARM_OVER_CURRENT)     Serial.print(F(" [OC]"));
        if (al & ALARM_PROFILE_MISMATCH) Serial.print(F(" [MISMATCH]"));
    }

    Serial.println();
}
