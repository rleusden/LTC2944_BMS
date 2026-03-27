/*
 * Calibration.ino — full discharge/charge calibration cycle
 *
 * What it does:
 *   1. Zeroes the current offset at rest.
 *   2. Enables the discharge load (load-enable pin) until empty voltage.
 *   3. Prompts you to connect a charger; waits until the battery is full.
 *   4. Saves both ACR anchor points to EEPROM.
 *      After this, the library blends voltage-SOC with coulomb-SOC for
 *      improved accuracy.
 *
 * Requirements:
 *   - A discharge load wired to the load-enable pin (default pin 5).
 *   - Serial Monitor open at 9600 baud.
 *
 * IMPORTANT:
 *   - Do NOT leave the sketch running unsupervised — it will drain the battery.
 *   - Calibration aborts automatically if temperature exceeds 55 C
 *     or if either phase takes longer than 30 minutes.
 */

#include <LTC2944_BMS.h>

LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    bms.setProfile(CHEM_LIION, 0);   // <- match your battery
    bms.setCapacity(2000);           // <- match your battery (mAh)
    bms.setShuntResistor(0.001f);    // 1 mOhm (reference hardware)

    if (!bms.begin()) {
        Serial.println(F("ERROR: LTC2944 not found!"));
        while (true) {}
    }

    Serial.print(F("Profile  : ")); Serial.println(bms.getProfileName());
    Serial.print(F("Capacity : ")); Serial.print(bms.getCapacityMah()); Serial.println(F(" mAh"));
    Serial.print(F("Prescaler: M=")); Serial.println(bms.getPrescaler());
    Serial.println();
    Serial.println(F("Starting calibration..."));
    Serial.println(F("Make sure the battery is at rest (no load, no charger) right now."));
    Serial.println();

    bms.startCalibration();
}

static const char* phaseName(CalPhase p) {
    switch (p) {
        case CAL_NONE:       return "IDLE";
        case CAL_ZERO:       return "ZEROING CURRENT";
        case CAL_WAIT:       return "SETTLING";
        case CAL_DISCHARGE:  return "DISCHARGING";
        case CAL_CHARGE:     return "WAITING FOR FULL CHARGE";
        case CAL_DONE:       return "DONE";
        case CAL_ABORT:      return "ABORTED";
        default:             return "?";
    }
}

CalPhase lastPhase = CAL_NONE;

void loop() {
    if (!bms.update()) return;

    CalPhase phase = bms.getCalPhase();

    if (phase != lastPhase) {
        Serial.print(F("Phase -> "));
        Serial.println(phaseName(phase));

        if (phase == CAL_CHARGE) {
            Serial.println(F(">>> Discharge complete.  Connect your charger now. <<<"));
        }
        if (phase == CAL_DONE) {
            Serial.println(F(">>> Calibration successful!  Saved to EEPROM. <<<"));
            Serial.println(F("Upload BasicBMS for normal use."));
        }
        if (phase == CAL_ABORT) {
            Serial.println(F(">>> Calibration ABORTED (over-temperature or timeout). <<<"));
        }
        lastPhase = phase;
    }

    if (phase != CAL_DONE && phase != CAL_ABORT && phase != CAL_NONE) {
        Serial.print(F("  V="));
        Serial.print(bms.getVoltage(), 3);
        Serial.print(F("V  I="));
        Serial.print(bms.getCurrent() * 1000.0f, 0);
        Serial.print(F("mA  T="));
        Serial.print(bms.getTemperature(), 1);
        Serial.print(F("C"));

        if (bms.getCapacityMah() > 0) {
            Serial.print(F("  REM="));
            Serial.print(bms.getRemainingMah());
            Serial.print(F("mAh"));
        }
        Serial.println();
    }
}
