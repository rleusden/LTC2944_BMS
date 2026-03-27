/*
 * Calibration.ino — run a full discharge/charge calibration cycle
 *
 * What it does:
 *   1. Zeroes the current offset at rest.
 *   2. Enables the discharge load (on the load pin) until the battery
 *      reaches its empty voltage threshold.
 *   3. Prompts you to connect a charger and waits until the battery is full.
 *   4. Saves both ACR (accumulated charge) anchor points to EEPROM.
 *      After this, update() will blend voltage-SOC with coulomb-SOC for
 *      improved accuracy.
 *
 * Requirements:
 *   - A discharge load wired to the load-enable pin (default pin 5).
 *   - Serial Monitor open at 9600 baud.
 *
 * IMPORTANT:
 *   - Do NOT leave the sketch running unsupervised — it will drain the battery.
 *   - The calibration will abort automatically if the die temperature
 *     exceeds 55 °C or if either phase takes longer than 30 minutes.
 */

#include <LTC2944_BMS.h>

LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}

    bms.setProfile(CHEM_LIION, 0);   // ← match your battery

    if (!bms.begin()) {
        Serial.println(F("ERROR: LTC2944 not found!"));
        while (true) {}
    }

    Serial.println(F("Starting calibration..."));
    Serial.println(F("Make sure the battery is at rest (no load, no charger) right now."));
    Serial.println();

    bms.startCalibration();
}

// Human-readable phase names
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

    // Print a message whenever the phase changes
    if (phase != lastPhase) {
        Serial.print(F("Phase → "));
        Serial.println(phaseName(phase));

        if (phase == CAL_CHARGE) {
            Serial.println(F(">>> Discharge complete.  Connect your charger now. <<<"));
        }
        if (phase == CAL_DONE) {
            Serial.println(F(">>> Calibration successful!  Saved to EEPROM. <<<"));
            Serial.println(F("Upload a normal sketch (e.g. BasicBMS) for regular use."));
        }
        if (phase == CAL_ABORT) {
            Serial.println(F(">>> Calibration ABORTED (over-temperature or timeout). <<<"));
        }
        lastPhase = phase;
    }

    // Print live readings every cycle while calibrating
    if (phase != CAL_DONE && phase != CAL_ABORT && phase != CAL_NONE) {
        Serial.print(F("  V="));
        Serial.print(bms.getVoltage(), 3);
        Serial.print(F("V  I="));
        Serial.print(bms.getCurrent() * 1000.0f, 0);
        Serial.print(F("mA  T="));
        Serial.print(bms.getTemperature(), 1);
        Serial.println(F("°C"));
    }
}
