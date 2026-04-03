/*
 * Calibration.ino — full discharge/charge calibration cycle
 *
 * What it does:
 *   1. Zeroes the current offset at rest (no load, no charger).
 *   2. Enables the discharge load (load-enable pin) until empty voltage.
 *   3. Prompts you to connect a charger; waits until the battery is full.
 *   4. Saves both ACR anchor points to EEPROM.
 *      After this, the library blends voltage-SOC with coulomb-SOC
 *      for improved accuracy across the full charge curve.
 *
 * Requirements:
 *   - A discharge load wired to the load-enable pin (default pin 5).
 *   - Serial Monitor open at 9600 baud.
 *
 * IMPORTANT:
 *   - Do NOT leave unattended — this sketch drains the battery.
 *   - Calibration aborts automatically if temperature exceeds 55 C
 *     or if either phase exceeds the configured timeout.
 *   - Start with a partially charged battery (20-80% recommended).
 *     A fully charged battery will take longer to discharge.
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
    bms.setProfile(CHEM_LIION, 2);      // 2S Li-ion

    // ── 2. Set battery capacity in mAh ──────────────────────────────────────
    bms.setCapacity(280);              // 280 mAh

    // ── 3. Set shunt resistor value ─────────────────────────────────────────
    bms.setShuntResistor(0.001f);       // 1 mOhm — reference hardware

    // ── 4. Optional: override full/empty voltage to match your charger ───────
    //   If your charger terminates at a non-standard voltage, set it here.
    //   This is especially important for calibration accuracy.
    //   Example for a charger set to 4.15V/cell on a 2S pack:
    //     bms.setFullVoltage(8.30f);
    //     bms.setEmptyVoltage(6.00f);

    // ── 5. Calibration timeout ───────────────────────────────────────────────
    //   Default is 240 minutes (4 hours).
    //   Increase for large packs with low discharge current.
    //   Formula: timeout > (capacity_mAh / discharge_mA) * 60 * 1.2
    //   Example for 10Ah pack at 1A: (10000/1000)*60*1.2 = 720 min
    //     bms.setCalibrationTimeout(720);

    // ── 6. Full-charge taper current threshold ───────────────────────────────
    //   The library waits for charge current to drop below this value
    //   for 5 consecutive seconds before declaring full.
    //   Default: 0.10 A (100 mA) — suitable for most chargers.
    //   Increase if your charger terminates at a higher current (e.g. 0.5 A):
    //     bms.setCalibrationFullCurrent(0.5f);

    // ── 7. Initialise ────────────────────────────────────────────────────────
    if (!bms.begin()) {
        Serial.println(F("ERROR: LTC2944 not found!"));
        while (true) {}
    }

    Serial.print(F("Profile  : ")); Serial.println(bms.getProfileName());
    Serial.print(F("Capacity : ")); Serial.print(bms.getCapacityMah()); Serial.println(F(" mAh"));
    Serial.print(F("Prescaler: M=")); Serial.println(bms.getPrescaler());
    Serial.println();
    Serial.println(F("Starting calibration..."));
    Serial.println(F("Ensure battery is at rest — no load, no charger."));
    Serial.println();

    bms.startCalibration();
}

static const char* phaseName(CalPhase p) {
    switch (p) {
        case CAL_NONE:       return "IDLE";
        case CAL_ZERO:       return "ZEROING CURRENT OFFSET";
        case CAL_WAIT:       return "SETTLING";
        case CAL_DISCHARGE:  return "DISCHARGING TO EMPTY";
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

    // Print a message on every phase transition
    if (phase != lastPhase) {
        Serial.print(F("Phase -> ")); Serial.println(phaseName(phase));

        if (phase == CAL_CHARGE) {
            Serial.println(F(">>> Discharge complete — connect your charger now. <<<"));
        }
        if (phase == CAL_DONE) {
            Serial.println(F(">>> Calibration complete! Data saved to EEPROM.    <<<"));
            Serial.println(F(">>> Upload BasicBMS for normal operation.           <<<"));
        }
        if (phase == CAL_ABORT) {
            Serial.println(F(">>> Calibration ABORTED — see # messages above for reason. <<<"));
            Serial.println(F(">>> Fix the issue and restart the Calibration sketch.       <<<"));
        }
        lastPhase = phase;
    }

    // Print live readings during active calibration phases
    if (phase != CAL_DONE && phase != CAL_ABORT && phase != CAL_NONE) {
        bms.print(Serial);
    }
}
