/*
 * ESPHome.ino — LTC2944_BMS with ESPHome UART output
 *
 * Sends battery measurements over UART in the compact ESPHome format.
 * An ESP32 running the companion ltc2944_bms_esphome.yaml reads these
 * lines and publishes them as Home Assistant sensors via MQTT/API.
 *
 * Output format — one line per second, terminated with \r\n:
 *   soc,voltage,current_mA,temp_C,charging,remaining_mAh,runtime_min,alarms\r\n
 *   Example: 65,3.849,-748,17.5,0,1300,104,0\r\n
 *
 * Lines prefixed with '#' are diagnostic messages — the ESPHome lambda
 * skips them automatically.
 *
 * Wiring:
 *   Pro Mini TX  ->  ESP32 RX (GPIO16 in ltc2944_bms_esphome.yaml)
 *   Pro Mini GND ->  ESP32 GND
 *   (one-way — no data flows from ESP32 to Pro Mini)
 *
 * Baud rate: 9600 — must match the ESPHome YAML uart: baud_rate setting.
 *
 * Hardware:
 *   Arduino Pro Mini (ATmega328P) + LTC2944 reference PCB
 *   I2C: SDA=A4, SCL=A5  |  Shunt: 1 mOhm (R1)
 */

#include <LTC2944_BMS.h>

LTC2944_BMS bms;

void setup() {
    Serial.begin(9600);

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
    bms.setProfile(CHEM_LIION, 1);        // <- change to match your battery

    // ── 2. Set battery capacity in mAh ──────────────────────────────────────
    bms.setCapacity(2000);                // <- change to match your battery

    // ── 3. Set shunt resistor value ─────────────────────────────────────────
    bms.setShuntResistor(0.001f);         // 1 mOhm — reference hardware

    // ── 4. Optional: override full/empty voltage to match your charger ───────
    //   Improves SOC accuracy when your charger uses a non-standard cutoff.
    //   Example for a charger set to 4.15V/cell on a 2S pack:
    //     bms.setFullVoltage(8.30f);

    // ── 5. Optional: adjust alarm thresholds ────────────────────────────────
    //   Increase the charge current alarm for large packs:
    //     bms.setMaxChargeCurrent(5.0f);

    // ── 6. ESPHome output mode ───────────────────────────────────────────────
    //   Compact CSV terminated with \r\n — parsed by the ESPHome YAML lambda.
    bms.setOutputMode(OUTPUT_ESPHOME);

    // ── 7. Initialise ────────────────────────────────────────────────────────
    if (!bms.begin()) {
        while (true) {
            Serial.println(F("# ERROR: LTC2944 not found — check wiring!"));
            delay(5000);
        }
    }

    // Startup diagnostics prefixed with '#' — ESPHome lambda skips these
    Serial.print(F("# Profile  : ")); Serial.println(bms.getProfileName());
    Serial.print(F("# Capacity : ")); Serial.print(bms.getCapacityMah()); Serial.println(F(" mAh"));
    Serial.print(F("# Prescaler: M=")); Serial.println(bms.getPrescaler());
    Serial.println(bms.isCalibrated()
        ? F("# Calibrated — coulomb+voltage SOC blend active")
        : F("# No calibration — voltage-curve SOC only"));
}

void loop() {
    if (!bms.update()) return;

    if (bms.isBatteryDisconnected()) {
        Serial.println(F("# Battery disconnected"));
        delay(5000);
        return;
    }

    // Outputs: soc,voltage,current_mA,temp_C,charging,remaining_mAh,runtime_min,alarms\r\n
    bms.print(Serial);
}
