#pragma once

/*
 * LTC2944_BMS — Arduino library for the LTC2944 battery gas gauge.
 *
 * Wraps all LTC2944 communication, SOC estimation, calibration,
 * battery-disconnect detection, and adaptive internal-resistance
 * learning into a clean, easy-to-use API.
 *
 * Designed for ATmega328P (Arduino Pro Mini / Nano / Uno).
 * Uses Wire (I2C) and EEPROM internally.
 *
 * Typical usage:
 *
 *   #include <LTC2944_BMS.h>
 *
 *   LTC2944_BMS bms;
 *
 *   void setup() {
 *     bms.setProfile(CHEM_LIION, 1);        // 2S Li-ion
 *     bms.setCapacity(4200);                // 4200 mAh
 *     bms.setShuntResistor(0.001f);         // 1 mOhm shunt
 *     bms.setFullVoltage(8.30f);            // match your charger termination
 *     bms.setOutputMode(OUTPUT_SERIAL);
 *     bms.begin();
 *   }
 *
 *   void loop() {
 *     if (bms.update()) bms.print(Serial);
 *   }
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/pgmspace.h>

// ──────────────────────────────────────────────────────────────────────────────
// Output mode
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Serial output format, set via setOutputMode() before begin().
 *
 * OUTPUT_SERIAL  (default)
 *   Human-readable line, e.g.:
 *   SOC=65%  V=3.849V  I=-748mA  T=17.5C  CHG=NO  REM=1300mAh  RUN=1h44m
 *
 * OUTPUT_CSV
 *   Comma-separated values, header printed once in begin().
 *   soc_%,voltage_V,current_mA,temp_C,charging,remaining_mAh,runtime_min,alarms
 *   65,3.849,-748,17.5,0,1300,104,0
 *
 * OUTPUT_ESPHOME
 *   Same fields as CSV, terminated with \r\n for ESPHome UART debug sscanf.
 *   65,3.849,-748,17.5,0,1300,104,0\r\n
 */
enum OutputMode : uint8_t {
    OUTPUT_SERIAL   = 0,
    OUTPUT_CSV      = 1,
    OUTPUT_ESPHOME  = 2
};

// ──────────────────────────────────────────────────────────────────────────────
// Battery chemistry
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Battery chemistry types. Pass one of these to setProfile().
 *
 * Li-ion / LiFePO4 classCode -> series cell count:
 *   0=1S  1=2S  2=3S  3=4S  4=6S  5=8S  6=12S  7=14S
 *
 * AGM / GEL classCode -> 12 V block count:
 *   0=12V  1=24V  2=48V
 */
enum BatteryChem : uint8_t {
    CHEM_LIION   = 0,  ///< Li-ion / Li-polymer  (3.7 V nominal per cell)
    CHEM_LIFEPO4 = 1,  ///< Lithium Iron Phosphate (3.2 V nominal per cell)
    CHEM_AGM     = 2,  ///< AGM lead-acid (12 V per block)
    CHEM_GEL     = 3   ///< GEL lead-acid  (12 V per block)
};

// ──────────────────────────────────────────────────────────────────────────────
// Measurement struct
// ──────────────────────────────────────────────────────────────────────────────

struct BmsMeasurement {
    int      soc;            ///< State of charge 0-100 %
    float    voltage;        ///< Pack voltage in Volts
    float    current;        ///< Current in Amperes (positive = charging)
    float    temperature;    ///< Die temperature in degrees C
    uint16_t remainingMah;   ///< Remaining capacity in mAh (0 if capacity not set)
    int16_t  runtimeMinutes; ///< Estimated runtime in minutes (-1 if unknown)
    uint16_t rawACR;         ///< Raw accumulated charge register
    uint16_t rawCurrent;     ///< Raw current register
    bool     charging;       ///< true while charge current is detected
    uint16_t alarms;         ///< Bitfield - see ALARM_* constants
};

// ──────────────────────────────────────────────────────────────────────────────
// Alarm bits
// ──────────────────────────────────────────────────────────────────────────────
static const uint16_t ALARM_SENSOR_FAIL      = 0x0001; ///< LTC2944 I2C read failed
static const uint16_t ALARM_NO_PROFILE       = 0x0002; ///< Battery profile not resolved
static const uint16_t ALARM_UNDER_VOLTAGE    = 0x0004; ///< vpack < vEmpty
static const uint16_t ALARM_OVER_VOLTAGE     = 0x0008; ///< vpack > vFull
static const uint16_t ALARM_ABS_UNDER_V      = 0x0010; ///< vpack < vAbsLow
static const uint16_t ALARM_ABS_OVER_V       = 0x0020; ///< vpack > vAbsHigh
static const uint16_t ALARM_TEMPERATURE      = 0x0040; ///< Temperature out of range
static const uint16_t ALARM_OVER_CURRENT     = 0x0080; ///< Discharge current too high
static const uint16_t ALARM_CHARGE_CURRENT   = 0x0100; ///< Charge current too high
static const uint16_t ALARM_PROFILE_MISMATCH = 0x0200; ///< Voltage outside profile range
static const uint16_t ALARM_ACR_ROLLOVER     = 0x0400; ///< ACR counter rolled over

// ──────────────────────────────────────────────────────────────────────────────
// Calibration phase
// ──────────────────────────────────────────────────────────────────────────────
enum CalPhase : uint8_t {
    CAL_NONE      = 0, ///< Normal operation
    CAL_ZERO      = 1, ///< Sampling current offset at rest
    CAL_WAIT      = 2, ///< Settling before discharge
    CAL_DISCHARGE = 3, ///< Discharging to empty voltage
    CAL_CHARGE    = 4, ///< Waiting for full charge
    CAL_DONE      = 5, ///< Calibration complete
    CAL_ABORT     = 6  ///< Aborted (temperature / timeout)
};

// ──────────────────────────────────────────────────────────────────────────────
// Hardware pin defaults
// ──────────────────────────────────────────────────────────────────────────────
static const uint8_t BMS_DEFAULT_PIN_LOAD_EN    =  5;
static const uint8_t BMS_DEFAULT_PIN_STATUS_LED = 13;

// ──────────────────────────────────────────────────────────────────────────────
// EEPROM record — defined outside the class so sizeof() works at file scope
// ──────────────────────────────────────────────────────────────────────────────
struct LearnedRecord {
    uint16_t magic;
    uint8_t  version;
    uint8_t  length;
    uint32_t sequence;
    float    resistance;
    float    confidence;
    uint16_t samples;
    float    lastRelaxedVoltage;
    uint16_t lastSeenRawACR;
    uint8_t  storedChem;
    uint8_t  storedClassCode;
    uint8_t  crc;
};

// ──────────────────────────────────────────────────────────────────────────────
// LTC2944_BMS class
// ──────────────────────────────────────────────────────────────────────────────

class LTC2944_BMS {
public:

    LTC2944_BMS();

    // ── Configuration — call before begin() ─────────────────────────────────

    /**
     * Select battery chemistry and series-cell count.
     *
     * Li-ion / LiFePO4 classCode = number of series cells:
     *   1, 2, 3, 4, 6, 8, 12, 14
     *
     * AGM / GEL classCode = pack voltage in Volts:
     *   12, 24, 48
     *
     * Examples:
     *   bms.setProfile(CHEM_LIION,    2);  // 2S Li-ion   (7.4 V nominal)
     *   bms.setProfile(CHEM_LIFEPO4,  4);  // 4S LiFePO4  (12.8 V nominal)
     *   bms.setProfile(CHEM_AGM,     12);  // 12 V AGM
     */
    void setProfile(BatteryChem chem, uint8_t classCode);

    /**
     * Set nominal battery capacity in mAh.
     * Enables remaining-mAh, runtime estimation, and correct ACR prescaler.
     */
    void setCapacity(uint16_t capacityMah);

    /**
     * Override the full-charge voltage for the pack in Volts.
     * Useful when your charger terminates at a non-standard voltage,
     * e.g. 8.30V instead of the profile default of 8.40V for a 2S pack.
     * Set to 0 (default) to use the profile table value.
     */
    void setFullVoltage(float volts);

    /**
     * Override the empty voltage for the pack in Volts.
     * Set to 0 (default) to use the profile table value.
     */
    void setEmptyVoltage(float volts);

    /**
     * Set shunt resistor value in Ohms.
     * Default: 0.001 (1 mOhm) — matches reference hardware.
     */
    void setShuntResistor(float ohms);

    /**
     * Set the calibration timeout in minutes.
     * Default: 240 minutes (4 hours).
     * Set longer for large battery packs with low discharge currents.
     */
    void setCalibrationTimeout(uint32_t minutes);

    /**
     * Set the charge-taper current threshold for full-charge detection
     * during calibration in Amperes.
     * Default: 0.1 A (100 mA).
     * Increase if your charger terminates at a higher current (e.g. 0.5 A
     * for chargers that use the 0.1C rule with large packs).
     * Requires CAL_FULL_CONFIRM_SAMPLES consecutive readings below this
     * threshold before declaring full.
     */
    void setCalibrationFullCurrent(float amps);

    /**
     * Set the minimum current threshold to consider a charger connected
     * during calibration, in Amperes.
     * Default: 0.030 A (30 mA).
     * The hard-cutoff detection path (CHG YES->NO at near-full voltage) does
     * not use this threshold — it works regardless of this setting.
     */
    void setCalibrationChargerOnCurrent(float amps);

    /**
     * Set the maximum discharge current alarm threshold in Amperes.
     * Default: 5.5 A.
     */
    void setMaxDischargeCurrent(float amps);

    /**
     * Set the maximum charge current alarm threshold in Amperes.
     * Default: 3.0 A. Increase for large packs with high charge rates.
     */
    void setMaxChargeCurrent(float amps);

    /** Select serial output format. Default: OUTPUT_SERIAL. */
    void setOutputMode(OutputMode mode);

    /** Override load-enable pin. Default: 5. */
    void setPinLoadEnable(uint8_t pin);

    /** Override status LED pin. Default: 13. */
    void setPinStatusLed(uint8_t pin);

    /** Set current deadband in Amperes. Default: 0.030 A. */
    void setCurrentDeadband(float amps);

    // ── Initialisation ───────────────────────────────────────────────────────

    /**
     * Initialise the library.
     * Call once in setup() after all setXxx() calls.
     * @return true if LTC2944 found and configured.
     */
    bool begin();

    // ── Main loop ────────────────────────────────────────────────────────────

    /**
     * Read all LTC2944 registers and update measurements.
     * Enforces 1-second minimum between reads.
     * @return true when a fresh measurement is available.
     */
    bool update();

    // ── Output ───────────────────────────────────────────────────────────────

    /**
     * Print latest measurement in the selected output mode.
     * Call after update() returns true.
     * @param port  Any Stream — Serial, Serial1, SoftwareSerial, etc.
     */
    void print(Stream& port);

    /**
     * Print the CSV header line.
     * Called automatically by begin() when OUTPUT_CSV is selected.
     */
    void printCsvHeader(Stream& port);

    // ── Measurement accessors ────────────────────────────────────────────────

    int      getSoc()            const;
    float    getVoltage()        const;
    float    getCurrent()        const;
    float    getTemperature()    const;
    uint16_t getAlarms()         const;
    bool     isCharging()        const;
    uint16_t getRemainingMah()   const;
    int16_t  getRuntimeMinutes() const;
    uint16_t getCapacityMah()    const;
    uint8_t  getPrescaler()      const;
    BmsMeasurement getMeasurement() const;

    // ── Disconnect detection ─────────────────────────────────────────────────

    bool isBatteryDisconnected() const;

    // ── Calibration ─────────────────────────────────────────────────────────

    void     startCalibration();
    void     stopCalibration();
    CalPhase getCalPhase()  const;
    bool     isCalibrated() const;
    void     resetCalibration();

    // ── Learned model ────────────────────────────────────────────────────────

    float getLearnedResistance()  const;
    float getLearningConfidence() const;
    void  resetForNewBattery();

    // ── Utility ──────────────────────────────────────────────────────────────

    const char* getProfileName()     const;
    bool        isProfilePlausible() const;
    bool        reinitChip();

    // ──────────────────────────────────────────────────────────────────────────
    // Private
    // ──────────────────────────────────────────────────────────────────────────
private:

    // Config
    BatteryChem _chem;
    uint8_t     _classCode;
    float       _shuntOhm;
    float       _overrideFullV;   // 0 = use profile
    float       _overrideEmptyV;  // 0 = use profile
    uint8_t     _pinLoadEn;
    uint8_t     _pinStatusLed;
    float       _currentDeadbandA;
    uint16_t    _capacityMah;
    uint8_t     _prescaler;
    OutputMode  _outputMode;
    uint32_t    _calTimeoutMs;
    float       _calFullCurrentA;
    float       _maxDischargeA;
    float       _maxChargeA;

    // Live state
    BmsMeasurement _meas;
    bool        _disconnected;
    bool        _profileMismatch;
    bool        _charging;
    bool        _calibrationMode;
    CalPhase    _calPhase;
    uint32_t    _calPhaseStartMs;
    uint32_t    _calRunStartMs;
    uint16_t    _zeroSamplesTaken;
    int32_t     _zeroAccumRawDelta;
    uint32_t    _lastSendMs;
    float       _currentOffset_mA;
    uint8_t     _disconnectCandidateCount;
    uint8_t     _i2cFailCount;

    // Full-charge confirmation (multiple samples to avoid false triggers)
    uint8_t     _calFullConfirmCount;

    // Hard-cutoff charger detection
    // Some chargers cut power abruptly at full rather than tapering current.
    // We detect this by watching for CHG YES->NO transition at near-full voltage.
    bool        _prevCharging;          // charging state from previous cycle
    float       _calChargerOnCurrentA;  // configurable charger-on threshold

    // ACR rollover tracking
    uint16_t    _lastRawACR;
    bool        _acrRollover;

    // Profile
    struct BatteryProfileInternal {
        bool        valid;
        BatteryChem chem;
        uint8_t     classCode;
        const char* name;
        uint8_t     seriesCount;
        bool        leadBased;
        float       vEmpty, vNominal, vFull, vAbsLow, vAbsHigh;
    };
    BatteryProfileInternal _profile;
    char _profileNameBuf[16];

    // Calibration
    struct CalData {
        float    vEmpty, vFull;
        uint16_t emptyChargeRaw, fullChargeRaw;
        bool     valid, hasEmptyRaw, hasFullRaw;
    };
    CalData _cal;

    // Learned model
    struct LearnedModel {
        float    resistance;
        float    confidence;
        bool     valid, dirty;
        uint16_t samples;
        float    lastRelaxedVoltage;
        uint16_t lastSeenRawACR;
        uint8_t  storedChem;
        uint8_t  storedClassCode;
    };
    LearnedModel  _model;

    struct ResLearnState {
        float    baselineV;
        float    baselineI;
        uint32_t baselineMs;
        bool     baselineValid;
    };
    ResLearnState _resLearn;
    uint32_t      _lastLearnedSaveMs;
    uint32_t      _lastRelaxedSeenMs;

    // Private methods
    bool     _writeReg8(uint8_t reg, uint8_t val);
    bool     _writeReg16(uint8_t reg, uint16_t val);
    bool     _readReg8(uint8_t reg, uint8_t& val);
    bool     _readReg16(uint8_t reg, uint16_t& val);

    float    _rawToVoltage(uint16_t raw) const;
    float    _rawToCurrent(uint16_t raw) const;
    float    _rawToCurrentNoOffset(uint16_t raw) const;
    float    _rawToTemperature(uint16_t raw) const;
    float    _rawAcrToMah(uint16_t raw) const;

    // Effective full/empty voltages (override or profile)
    float    _effectiveVFull()  const;
    float    _effectiveVEmpty() const;

    uint8_t  _computePrescaler() const;
    bool     _writePrescaler(uint8_t m);

    void     _resolveProfile();
    bool     _isVoltagePlausible(float v) const;
    bool     _checkAcrRollover(uint16_t rawACR);
    int      _estimateSoc(float vbat, float currentA, uint16_t rawACR, bool charging);
    int      _socFromVoltageCurve(float vbat, float currentA, uint16_t rawACR, bool charging,
                                   float chargeCompV, int lowerBandSoc, int upperBandSoc);
    int      _estimateCoulombSoc(uint16_t rawACR) const;
    int      _adaptiveBlend(int coulombSoc, int voltageSoc, float currentA) const;
    uint16_t _makeAlarms(float v, float i, float t, bool sensorOk) const;

    void     _updateCalibration(uint16_t rawCurr, uint16_t rawACR,
                                 float vbat, float tempC, float currentA);
    void     _enterCalPhase(CalPhase phase);
    void     _setLoad(bool en);

    bool     _checkDisconnectOnReadFailure();
    bool     _checkDisconnectOnMeasurement(float vbat, float currentA);
    void     _writeDisconnectFlag();
    void     _clearDisconnectFlag();
    bool     _readDisconnectFlag() const;

    void     _loadOffset();
    void     _saveOffset(float offset_mA);
    void     _loadCalibration();
    void     _saveCalEmpty(float v, uint16_t raw);
    void     _saveCalFull(float v, uint16_t raw);

    void     _resetLearnedModel(bool newBattery);
    bool     _loadLearnedModel();
    bool     _saveLearnedModel(bool force);
    void     _updateResistanceLearning(float v, float i, uint32_t now);
    void     _updateRelaxedSignature(float v, float i, uint16_t rawACR, uint32_t now);
    float    _effectiveResistance() const;

    uint8_t  _crc8(const uint8_t* data, size_t len) const;
    uint8_t  _recordCrc(const LearnedRecord& r) const;
    bool     _isRecordValid(const LearnedRecord& r) const;

    static int   _clamp(int v, int lo, int hi);
    static float _clampf(float v, float lo, float hi);
    static float _absf(float v);
    bool         _currentNearZero(float i) const;
    void         _copyProfileName();
};
