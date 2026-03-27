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
 *     bms.setProfile(CHEM_LIION, 1);   // 1S Li-ion
 *     bms.begin();
 *   }
 *
 *   void loop() {
 *     if (bms.update()) {
 *       Serial.print("SOC: "); Serial.println(bms.getSoc());
 *       Serial.print("V:   "); Serial.println(bms.getVoltage());
 *     }
 *   }
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/pgmspace.h>

// ──────────────────────────────────────────────────────────────────────────────
// Battery chemistry selector
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Battery chemistry types supported by the library.
 * Pass one of these to setProfile().
 */
enum BatteryChem : uint8_t {
    CHEM_LIION   = 0,  ///< Li-ion / Li-polymer (3.7 V nominal per cell)
    CHEM_LIFEPO4 = 1,  ///< Lithium Iron Phosphate (3.2 V nominal per cell)
    CHEM_AGM     = 2,  ///< AGM lead-acid (12 V per block)
    CHEM_GEL     = 3   ///< GEL lead-acid  (12 V per block)
};

// ──────────────────────────────────────────────────────────────────────────────
// Public data types
// ──────────────────────────────────────────────────────────────────────────────

/** All live measurements returned by update(). */
struct BmsMeasurement {
    int      soc;          ///< State of charge 0–100 %
    float    voltage;      ///< Pack voltage in Volts
    float    current;      ///< Current in Amperes (positive = charging)
    float    temperature;  ///< Die temperature in °C
    uint16_t rawACR;       ///< Raw accumulated charge register (for logging)
    uint16_t rawCurrent;   ///< Raw current register (for logging / CRC)
    bool     charging;     ///< true while charge current is detected
    uint16_t alarms;       ///< Bitfield — see ALARM_* constants below
};

// ──────────────────────────────────────────────────────────────────────────────
// Alarm bits (BmsMeasurement::alarms)
// ──────────────────────────────────────────────────────────────────────────────
static const uint16_t ALARM_SENSOR_FAIL    = 0x0001; ///< LTC2944 I²C read failed
static const uint16_t ALARM_NO_PROFILE     = 0x0002; ///< Battery profile not resolved
static const uint16_t ALARM_UNDER_VOLTAGE  = 0x0004; ///< vpack < vEmpty
static const uint16_t ALARM_OVER_VOLTAGE   = 0x0008; ///< vpack > vFull
static const uint16_t ALARM_ABS_UNDER_V    = 0x0010; ///< vpack < vAbsLow (hardware fault)
static const uint16_t ALARM_ABS_OVER_V     = 0x0020; ///< vpack > vAbsHigh (hardware fault)
static const uint16_t ALARM_TEMPERATURE    = 0x0040; ///< Temperature out of –10…60 °C
static const uint16_t ALARM_OVER_CURRENT   = 0x0080; ///< Discharge current > 5.5 A
static const uint16_t ALARM_CHARGE_CURRENT = 0x0100; ///< Charge current > 3.0 A
static const uint16_t ALARM_PROFILE_MISMATCH = 0x0200;///< Measured voltage outside profile range

// ──────────────────────────────────────────────────────────────────────────────
// Calibration phase
// ──────────────────────────────────────────────────────────────────────────────
/**
 * Calibration state machine phases, readable via getCalPhase().
 * Calibration is optional; without it the library uses voltage-curve SOC only.
 */
enum CalPhase : uint8_t {
    CAL_NONE      = 0, ///< Normal operation — no calibration running
    CAL_ZERO      = 1, ///< Sampling current offset at rest
    CAL_WAIT      = 2, ///< Settling delay before enabling discharge load
    CAL_DISCHARGE = 3, ///< Discharging to empty voltage
    CAL_CHARGE    = 4, ///< Waiting for full charge
    CAL_DONE      = 5, ///< Calibration complete
    CAL_ABORT     = 6  ///< Calibration aborted (temperature / timeout)
};

// ──────────────────────────────────────────────────────────────────────────────
// Hardware pin defaults  (change with setPin*() before begin())
// ──────────────────────────────────────────────────────────────────────────────
static const uint8_t BMS_DEFAULT_PIN_LOAD_EN    =  5; ///< Load MOSFET / relay
static const uint8_t BMS_DEFAULT_PIN_STATUS_LED = 13; ///< Status LED

// ──────────────────────────────────────────────────────────────────────────────
// LTC2944_BMS class
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Main BMS class.  Create one instance in your sketch.
 */
class LTC2944_BMS {
public:
    // ── Construction & configuration ────────────────────────────────────────

    LTC2944_BMS();

    /**
     * Select the battery chemistry and class (series-cell count or 12 V block
     * count for lead-acid).
     *
     * Must be called before begin().
     *
     * Li-ion / LiFePO4 classCode → series cell count:
     *   0 = 1S, 1 = 2S, 2 = 3S, 3 = 4S, 4 = 6S, 5 = 8S, 6 = 12S, 7 = 14S
     *
     * AGM / GEL classCode → 12 V block count:
     *   0 = 12 V, 1 = 24 V, 2 = 48 V
     *
     * @param chem       Battery chemistry (CHEM_LIION, CHEM_LIFEPO4, CHEM_AGM, CHEM_GEL)
     * @param classCode  Series cell / block count selector (see above)
     */
    void setProfile(BatteryChem chem, uint8_t classCode);

    /**
     * Override the shunt resistor value (default 10 mΩ).
     * Call before begin() if your hardware uses a different shunt.
     * @param ohms  Shunt resistance in Ohms (e.g. 0.010 for 10 mΩ)
     */
    void setShuntResistor(float ohms);

    /**
     * Override the GPIO pin used to drive the calibration discharge load.
     * Default: pin 5.
     */
    void setPinLoadEnable(uint8_t pin);

    /**
     * Override the GPIO pin used to drive the status LED.
     * Default: pin 13.
     */
    void setPinStatusLed(uint8_t pin);

    /**
     * Set a current deadband in Amperes (default 30 mA).
     * Currents whose absolute value is below this threshold are reported as 0.
     */
    void setCurrentDeadband(float amps);

    // ── Initialisation ──────────────────────────────────────────────────────

    /**
     * Initialise the library: Wire.begin(), EEPROM load, LTC2944 setup.
     *
     * Call once in setup() after setProfile() (and any other setXxx() calls).
     *
     * @return true if the LTC2944 was found and configured successfully.
     */
    bool begin();

    // ── Main loop call ───────────────────────────────────────────────────────

    /**
     * Read all LTC2944 registers, compute SOC, update alarms and learned model.
     *
     * Call this every loop iteration (or at whatever interval suits your sketch).
     * The library enforces a 1-second minimum between actual sensor reads; calls
     * within that window return false immediately.
     *
     * @return true when a new measurement was taken and getXxx() values are fresh.
     */
    bool update();

    // ── Measurement accessors (valid after update() returns true) ────────────

    /** State of charge 0–100 %. */
    int     getSoc()         const;

    /** Pack voltage in Volts. */
    float   getVoltage()     const;

    /** Current in Amperes (positive = charging, negative = discharging). */
    float   getCurrent()     const;

    /** Die temperature in °C. */
    float   getTemperature() const;

    /** Alarm bitfield — OR of ALARM_* constants. */
    uint16_t getAlarms()     const;

    /** true while a charge current is detected. */
    bool    isCharging()     const;

    /** Copy the last measurement struct. */
    BmsMeasurement getMeasurement() const;

    // ── Disconnect detection ─────────────────────────────────────────────────

    /**
     * Returns true if the library has detected a battery disconnect and written
     * the disconnect flag to EEPROM.  The sketch should stop sending telemetry
     * and wait for reconnect / reboot.
     */
    bool isBatteryDisconnected() const;

    // ── Calibration ─────────────────────────────────────────────────────────

    /**
     * Start the automatic calibration sequence.
     * The load pin (PIN_LOAD_EN) will be driven HIGH during the discharge phase.
     * Monitor getCalPhase() to track progress.
     */
    void startCalibration();

    /** Stop / abort any running calibration. */
    void stopCalibration();

    /** Current calibration phase. */
    CalPhase getCalPhase() const;

    /** True if calibration data (empty/full voltage + ACR window) is available. */
    bool isCalibrated() const;

    /** Erase all calibration data from EEPROM and RAM. */
    void resetCalibration();

    // ── Learned internal-resistance model ────────────────────────────────────

    /**
     * Current learned internal resistance in Ohms.
     * Returns the conservative default (80 mΩ) until enough samples have been
     * collected (confidence ≥ 0.10).
     */
    float getLearnedResistance() const;

    /**
     * Confidence of the learned resistance model, 0.0–1.0.
     * Reaches 1.0 after approximately 12 good current-step samples.
     */
    float getLearningConfidence() const;

    /**
     * Call this when a new battery is fitted (resets the learned model and
     * calibration data).  Equivalent to the NEW_BAT jumper in the hardware design.
     */
    void resetForNewBattery();

    // ── Utility ──────────────────────────────────────────────────────────────

    /**
     * Returns the name string of the currently resolved battery profile,
     * e.g. "LIION_3S".  Returns "INVALID" if the profile is not found.
     */
    const char* getProfileName() const;

    /**
     * True if the measured voltage is within the plausibility window of the
     * selected profile.
     */
    bool isProfilePlausible() const;

    /**
     * Re-initialise the LTC2944 over I²C (useful after a transient I²C error).
     * @return true on success.
     */
    bool reinitChip();

    // ──────────────────────────────────────────────────────────────────────────
    // Private implementation
    // ──────────────────────────────────────────────────────────────────────────
private:

    // ── Hardware config ──────────────────────────────────────────────────────
    BatteryChem _chem;
    uint8_t     _classCode;
    float       _shuntOhm;
    uint8_t     _pinLoadEn;
    uint8_t     _pinStatusLed;
    float       _currentDeadbandA;

    // ── Live state ───────────────────────────────────────────────────────────
    BmsMeasurement _meas;
    bool        _disconnected;
    bool        _profileMismatch;
    bool        _charging;          // hysteretic charging flag
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

    // ── Profile ──────────────────────────────────────────────────────────────
    struct BatteryProfileInternal {
        bool    valid;
        BatteryChem chem;
        uint8_t classCode;
        const char* name;      // PROGMEM pointer
        uint8_t seriesCount;
        bool    leadBased;
        float   vEmpty, vNominal, vFull, vAbsLow, vAbsHigh;
    };
    BatteryProfileInternal _profile;
    char _profileNameBuf[16];

    // ── Calibration data ─────────────────────────────────────────────────────
    struct CalData {
        float    vEmpty, vFull;
        uint16_t emptyChargeRaw, fullChargeRaw;
        bool     valid, hasEmptyRaw, hasFullRaw;
    };
    CalData _cal;

    // ── Learned resistance model ─────────────────────────────────────────────
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
    LearnedModel _model;

    struct ResLearnState {
        float    baselineV;
        float    baselineI;
        uint32_t baselineMs;
        bool     baselineValid;
    };
    ResLearnState _resLearn;
    uint32_t _lastLearnedSaveMs;
    uint32_t _lastRelaxedSeenMs;

    // ── EEPROM record for learned model ──────────────────────────────────────
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

    // ── Private methods ───────────────────────────────────────────────────────
    bool     _writeReg8(uint8_t reg, uint8_t val);
    bool     _readReg8(uint8_t reg, uint8_t& val);
    bool     _readReg16(uint8_t reg, uint16_t& val);

    float    _rawToVoltage(uint16_t raw) const;
    float    _rawToCurrent(uint16_t raw) const;
    float    _rawToCurrentNoOffset(uint16_t raw) const;
    float    _rawToTemperature(uint16_t raw) const;

    void     _resolveProfile();
    bool     _isVoltagePlausible(float v) const;
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

    static int _clamp(int v, int lo, int hi);
    static float _clampf(float v, float lo, float hi);
    static float _absf(float v);
    bool _currentNearZero(float i) const;

    // Copy PROGMEM name string to _profileNameBuf
    void _copyProfileName();
};
