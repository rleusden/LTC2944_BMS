#pragma once

/*
 * LTC2944_BMS — Arduino library for the LTC2944 battery gas gauge.
 *
 * Designed for ATmega328P (Arduino Pro Mini / Nano / Uno).
 * Uses Wire (I2C) and EEPROM internally.
 *
 * Changes in this version
 * ───────────────────────
 * FIX  IR compensation direction during charging was inverted.
 *      The signed current term (currentA * irOhm) already handles both
 *      charge and discharge correctly because currentA is positive during
 *      charging.  The old code subtracted an additional chargeCompV offset on
 *      top, pushing the OCV estimate ~10-20 % too high during a charge cycle
 *      and causing voltage-SOC to read well above the coulomb SOC.
 *      chargeCompV is now added (not subtracted) so it provides a small
 *      upward correction for the CC-phase OCV lag rather than amplifying it.
 *
 * FIX  Resistance learner now only updates on discharge step transitions.
 *      During charging, rising OCV adds to dV in the same direction as the
 *      IR drop, making dV/dI appear much larger than true internal resistance.
 *      The learner now rejects any transition where the new current is not
 *      negative (i.e. not a discharge step from rest).
 *
 * FIX  Per-profile plausible-resistance cap.  A cap of 60 mΩ × seriesCount
 *      is applied before each EMA update so a single bad measurement cannot
 *      inflate the learned R for many subsequent cycles.
 *
 * NEW  Coulombic efficiency (η) correction — setEfficiency().
 *      Separate charge and discharge efficiency factors are applied to a
 *      floating-point ACR shadow register so that round-trip losses do not
 *      accumulate as SOC drift.  Defaults: ηc = 0.99, ηd = 1.00.
 *
 * NEW  OCV-SOC lookup tables in PROGMEM for Li-ion and LiFePO4.
 *      11-point per-chemistry curves replace the two-segment linear
 *      interpolation and accurately model the flat plateau region.
 *      Lead-acid chemistries retain piecewise linear interpolation.
 *
 * NEW  SOH (State of Health) — getSoh().
 *      At the end of each full calibration cycle, measured Cmax is compared
 *      to the rated capacity and a SOH % is computed and persisted to EEPROM.
 *
 * NEW  Temperature capacity derating.
 *      remainingMah and runtimeMinutes are scaled by a per-chemistry
 *      piecewise-linear derating factor based on measured die temperature.
 *      Model: -20 °C → 60 %, 25 °C → 100 %, 60 °C → 95 %.
 *      SOC itself is unaffected; only derived energy outputs are derated.
 *
 * NEW  Self-discharge correction — setSelfDischargeRate().
 *      A configurable rate (mAh/hour, default 0.05) is subtracted from the
 *      eta-corrected ACR accumulator each update tick.  On boot the elapsed
 *      time since the last EEPROM timestamp is used to back-apply the loss.
 *
 * Typical usage (unchanged):
 *
 *   LTC2944_BMS bms;
 *   void setup() {
 *     bms.setProfile(CHEM_LIION, 2);
 *     bms.setCapacity(4200);
 *     bms.setShuntResistor(0.001f);
 *     bms.setFullVoltage(8.30f);
 *     bms.begin();
 *   }
 *   void loop() {
 *     if (bms.update()) bms.print(Serial);
 *   }
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/pgmspace.h>


#define LTC2944_BMS_ENABLE_DEBUG 0



// ─── Output mode ─────────────────────────────────────────────────────────────
enum OutputMode : uint8_t {
    OUTPUT_SERIAL   = 0,
    OUTPUT_CSV      = 1,
    OUTPUT_ESPHOME  = 2
};

// ─── Battery chemistry ────────────────────────────────────────────────────────
enum BatteryChem : uint8_t {
    CHEM_LIION   = 0,
    CHEM_LIFEPO4 = 1,
    CHEM_AGM     = 2,
    CHEM_GEL     = 3
};

// ─── Measurement struct ───────────────────────────────────────────────────────
struct BmsMeasurement {
    int      soc;            ///< State of charge 0-100 %
    int      soh;            ///< State of health 0-100 % (100 until first full cal)
    float    voltage;
    float    current;        ///< Amperes, positive = charging
    float    temperature;    ///< Die temperature °C
    uint16_t remainingMah;   ///< Temperature-derated remaining capacity
    int16_t  runtimeMinutes; ///< -1 if unknown
    uint16_t rawACR;
    uint16_t rawCurrent;
    bool     charging;
    uint16_t alarms;
};

// ─── Debug state ──────────────────────────────────────────────────────────────
#if LTC2944_BMS_ENABLE_DEBUG
enum DebugSocSource : uint8_t {
    DEBUG_SOC_UNKNOWN          = 0,
    DEBUG_SOC_VOLTAGE_BOOT     = 1,
    DEBUG_SOC_VOLTAGE_ROLLOVER = 2,
    DEBUG_SOC_VOLTAGE_ONLY     = 3,
    DEBUG_SOC_BLEND            = 4
};

struct BmsDebugState {
    int      finalSoc;
    int      voltageSoc;
    int      coulombSoc;         ///< -1 if unavailable
    uint8_t  socSource;
    float    effectiveVEmpty;
    float    effectiveVFull;
    float    effectiveResistance;
    float    compensatedCellV;
    float    compensatedPackV;
    float    tempDerateFactor;
    uint16_t rawVoltage;
    uint16_t rawCurrent;
    uint16_t rawACR;
    uint8_t  acrBootTicks;
    bool     acrRollover;
    bool     haveCalibration;
    bool     usedCoulomb;
    bool     charging;
};
#else
struct BmsDebugState {};
#endif

// ─── Alarm bits ───────────────────────────────────────────────────────────────
static const uint16_t ALARM_SENSOR_FAIL      = 0x0001;
static const uint16_t ALARM_NO_PROFILE       = 0x0002;
static const uint16_t ALARM_UNDER_VOLTAGE    = 0x0004;
static const uint16_t ALARM_OVER_VOLTAGE     = 0x0008;
static const uint16_t ALARM_ABS_UNDER_V      = 0x0010;
static const uint16_t ALARM_ABS_OVER_V       = 0x0020;
static const uint16_t ALARM_TEMPERATURE      = 0x0040;
static const uint16_t ALARM_OVER_CURRENT     = 0x0080;
static const uint16_t ALARM_CHARGE_CURRENT   = 0x0100;
static const uint16_t ALARM_PROFILE_MISMATCH = 0x0200;
static const uint16_t ALARM_ACR_ROLLOVER     = 0x0400;

// ─── Calibration phase ────────────────────────────────────────────────────────
enum CalPhase : uint8_t {
    CAL_NONE      = 0,
    CAL_ZERO      = 1,
    CAL_WAIT      = 2,
    CAL_DISCHARGE = 3,
    CAL_CHARGE    = 4,
    CAL_DONE      = 5,
    CAL_ABORT     = 6
};

// ─── Hardware pin defaults ────────────────────────────────────────────────────
static const uint8_t BMS_DEFAULT_PIN_LOAD_EN    =  5;
static const uint8_t BMS_DEFAULT_PIN_STATUS_LED = 13;

// ─── EEPROM record ────────────────────────────────────────────────────────────
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

// ─── Main class ───────────────────────────────────────────────────────────────
class LTC2944_BMS {
public:

    LTC2944_BMS();

    // Configuration — call before begin()
    void setProfile(BatteryChem chem, uint8_t classCode);
    void setCapacity(uint16_t capacityMah);
    void setFullVoltage(float volts);
    void setEmptyVoltage(float volts);
    void setShuntResistor(float ohms);
    void setCalibrationTimeout(uint32_t minutes);
    void setCalibrationFullCurrent(float amps);
    void setCalibrationChargerOnCurrent(float amps);
    void setMaxDischargeCurrent(float amps);
    void setMaxChargeCurrent(float amps);
    void setOutputMode(OutputMode mode);
    void setPinLoadEnable(uint8_t pin);
    void setPinStatusLed(uint8_t pin);
    void setCurrentDeadband(float amps);

    /**
     * Set Coulombic efficiency factors (0.0–1.0).
     * etaCharge    — fraction of charge accepted by the cell (default 0.99).
     * etaDischarge — fraction of stored charge that can be extracted (default 1.00).
     */
    void setEfficiency(float etaCharge, float etaDischarge);

    /**
     * Set self-discharge rate in mAh/hour.
     * Default 0.05 mAh/h (≈ 1 %/month on a 3 Ah cell).
     * Set 0.0 to disable.
     */
    void setSelfDischargeRate(float mahPerHour);

    // Initialisation
    bool begin();

    // Main loop — call in loop(), returns true on fresh measurement
    bool update();

    // Output
    void print(Stream& port);
    void printCsvHeader(Stream& port);
    void printDebugStatus(Stream& port);

    // Accessors
    int      getSoc()            const;
    int      getSoh()            const;
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
#if LTC2944_BMS_ENABLE_DEBUG
    BmsDebugState   getDebugState()  const;
#endif

    // Disconnect detection
    bool isBatteryDisconnected() const;

    // Calibration
    void     startCalibration();
    void     stopCalibration();
    CalPhase getCalPhase()  const;
    bool     isCalibrated() const;
    void     resetCalibration();

    // Learned model
    float getLearnedResistance()  const;
    float getLearningConfidence() const;
    void  resetForNewBattery();

    // Utility
    const char* getProfileName()     const;
    bool        isProfilePlausible() const;
    bool        reinitChip();

private:

    // Config
    BatteryChem _chem;
    uint8_t     _classCode;
    float       _shuntOhm;
    float       _overrideFullV;
    float       _overrideEmptyV;
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
    float       _etaCharge;
    float       _etaDischarge;
    float       _selfDischargeRate;

    // Live state
    BmsMeasurement _meas;
#if LTC2944_BMS_ENABLE_DEBUG
    BmsDebugState _debug;
#endif
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

    // Efficiency-corrected coulomb accumulator
    float       _etaAcrAccum;   ///< Fractional ACR position after eta correction
    bool        _etaAccumValid;

    // Full-charge confirmation
    uint8_t     _calFullConfirmCount;

    // Hard-cutoff charger detection
    bool        _prevCharging;
    float       _calChargerOnCurrentA;
    float       _calPeakChargeV;
    bool        _calNearFullSeen;
    uint32_t    _calNearFullSeenMs;
    bool        _calCutoffPending;
    uint32_t    _calCutoffMs;
    uint8_t     _calNoChargeCount;

    // ACR rollover tracking
    uint16_t    _lastRawACR;
    bool        _acrRollover;
    bool        _haveObservedAcr;
    uint8_t     _acrBootTicks;
    bool        _bootSeedPending;

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

    // SOH
    int      _soh;
    uint32_t _lastTimestampMs; ///< millis() when last EEPROM timestamp was written

    // I2C
    bool     _writeReg8(uint8_t reg, uint8_t val);
    bool     _writeReg16(uint8_t reg, uint16_t val);
    bool     _readReg8(uint8_t reg, uint8_t& val);
    bool     _writeACR(uint16_t val);
    bool     _readReg16(uint8_t reg, uint16_t& val);

    // Conversions
    float    _rawToVoltage(uint16_t raw) const;
    float    _rawToCurrent(uint16_t raw) const;
    float    _rawToCurrentNoOffset(uint16_t raw) const;
    float    _rawToTemperature(uint16_t raw) const;
    float    _rawAcrToMah(uint16_t raw) const;

    float    _effectiveVFull()  const;
    float    _effectiveVEmpty() const;
    uint8_t  _computePrescaler() const;
    bool     _writePrescaler(uint8_t m);

    // Profile
    void     _resolveProfile();
    bool     _isVoltagePlausible(float v) const;
    bool     _checkAcrRollover(uint16_t rawACR);

    // SOC estimation
    int      _estimateSoc(float vbat, float currentA, uint16_t rawACR, bool charging);
    int      _estimateVoltageSocOnly(float vbat, float currentA, uint16_t rawACR, bool charging);
    int      _socFromOcvCurve(float ocvCell) const;
    int      _socFromVoltageCurve(float vbat, float currentA, uint16_t rawACR, bool charging,
                                   float chargeCompV, int lowerBandSoc, int upperBandSoc);
    int      _estimateCoulombSoc() const;  // uses _etaAcrAccum
    int      _adaptiveBlend(int coulombSoc, int voltageSoc, float currentA) const;
    uint16_t _makeAlarms(float v, float i, float t, bool sensorOk) const;

    // Eta accumulator
    void     _seedEtaAccum(uint16_t rawACR);
    void     _updateEtaAccum(uint16_t rawACR, bool charging);
    bool     _restoreBootSeedFromRawVoltage(uint16_t rawVolt, uint16_t* seededACR = nullptr);

    // Temperature derating
    float    _tempDerateFactor(float tempC) const;

    // Self-discharge
    void     _applyBootSelfDischarge();
    void     _saveTimestamp();
    uint32_t _loadTimestamp() const;

    // Calibration state machine
    void     _updateCalibration(uint16_t rawCurr, uint16_t rawACR,
                                 float vbat, float tempC, float currentA);
    void     _enterCalPhase(CalPhase phase);
    void     _setLoad(bool en);

    // SOH
    void     _updateSoh(float measuredCmaxMah);
    void     _loadSoh();
    void     _saveSoh();

    // Disconnect
    bool     _checkDisconnectOnReadFailure();
    bool     _checkDisconnectOnMeasurement(float vbat, float currentA);
    void     _writeDisconnectFlag();
    void     _clearDisconnectFlag();
    bool     _readDisconnectFlag() const;

    // EEPROM
    void     _loadOffset();
    void     _saveOffset(float offset_mA);
    void     _loadCalibration();
    void     _saveCalEmpty(float v, uint16_t raw);
    void     _saveCalFull(float v, uint16_t raw);

    // Learned model
    void     _resetLearnedModel(bool newBattery);
    bool     _loadLearnedModel();
    bool     _saveLearnedModel(bool force);
    void     _updateResistanceLearning(float v, float i, uint32_t now);
    void     _updateRelaxedSignature(float v, float i, uint16_t rawACR, uint32_t now);
    float    _effectiveResistance() const;

    // CRC
    uint8_t  _crc8(const uint8_t* data, size_t len) const;
    uint8_t  _recordCrc(const LearnedRecord& r) const;
    bool     _isRecordValid(const LearnedRecord& r) const;

    // Helpers
    static int   _clamp(int v, int lo, int hi);
    static float _clampf(float v, float lo, float hi);
    static float _absf(float v);
    bool         _currentNearZero(float i) const;
    void         _copyProfileName();
#if LTC2944_BMS_ENABLE_DEBUG
    const __FlashStringHelper* _socSourceText(uint8_t src) const;
#endif
};
