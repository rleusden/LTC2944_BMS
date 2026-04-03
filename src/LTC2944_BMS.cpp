#include "LTC2944_BMS.h"

// ──────────────────────────────────────────────────────────────────────────────
// LTC2944 I2C address & registers
// ──────────────────────────────────────────────────────────────────────────────
static const uint8_t LTC2944_ADDR        = 0x64;
static const uint8_t REG_CONTROL         = 0x01;
static const uint8_t REG_ACC_CHARGE_MSB  = 0x02;
static const uint8_t REG_VOLTAGE_MSB     = 0x08;
static const uint8_t REG_CURRENT_MSB     = 0x0E;
static const uint8_t REG_TEMPERATURE_MSB = 0x14;

// ──────────────────────────────────────────────────────────────────────────────
// EEPROM addresses
// ──────────────────────────────────────────────────────────────────────────────
static const int EE_MAGIC           = 0x00;
static const int EE_OFFSET          = 0x04;
static const int EE_DISCONNECT      = 0x08;
static const int EE_CAL_MAGIC       = 0x20;
static const int EE_CAL_EMPTY_V     = 0x24;
static const int EE_CAL_FULL_V      = 0x28;
static const int EE_CAL_EMPTY_RAW   = 0x2C;
static const int EE_CAL_EMPTY_MAGIC = 0x2E;
static const int EE_CAL_FULL_RAW    = 0x30;
static const int EE_CAL_FULL_MAGIC  = 0x32;
static const int EE_LEARNED_SLOT_A  = 0xA0;
static const int EE_LEARNED_SLOT_B  = EE_LEARNED_SLOT_A + (int)sizeof(LearnedRecord);

static const uint32_t EEPROM_MAGIC    = 0x42C0FFEEul;
static const uint32_t CAL_MAGIC       = 0xCA1BCA1Bul;
static const uint16_t EMPTY_ACR_MAGIC = 0xA44Cu;
static const uint16_t FULL_ACR_MAGIC  = 0xF44Cu;
static const uint8_t  DISCONNECT_MARKER = 0xD1;
static const uint16_t LEARNED_MAGIC   = 0x4C42u;
static const uint8_t  LEARNED_VERSION = 1;
static const uint32_t LEARNED_SAVE_INTERVAL_MS = 30UL * 60UL * 1000UL;

// ──────────────────────────────────────────────────────────────────────────────
// Calibration constants
// ──────────────────────────────────────────────────────────────────────────────
static const uint16_t ZERO_SAMPLES          = 32;
static const uint32_t CAL_SETTLE_MS         = 5000UL;
static const float    CAL_FULL_VOLT_PCT     = 0.98f;
// Require this many consecutive readings below the taper threshold
// before declaring full charge — prevents false triggers from balance current
static const uint8_t  CAL_FULL_CONFIRM_SAMPLES = 5;

// ──────────────────────────────────────────────────────────────────────────────
// Battery profiles in PROGMEM
// ──────────────────────────────────────────────────────────────────────────────
static const char N_LIION_1S[]  PROGMEM = "LIION_1S";
static const char N_LIION_2S[]  PROGMEM = "LIION_2S";
static const char N_LIION_3S[]  PROGMEM = "LIION_3S";
static const char N_LIION_4S[]  PROGMEM = "LIION_4S";
static const char N_LIION_6S[]  PROGMEM = "LIION_6S";
static const char N_LIION_8S[]  PROGMEM = "LIION_8S";
static const char N_LIION_12S[] PROGMEM = "LIION_12S";
static const char N_LIION_14S[] PROGMEM = "LIION_14S";
static const char N_LFP_1S[]    PROGMEM = "LFP_1S";
static const char N_LFP_2S[]    PROGMEM = "LFP_2S";
static const char N_LFP_3S[]    PROGMEM = "LFP_3S";
static const char N_LFP_4S[]    PROGMEM = "LFP_4S";
static const char N_LFP_6S[]    PROGMEM = "LFP_6S";
static const char N_LFP_8S[]    PROGMEM = "LFP_8S";
static const char N_LFP_12S[]   PROGMEM = "LFP_12S";
static const char N_LFP_14S[]   PROGMEM = "LFP_14S";
static const char N_AGM_12V[]   PROGMEM = "AGM_12V";
static const char N_AGM_24V[]   PROGMEM = "AGM_24V";
static const char N_AGM_48V[]   PROGMEM = "AGM_48V";
static const char N_GEL_12V[]   PROGMEM = "GEL_12V";
static const char N_GEL_24V[]   PROGMEM = "GEL_24V";
static const char N_GEL_48V[]   PROGMEM = "GEL_48V";
static const char N_INVALID[]   PROGMEM = "INVALID";

struct RawProfile {
    bool    valid;
    uint8_t chem;
    uint8_t classCode;
    const char* name;
    uint8_t seriesCount;
    bool    leadBased;
    float   vEmpty, vNominal, vFull, vAbsLow, vAbsHigh;
};

static const RawProfile kProfiles[] PROGMEM = {
    // Li-ion: classCode = series cell count (1-14)
    {true,CHEM_LIION,  1,N_LIION_1S,   1,false, 3.0f, 3.7f, 4.2f, 2.5f, 4.3f},
    {true,CHEM_LIION,  2,N_LIION_2S,   2,false, 6.0f, 7.4f, 8.4f, 5.0f, 8.6f},
    {true,CHEM_LIION,  3,N_LIION_3S,   3,false, 9.0f,11.1f,12.6f, 7.5f,12.9f},
    {true,CHEM_LIION,  4,N_LIION_4S,   4,false,12.0f,14.8f,16.8f,10.0f,17.2f},
    {true,CHEM_LIION,  6,N_LIION_6S,   6,false,18.0f,22.2f,25.2f,15.0f,25.8f},
    {true,CHEM_LIION,  8,N_LIION_8S,   8,false,24.0f,29.6f,33.6f,20.0f,34.4f},
    {true,CHEM_LIION, 12,N_LIION_12S, 12,false,36.0f,44.4f,50.4f,30.0f,51.6f},
    {true,CHEM_LIION, 14,N_LIION_14S, 14,false,42.0f,51.8f,58.8f,35.0f,60.2f},
    // LiFePO4: classCode = series cell count (1-14)
    {true,CHEM_LIFEPO4, 1,N_LFP_1S,   1,false, 2.8f, 3.2f, 3.6f, 2.5f, 3.8f},
    {true,CHEM_LIFEPO4, 2,N_LFP_2S,   2,false, 5.6f, 6.4f, 7.2f, 5.0f, 7.6f},
    {true,CHEM_LIFEPO4, 3,N_LFP_3S,   3,false, 8.4f, 9.6f,10.8f, 7.5f,11.4f},
    {true,CHEM_LIFEPO4, 4,N_LFP_4S,   4,false,11.2f,12.8f,14.4f,10.0f,15.2f},
    {true,CHEM_LIFEPO4, 6,N_LFP_6S,   6,false,16.8f,19.2f,21.6f,15.0f,22.8f},
    {true,CHEM_LIFEPO4, 8,N_LFP_8S,   8,false,22.4f,25.6f,28.8f,20.0f,30.4f},
    {true,CHEM_LIFEPO4,12,N_LFP_12S, 12,false,33.6f,38.4f,43.2f,30.0f,45.6f},
    {true,CHEM_LIFEPO4,14,N_LFP_14S, 14,false,39.2f,44.8f,50.4f,35.0f,53.2f},
    // AGM: classCode = pack voltage (12, 24, 48)
    {true,CHEM_AGM, 12,N_AGM_12V,  6,true, 10.5f,12.6f,14.4f,10.0f,14.8f},
    {true,CHEM_AGM, 24,N_AGM_24V, 12,true, 21.0f,25.2f,28.8f,20.0f,29.6f},
    {true,CHEM_AGM, 48,N_AGM_48V, 24,true, 42.0f,50.4f,57.6f,40.0f,59.2f},
    // GEL: classCode = pack voltage (12, 24, 48)
    {true,CHEM_GEL, 12,N_GEL_12V,  6,true, 10.5f,12.6f,14.2f,10.0f,14.8f},
    {true,CHEM_GEL, 24,N_GEL_24V, 12,true, 21.0f,25.2f,28.4f,20.0f,29.6f},
    {true,CHEM_GEL, 48,N_GEL_48V, 24,true, 42.0f,50.4f,56.8f,40.0f,59.2f},
};
static const uint8_t kProfileCount = sizeof(kProfiles) / sizeof(RawProfile);

// ──────────────────────────────────────────────────────────────────────────────
// Constructor
// ──────────────────────────────────────────────────────────────────────────────

LTC2944_BMS::LTC2944_BMS()
    : _chem(CHEM_LIION)
    , _classCode(0)
    , _shuntOhm(0.001f)
    , _overrideFullV(0.0f)
    , _overrideEmptyV(0.0f)
    , _pinLoadEn(BMS_DEFAULT_PIN_LOAD_EN)
    , _pinStatusLed(BMS_DEFAULT_PIN_STATUS_LED)
    , _currentDeadbandA(0.030f)
    , _capacityMah(0)
    , _prescaler(1)
    , _outputMode(OUTPUT_SERIAL)
    , _calTimeoutMs(240UL * 60UL * 1000UL)   // 4 hours default
    , _calFullCurrentA(0.10f)                  // 100 mA default
    , _maxDischargeA(5.5f)
    , _maxChargeA(3.0f)
    , _meas{}
    , _disconnected(false)
    , _profileMismatch(false)
    , _charging(false)
    , _calibrationMode(false)
    , _calPhase(CAL_NONE)
    , _calPhaseStartMs(0)
    , _calRunStartMs(0)
    , _zeroSamplesTaken(0)
    , _zeroAccumRawDelta(0)
    , _lastSendMs(0)
    , _currentOffset_mA(0.0f)
    , _disconnectCandidateCount(0)
    , _i2cFailCount(0)
    , _calFullConfirmCount(0)
    , _prevCharging(false)
    , _calChargerOnCurrentA(0.030f)
    , _lastRawACR(0)
    , _acrRollover(false)
    , _acrBootTicks(3)
    , _profile{}
    , _profileNameBuf{}
    , _cal{}
    , _model{}
    , _resLearn{}
    , _lastLearnedSaveMs(0)
    , _lastRelaxedSeenMs(0)
{}

// ──────────────────────────────────────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::setProfile(BatteryChem chem, uint8_t classCode) { _chem = chem; _classCode = classCode; }
void LTC2944_BMS::setCapacity(uint16_t mah)        { _capacityMah = mah; }
void LTC2944_BMS::setFullVoltage(float v)          { _overrideFullV = v; }
void LTC2944_BMS::setEmptyVoltage(float v)         { _overrideEmptyV = v; }
void LTC2944_BMS::setShuntResistor(float ohms)     { _shuntOhm = ohms; }
void LTC2944_BMS::setCalibrationTimeout(uint32_t m){ _calTimeoutMs = m * 60UL * 1000UL; }
void LTC2944_BMS::setCalibrationFullCurrent(float a){ _calFullCurrentA = a; }
void LTC2944_BMS::setCalibrationChargerOnCurrent(float a){ _calChargerOnCurrentA = a; }
void LTC2944_BMS::setMaxDischargeCurrent(float a)  { _maxDischargeA = a; }
void LTC2944_BMS::setMaxChargeCurrent(float a)     { _maxChargeA = a; }
void LTC2944_BMS::setOutputMode(OutputMode m)      { _outputMode = m; }
void LTC2944_BMS::setPinLoadEnable(uint8_t p)      { _pinLoadEn = p; }
void LTC2944_BMS::setPinStatusLed(uint8_t p)       { _pinStatusLed = p; }
void LTC2944_BMS::setCurrentDeadband(float a)      { _currentDeadbandA = a; }

// ──────────────────────────────────────────────────────────────────────────────
// Effective voltage helpers
// ──────────────────────────────────────────────────────────────────────────────

float LTC2944_BMS::_effectiveVFull() const {
    if (_overrideFullV > 0.0f) return _overrideFullV;
    // isfinite(0.0f) is true, so guard against zero explicitly.
    // _cal.vFull is 0.0f until the full anchor is saved at end of CAL_CHARGE,
    // so returning it early would make nearFull checks fire at any voltage.
    if (_cal.valid && _cal.vFull > 0.0f) return _cal.vFull;
    return _profile.valid ? _profile.vFull : 0.0f;
}

float LTC2944_BMS::_effectiveVEmpty() const {
    if (_overrideEmptyV > 0.0f) return _overrideEmptyV;
    if (_cal.valid && isfinite(_cal.vEmpty)) return _cal.vEmpty;
    return _profile.valid ? _profile.vEmpty : 0.0f;
}

// ──────────────────────────────────────────────────────────────────────────────
// Prescaler
// ──────────────────────────────────────────────────────────────────────────────

uint8_t LTC2944_BMS::_computePrescaler() const {
    if (_capacityMah == 0) return 1;
    float shuntMilliOhm = _shuntOhm * 1000.0f;
    float mIdeal = ((float)_capacityMah * shuntMilliOhm) / 40.96f;
    uint8_t m = 1;
    while ((float)m < mIdeal && m < 128) m <<= 1;
    return m;
}

bool LTC2944_BMS::_writePrescaler(uint8_t m) {
    uint8_t mBits = 0, tmp = m;
    while (tmp > 1) { tmp >>= 1; mBits++; }
    uint8_t ctrl = (uint8_t)(0xC4 | (mBits << 3));
    return _writeReg8(REG_CONTROL, ctrl);
}

float LTC2944_BMS::_rawAcrToMah(uint16_t raw) const {
    float fullScaleMah = (0.340f / _shuntOhm) * ((float)_prescaler / 128.0f) * 1000.0f;
    return ((float)raw / 65535.0f) * fullScaleMah;
}

// ──────────────────────────────────────────────────────────────────────────────
// ACR rollover detection
// Detects when the 16-bit ACR counter wraps during a long discharge.
// A rollover is confirmed when the new value is much lower than the previous.
// ──────────────────────────────────────────────────────────────────────────────

bool LTC2944_BMS::_checkAcrRollover(uint16_t rawACR) {
    bool rollover = false;
    if (_lastRawACR > 0x8000 && rawACR < 0x1000) {
        // Large downward jump — counter wrapped from near-max to near-zero
        rollover = true;
        _acrRollover = true;
    }
    _lastRawACR = rawACR;
    return rollover;
}

// ──────────────────────────────────────────────────────────────────────────────
// Initialisation
// ──────────────────────────────────────────────────────────────────────────────

bool LTC2944_BMS::begin() {
    pinMode(_pinLoadEn,    OUTPUT);
    pinMode(_pinStatusLed, OUTPUT);
    digitalWrite(_pinLoadEn,    LOW);
    digitalWrite(_pinStatusLed, LOW);

    Wire.begin();
    _loadOffset();
    _loadCalibration();
    _resolveProfile();
    _loadLearnedModel();
    _prescaler = _computePrescaler();

    if (_outputMode == OUTPUT_CSV) printCsvHeader(Serial);

    bool ok = reinitChip();

    // Seed the ACR register to a position matching the voltage-estimated SOC.
    // After a flash or power cycle the LTC2944 resets ACR to 0x7FFF (mid-scale)
    // which has no relation to actual charge state. We read the current voltage,
    // estimate SOC from the voltage curve, then write ACR to the proportional
    // position within the calibrated empty..full window.
    // This gives the coulomb counter a meaningful starting point so the blend
    // is immediately sensible rather than jumping once the settle window expires.
    // If calibration is not yet valid we leave ACR at chip default and rely on
    // voltage-only SOC until calibration completes.
    if (_cal.valid && _cal.hasEmptyRaw && _cal.hasFullRaw) {
        uint16_t rawVoltBoot = 0;
        if (_readReg16(REG_VOLTAGE_MSB, rawVoltBoot)) {
            float vBoot = _rawToVoltage(rawVoltBoot);
            // Crude linear voltage SOC (no IR comp, no current yet)
            float vEmpty = _effectiveVEmpty();
            float vFull  = _effectiveVFull();
            float socFrac = 0.0f;
            if (vFull > vEmpty + 0.1f)
                socFrac = _clampf((vBoot - vEmpty) / (vFull - vEmpty), 0.0f, 1.0f);
            long span    = (long)_cal.fullChargeRaw - (long)_cal.emptyChargeRaw;
            uint16_t acrSeed = (uint16_t)(_cal.emptyChargeRaw + (long)(socFrac * (float)span));
            _writeACR(acrSeed);
            _lastRawACR = acrSeed;
        }
    }
    // Use voltage-only SOC for the first 3 update() ticks while the ACR
    // register settles and the current measurement stabilises.
    _acrBootTicks = 3;

    return ok;
}

bool LTC2944_BMS::reinitChip() {
    if (!_writePrescaler(_prescaler)) return false;
    delay(50);
    uint8_t ctrl = 0;
    return _readReg8(REG_CONTROL, ctrl);
}

// ──────────────────────────────────────────────────────────────────────────────
// Main loop
// ──────────────────────────────────────────────────────────────────────────────

bool LTC2944_BMS::update() {
    if (_disconnected) return false;

    uint32_t now = millis();
    if (now - _lastSendMs < 1000UL) return false;
    _lastSendMs = now;

    uint16_t rawACR = 0, rawVolt = 0, rawCurr = 0, rawTemp = 0;
    bool ok = _readReg16(REG_ACC_CHARGE_MSB,  rawACR)
           && _readReg16(REG_VOLTAGE_MSB,     rawVolt)
           && _readReg16(REG_CURRENT_MSB,     rawCurr)
           && _readReg16(REG_TEMPERATURE_MSB, rawTemp);

    if (!ok) {
        _disconnected = _checkDisconnectOnReadFailure();
        _meas.alarms  = ALARM_SENSOR_FAIL;
        return false;
    }

    _i2cFailCount = 0;

    // Check for ACR rollover before using the value
    if (_checkAcrRollover(rawACR)) {
        // Reset rollover flag after one cycle so it appears in alarms once
        _acrRollover = true;
    }

    float vbat    = _rawToVoltage(rawVolt);
    float current = _rawToCurrent(rawCurr);
    float tempC   = _rawToTemperature(rawTemp);

    _profileMismatch = !_isVoltagePlausible(vbat);

    _prevCharging = _charging;
    if (!_charging && current >  0.03f) _charging = true;
    else if (_charging && current < 0.01f) _charging = false;

    _updateCalibration(rawCurr, rawACR, vbat, tempC, current);

    if (_checkDisconnectOnMeasurement(vbat, current)) {
        _disconnected = true;
        return false;
    }

    int soc = _estimateSoc(vbat, current, rawACR, _charging);

    uint16_t remainingMah = 0;
    if (_capacityMah > 0)
        remainingMah = (uint16_t)(((long)soc * _capacityMah) / 100L);

    int16_t runtimeMinutes = -1;
    if (_capacityMah > 0 && !_charging && current < -0.030f) {
        float dischargeA = _absf(current);
        runtimeMinutes = (int16_t)(((float)remainingMah / (dischargeA * 1000.0f)) * 60.0f);
        if (runtimeMinutes < 0) runtimeMinutes = 0;
    }

    _updateResistanceLearning(vbat, current, now);
    _updateRelaxedSignature(vbat, current, rawACR, now);
    _saveLearnedModel(false);

    _meas.soc            = soc;
    _meas.voltage        = vbat;
    _meas.current        = current;
    _meas.temperature    = tempC;
    _meas.remainingMah   = remainingMah;
    _meas.runtimeMinutes = runtimeMinutes;
    _meas.rawACR         = rawACR;
    _meas.rawCurrent     = rawCurr;
    _meas.charging       = _charging;
    _meas.alarms         = _makeAlarms(vbat, current, tempC, true);

    // Clear rollover flag after it has been included in one alarm report
    _acrRollover = false;

    bool ledOn = _calibrationMode && _calPhase != CAL_DONE && _calPhase != CAL_ABORT;
    digitalWrite(_pinStatusLed, ledOn ? HIGH : LOW);

    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Output
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::printDebug(Stream& port) {
    port.println(F("\n=== LTC2944_BMS DEBUG ==="));

    // -- EEPROM: current offset
    uint32_t offMagic = 0; EEPROM.get(EE_MAGIC, offMagic);
    port.print(F("EEPROM offset magic : 0x")); port.println(offMagic, HEX);
    if (offMagic == EEPROM_MAGIC) {
        float off = 0.0f; EEPROM.get(EE_OFFSET, off);
        port.print(F("Current offset      : ")); port.print(off, 2); port.println(F(" mA"));
    } else {
        port.println(F("Current offset      : (not saved)"));
    }

    // -- EEPROM: calibration
    uint32_t calMagic = 0; EEPROM.get(EE_CAL_MAGIC, calMagic);
    port.print(F("EEPROM cal magic    : 0x")); port.println(calMagic, HEX);
    if (calMagic == CAL_MAGIC) {
        float vE = 0.0f, vF = 0.0f;
        EEPROM.get(EE_CAL_EMPTY_V, vE);
        EEPROM.get(EE_CAL_FULL_V,  vF);
        port.print(F("Cal vEmpty          : ")); port.println(vE, 4);
        port.print(F("Cal vFull           : ")); port.println(vF, 4);

        uint16_t emMagic = 0, fuMagic = 0;
        EEPROM.get(EE_CAL_EMPTY_MAGIC, emMagic);
        EEPROM.get(EE_CAL_FULL_MAGIC,  fuMagic);
        port.print(F("Empty ACR magic     : 0x")); port.println(emMagic, HEX);
        port.print(F("Full  ACR magic     : 0x")); port.println(fuMagic, HEX);

        if (emMagic == EMPTY_ACR_MAGIC) {
            uint16_t eRaw = 0; EEPROM.get(EE_CAL_EMPTY_RAW, eRaw);
            port.print(F("Cal emptyChargeRaw  : 0x")); port.println(eRaw, HEX);
        } else {
            port.println(F("Cal emptyChargeRaw  : (magic bad)"));
        }
        if (fuMagic == FULL_ACR_MAGIC) {
            uint16_t fRaw = 0; EEPROM.get(EE_CAL_FULL_RAW, fRaw);
            port.print(F("Cal fullChargeRaw   : 0x")); port.println(fRaw, HEX);
        } else {
            port.println(F("Cal fullChargeRaw   : (magic bad)"));
        }
    } else {
        port.println(F("Calibration         : (not saved)"));
    }

    // -- RAM calibration struct
    port.print(F("_cal.valid          : ")); port.println(_cal.valid);
    port.print(F("_cal.hasEmptyRaw    : ")); port.println(_cal.hasEmptyRaw);
    port.print(F("_cal.hasFullRaw     : ")); port.println(_cal.hasFullRaw);
    port.print(F("_cal.vEmpty         : ")); port.println(_cal.vEmpty, 4);
    port.print(F("_cal.vFull          : ")); port.println(_cal.vFull,  4);
    port.print(F("_cal.emptyChargeRaw : 0x")); port.println(_cal.emptyChargeRaw, HEX);
    port.print(F("_cal.fullChargeRaw  : 0x")); port.println(_cal.fullChargeRaw,  HEX);

    // -- Live chip registers
    uint16_t liveACR = 0, liveV = 0, liveI = 0;
    bool acrOk = _readReg16(REG_ACC_CHARGE_MSB, liveACR);
    bool vOk   = _readReg16(REG_VOLTAGE_MSB,    liveV);
    bool iOk   = _readReg16(REG_CURRENT_MSB,    liveI);
    port.print(F("Live ACR register   : "));
    if (acrOk) { port.print(F("0x")); port.println(liveACR, HEX); }
    else          port.println(F("(read failed)"));
    port.print(F("Live voltage raw    : "));
    if (vOk)   { port.print(F("0x")); port.print(liveV, HEX);
                 port.print(F("  (")); port.print(_rawToVoltage(liveV), 3); port.println(F(" V)")); }
    else          port.println(F("(read failed)"));
    port.print(F("Live current raw    : "));
    if (iOk)   { port.print(F("0x")); port.println(liveI, HEX); }
    else          port.println(F("(read failed)"));

    // -- ACR seed result
    port.print(F("_lastRawACR (seed)  : 0x")); port.println(_lastRawACR, HEX);
    port.print(F("_acrBootTicks left  : ")); port.println(_acrBootTicks);

    // Raw EEPROM byte dump around cal region
    port.println(F("--- Raw EEPROM 0x20..0x33 ---"));
    for (int addr = 0x20; addr <= 0x33; addr++) {
        uint8_t b = EEPROM.read(addr);
        port.print(F("0x")); 
        if (addr < 0x10) port.print(F("0"));
        port.print(addr, HEX);
        port.print(F(": 0x"));
        if (b < 0x10) port.print(F("0"));
        port.println(b, HEX);
    }
    port.println(F("=========================\n"));
}

void LTC2944_BMS::printCsvHeader(Stream& port) {
    port.println(F("soc_%,voltage_V,current_mA,temp_C,charging,remaining_mAh,runtime_min,alarms"));
}

void LTC2944_BMS::print(Stream& port) {
    long current_mA = lround(_meas.current * 1000.0f);
    int  runtime    = (_meas.runtimeMinutes >= 0) ? (int)_meas.runtimeMinutes : -1;

    switch (_outputMode) {

        case OUTPUT_SERIAL:
        default:
            port.print(F("SOC="));   port.print(_meas.soc); port.print(F("%"));
            port.print(F("  V="));   port.print(_meas.voltage, 3); port.print(F("V"));
            port.print(F("  I="));   port.print(current_mA); port.print(F("mA"));
            port.print(F("  T="));   port.print(_meas.temperature, 1); port.print(F("C"));
            port.print(F("  CHG=")); port.print(_meas.charging ? F("YES") : F("NO"));
            if (_capacityMah > 0) {
                port.print(F("  REM=")); port.print(_meas.remainingMah); port.print(F("mAh"));
                if (runtime >= 0) {
                    port.print(F("  RUN="));
                    if (runtime >= 60) {
                        port.print(runtime / 60); port.print(F("h"));
                        port.print(runtime % 60); port.print(F("m"));
                    } else {
                        port.print(runtime); port.print(F("m"));
                    }
                }
            }
            if (_meas.alarms) {
                port.print(F("  ALARM=0x")); port.print(_meas.alarms, HEX);
                if (_meas.alarms & ALARM_UNDER_VOLTAGE)    port.print(F(" [UV]"));
                if (_meas.alarms & ALARM_OVER_VOLTAGE)     port.print(F(" [OV]"));
                if (_meas.alarms & ALARM_TEMPERATURE)      port.print(F(" [TEMP]"));
                if (_meas.alarms & ALARM_OVER_CURRENT)     port.print(F(" [OC]"));
                if (_meas.alarms & ALARM_CHARGE_CURRENT)   port.print(F(" [CC]"));
                if (_meas.alarms & ALARM_PROFILE_MISMATCH) port.print(F(" [MISMATCH]"));
                if (_meas.alarms & ALARM_ACR_ROLLOVER)     port.print(F(" [ROLLOVER]"));
            }
            port.println();
            break;

        case OUTPUT_CSV:
            port.print(_meas.soc);              port.print(',');
            port.print(_meas.voltage, 3);       port.print(',');
            port.print(current_mA);             port.print(',');
            port.print(_meas.temperature, 1);   port.print(',');
            port.print(_meas.charging ? 1 : 0); port.print(',');
            port.print(_meas.remainingMah);     port.print(',');
            port.print(runtime);                port.print(',');
            port.println(_meas.alarms);
            break;

        case OUTPUT_ESPHOME:
            port.print(_meas.soc);              port.print(',');
            port.print(_meas.voltage, 3);       port.print(',');
            port.print(current_mA);             port.print(',');
            port.print(_meas.temperature, 1);   port.print(',');
            port.print(_meas.charging ? 1 : 0); port.print(',');
            port.print(_meas.remainingMah);     port.print(',');
            port.print(runtime);                port.print(',');
            port.print(_meas.alarms);
            port.print('\r'); port.println();
            break;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Accessors
// ──────────────────────────────────────────────────────────────────────────────

int      LTC2944_BMS::getSoc()            const { return _meas.soc; }
float    LTC2944_BMS::getVoltage()        const { return _meas.voltage; }
float    LTC2944_BMS::getCurrent()        const { return _meas.current; }
float    LTC2944_BMS::getTemperature()    const { return _meas.temperature; }
uint16_t LTC2944_BMS::getAlarms()         const { return _meas.alarms; }
bool     LTC2944_BMS::isCharging()        const { return _meas.charging; }
uint16_t LTC2944_BMS::getRemainingMah()   const { return _meas.remainingMah; }
int16_t  LTC2944_BMS::getRuntimeMinutes() const { return _meas.runtimeMinutes; }
uint16_t LTC2944_BMS::getCapacityMah()    const { return _capacityMah; }
uint8_t  LTC2944_BMS::getPrescaler()      const { return _prescaler; }

BmsMeasurement LTC2944_BMS::getMeasurement()       const { return _meas; }
bool           LTC2944_BMS::isBatteryDisconnected() const { return _disconnected; }

// ──────────────────────────────────────────────────────────────────────────────
// Calibration
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::startCalibration() {
    _calibrationMode = true;
    _calFullConfirmCount = 0;
    // Clear any previous calibration anchors so fullNotYetSaved is true
    // throughout the new run. Stale anchors from a previous calibration
    // would block PATH B and PATH A from saving new values.
    resetCalibration();
    // Reset the current offset to zero before the zero phase begins.
    // Any previously stored offset would corrupt the charger-on safety check
    // and the zero-sample accumulation, since currentA is offset-corrected.
    _currentOffset_mA = 0.0f;
    _saveOffset(0.0f);
    _enterCalPhase(CAL_ZERO);
}
void LTC2944_BMS::stopCalibration()  { _calibrationMode = false; _enterCalPhase(CAL_NONE); }
CalPhase LTC2944_BMS::getCalPhase()  const { return _calPhase; }

bool LTC2944_BMS::isCalibrated() const {
    return _cal.valid && _cal.hasEmptyRaw && _cal.hasFullRaw
        && (_cal.fullChargeRaw != _cal.emptyChargeRaw);
}

void LTC2944_BMS::resetCalibration() {
    _cal = {};
    uint32_t zero = 0;
    EEPROM.put(EE_CAL_MAGIC, zero);
}

// ──────────────────────────────────────────────────────────────────────────────
// Learned model
// ──────────────────────────────────────────────────────────────────────────────

float LTC2944_BMS::getLearnedResistance()  const { return _effectiveResistance(); }
float LTC2944_BMS::getLearningConfidence() const { return _model.confidence; }

void LTC2944_BMS::resetForNewBattery() {
    _resetLearnedModel(true);
    _saveLearnedModel(true);
    resetCalibration();
}

// ──────────────────────────────────────────────────────────────────────────────
// Utility
// ──────────────────────────────────────────────────────────────────────────────

const char* LTC2944_BMS::getProfileName()    const { return _profileNameBuf; }
bool        LTC2944_BMS::isProfilePlausible() const { return !_profileMismatch; }

// ──────────────────────────────────────────────────────────────────────────────
// I2C helpers
// ──────────────────────────────────────────────────────────────────────────────

// Shutdown-safe ACR write.
// The LTC2944 datasheet requires B[0]=1 (shutdown) before writing the ACR
// register (C/D) to avoid a race with the internal delta-sigma integrator.
// We read the current control register, set the shutdown bit, write ACR,
// then restore the original control value.
bool LTC2944_BMS::_writeACR(uint16_t val) {
    uint8_t ctrl = 0;
    if (!_readReg8(REG_CONTROL, ctrl)) return false;
    if (!_writeReg8(REG_CONTROL, ctrl | 0x01)) return false;  // assert shutdown
    delay(1);
    bool ok = _writeReg16(REG_ACC_CHARGE_MSB, val);
    _writeReg8(REG_CONTROL, ctrl);                             // restore (clears shutdown)
    return ok;
}

bool LTC2944_BMS::_writeReg8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LTC2944_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool LTC2944_BMS::_writeReg16(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(LTC2944_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    return Wire.endTransmission() == 0;
}

bool LTC2944_BMS::_readReg8(uint8_t reg, uint8_t& val) {
    Wire.beginTransmission(LTC2944_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)LTC2944_ADDR, (uint8_t)1) != 1) return false;
    val = Wire.read();
    return true;
}

bool LTC2944_BMS::_readReg16(uint8_t reg, uint16_t& val) {
    Wire.beginTransmission(LTC2944_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)LTC2944_ADDR, (uint8_t)2) != 2) return false;
    uint8_t msb = Wire.read(), lsb = Wire.read();
    val = ((uint16_t)msb << 8) | lsb;
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Conversions
// ──────────────────────────────────────────────────────────────────────────────

float LTC2944_BMS::_rawToVoltage(uint16_t raw) const { return 70.8f * ((float)raw / 65535.0f); }

float LTC2944_BMS::_rawToCurrentNoOffset(uint16_t raw) const {
    return (0.064f / _shuntOhm) * (((int32_t)raw - 32767) / 32767.0f);
}

float LTC2944_BMS::_rawToCurrent(uint16_t raw) const {
    float a = _rawToCurrentNoOffset(raw) - (_currentOffset_mA / 1000.0f);
    if (_absf(a) < _currentDeadbandA) a = 0.0f;
    return a;
}

float LTC2944_BMS::_rawToTemperature(uint16_t raw) const {
    return 510.0f * ((float)raw / 65535.0f) - 273.15f;
}

// ──────────────────────────────────────────────────────────────────────────────
// Profile resolution
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_resolveProfile() {
    RawProfile tmp;
    for (uint8_t i = 0; i < kProfileCount; i++) {
        memcpy_P(&tmp, &kProfiles[i], sizeof(RawProfile));
        if ((BatteryChem)tmp.chem == _chem && tmp.classCode == _classCode) {
            _profile.valid       = true;
            _profile.chem        = (BatteryChem)tmp.chem;
            _profile.classCode   = tmp.classCode;
            _profile.name        = tmp.name;
            _profile.seriesCount = tmp.seriesCount;
            _profile.leadBased   = tmp.leadBased;
            _profile.vEmpty      = tmp.vEmpty;
            _profile.vNominal    = tmp.vNominal;
            _profile.vFull       = tmp.vFull;
            _profile.vAbsLow     = tmp.vAbsLow;
            _profile.vAbsHigh    = tmp.vAbsHigh;
            _copyProfileName();
            return;
        }
    }
    _profile = {}; _profile.valid = false; _profile.name = N_INVALID;
    _copyProfileName();
}

void LTC2944_BMS::_copyProfileName() {
    strncpy_P(_profileNameBuf, (PGM_P)_profile.name, sizeof(_profileNameBuf) - 1);
    _profileNameBuf[sizeof(_profileNameBuf) - 1] = '\0';
}

bool LTC2944_BMS::_isVoltagePlausible(float v) const {
    if (!_profile.valid) return false;
    // Use override full voltage for upper bound if set
    float absHigh = (_overrideFullV > 0.0f) ? (_overrideFullV + 0.1f) : (_profile.vAbsHigh + 0.5f);
    return v >= _profile.vAbsLow && v <= absHigh;
}

// ──────────────────────────────────────────────────────────────────────────────
// SOC estimation
// ──────────────────────────────────────────────────────────────────────────────

int LTC2944_BMS::_estimateSoc(float vbat, float currentA, uint16_t rawACR, bool charging) {
    if (!_profile.valid || _profile.seriesCount == 0) return 0;

    // During the boot-settle window the ACR register may not have integrated
    // away from chip reset default yet. Use voltage-only SOC and pass the
    // saved full-charge anchor as rawACR so the emptyRaw guard in
    // _socFromVoltageCurve cannot force SOC=0 on a good battery voltage.
    if (_acrBootTicks > 0) {
        _acrBootTicks--;
        uint16_t safeACR = (_cal.valid && _cal.hasFullRaw) ? _cal.fullChargeRaw : rawACR;
        switch (_profile.chem) {
            case CHEM_LIION:   return _socFromVoltageCurve(vbat, currentA, safeACR, charging, 0.03f, 40, 100);
            case CHEM_LIFEPO4: return _socFromVoltageCurve(vbat, currentA, safeACR, charging, 0.02f, 35, 100);
            default:           return _socFromVoltageCurve(vbat, currentA, safeACR, charging, 0.10f, 50, 100);
        }
    }

    // If ACR rolled over, fall back to voltage-only SOC until recalibrated
    if (_acrRollover) {
        switch (_profile.chem) {
            case CHEM_LIION:   return _socFromVoltageCurve(vbat, currentA, rawACR, charging, 0.03f, 40, 100);
            case CHEM_LIFEPO4: return _socFromVoltageCurve(vbat, currentA, rawACR, charging, 0.02f, 35, 100);
            default:           return _socFromVoltageCurve(vbat, currentA, rawACR, charging, 0.10f, 50, 100);
        }
    }

    int voltageSoc;
    switch (_profile.chem) {
        case CHEM_LIION:   voltageSoc = _socFromVoltageCurve(vbat, currentA, rawACR, charging, 0.03f, 40, 100); break;
        case CHEM_LIFEPO4: voltageSoc = _socFromVoltageCurve(vbat, currentA, rawACR, charging, 0.02f, 35, 100); break;
        default:           voltageSoc = _socFromVoltageCurve(vbat, currentA, rawACR, charging, 0.10f, 50, 100); break;
    }
    int coulombSoc = _estimateCoulombSoc(rawACR);
    if (coulombSoc < 0) return voltageSoc;
    return _adaptiveBlend(coulombSoc, voltageSoc, currentA);
}

int LTC2944_BMS::_socFromVoltageCurve(float vbat, float currentA, uint16_t rawACR,
                                       bool charging, float chargeCompV,
                                       int lowerBandSoc, int upperBandSoc) {
    if (!_profile.valid || _profile.seriesCount == 0) return 0;

    const float irOhm = _effectiveResistance();
    const float relaxCurrentA = 0.05f, fullCompCurrent = 0.50f;

    float cellV  = vbat / (float)_profile.seriesCount;
    // Use effective voltages (override > calibrated > profile default)
    float vEmpty = _effectiveVEmpty() / (float)_profile.seriesCount;
    float vNom   = _profile.vNominal  / (float)_profile.seriesCount;
    float vFull  = _effectiveVFull()  / (float)_profile.seriesCount;

    float absI = _absf(currentA);
    float relax;
    if      (absI <= relaxCurrentA)   relax = 0.0f;
    else if (absI >= fullCompCurrent) relax = 1.0f;
    else                              relax = (absI - relaxCurrentA) / (fullCompCurrent - relaxCurrentA);

    float compV = cellV - (currentA * irOhm * relax);
    if (charging) compV -= (chargeCompV * relax);

    if (_cal.valid && _cal.hasEmptyRaw && rawACR <= _cal.emptyChargeRaw) return 0;

    int soc;
    if      (compV <= vEmpty) soc = 0;
    else if (compV >= vFull)  soc = 100;
    else if (compV <= vNom)
        soc = (int)(((compV - vEmpty) / (vNom - vEmpty)) * lowerBandSoc);
    else
        soc = lowerBandSoc + (int)(((compV - vNom) / (vFull - vNom)) * (upperBandSoc - lowerBandSoc));

    return _clamp(soc, 0, 100);
}

int LTC2944_BMS::_estimateCoulombSoc(uint16_t rawACR) const {
    if (!_cal.valid || !_cal.hasEmptyRaw || !_cal.hasFullRaw) return -1;
    if (_cal.fullChargeRaw == _cal.emptyChargeRaw) return -1;
    long span = (long)_cal.fullChargeRaw - (long)_cal.emptyChargeRaw;
    long pos  = (long)rawACR             - (long)_cal.emptyChargeRaw;
    return _clamp((int)((100L * pos) / span), 0, 100);
}

int LTC2944_BMS::_adaptiveBlend(int coulombSoc, int voltageSoc, float currentA) const {
    float absI = _absf(currentA);
    float vw;
    if      (absI < 0.2f) vw = 0.50f;
    else if (absI < 1.0f) vw = 0.30f;
    else if (absI < 5.0f) vw = 0.15f;
    else                  vw = 0.05f;
    return _clamp((int)lround((float)coulombSoc * (1.0f - vw) + (float)voltageSoc * vw), 0, 100);
}

// ──────────────────────────────────────────────────────────────────────────────
// Alarms — uses configurable thresholds
// ──────────────────────────────────────────────────────────────────────────────

uint16_t LTC2944_BMS::_makeAlarms(float v, float i, float t, bool sensorOk) const {
    uint16_t a = 0;
    if (!sensorOk)         a |= ALARM_SENSOR_FAIL;
    if (!_profile.valid)   a |= ALARM_NO_PROFILE;
    if (_profile.valid) {
        float vEmpty = _effectiveVEmpty();
        float vFull  = _effectiveVFull();
        // Only fire UV/OV alarms when the voltage window is valid.
        // This suppresses false alarms when:
        //   - No calibration data exists and vFull == vEmpty (both from profile
        //     but overrides not set — shouldn't happen but guard anyway)
        //   - Calibration partially complete (empty saved, full not yet saved)
        //     which causes vFull to still hold the profile default while vEmpty
        //     was just updated to the measured empty voltage
        // The 0.5V minimum window covers all supported chemistries.
        bool voltageWindowValid = (vFull > vEmpty + 0.5f);
        if (voltageWindowValid) {
            if (v < vEmpty)  a |= ALARM_UNDER_VOLTAGE;
            // OV threshold: calibrated vFull is the observed charger-cutoff
            // voltage, not a hard ceiling. The pack voltage can exceed it by a
            // small amount while the charger is still in the CV phase, or when
            // a hard-cutoff charger reconnects. Add a per-cell margin (50 mV)
            // so the alarm only fires if the voltage is genuinely anomalous.
            // If no calibration is active the profile vFull is used directly
            // (it already carries headroom from the profile definition).
            float seriesN  = (_profile.seriesCount > 0) ? (float)_profile.seriesCount : 1.0f;
            float vFullAlarm = vFull + (0.05f * seriesN);
            if (v > vFullAlarm)  a |= ALARM_OVER_VOLTAGE;
        }
        if (v < _profile.vAbsLow)   a |= ALARM_ABS_UNDER_V;
        if (v > _profile.vAbsHigh)  a |= ALARM_ABS_OVER_V;
    }
    if (t < -10.0f || t > 60.0f)   a |= ALARM_TEMPERATURE;
    if (i < -_maxDischargeA)        a |= ALARM_OVER_CURRENT;
    if (i >  _maxChargeA)           a |= ALARM_CHARGE_CURRENT;
    if (_profileMismatch)           a |= ALARM_PROFILE_MISMATCH;
    if (_acrRollover)               a |= ALARM_ACR_ROLLOVER;
    return a;
}

// ──────────────────────────────────────────────────────────────────────────────
// Disconnect detection
// ──────────────────────────────────────────────────────────────────────────────

bool LTC2944_BMS::_checkDisconnectOnReadFailure() {
    _i2cFailCount++;
    if (_i2cFailCount >= 2) { _i2cFailCount = 0; reinitChip(); }
    return false;
}

bool LTC2944_BMS::_checkDisconnectOnMeasurement(float vbat, float currentA) {
    if (!_profile.valid) return false;
    bool sig = (vbat < _profile.vAbsLow) && (_absf(currentA) < 0.05f);
    if (sig) {
        _disconnectCandidateCount++;
        if (_disconnectCandidateCount >= 2) { _writeDisconnectFlag(); return true; }
    } else {
        _disconnectCandidateCount = 0;
    }
    return false;
}

void LTC2944_BMS::_writeDisconnectFlag() { EEPROM.update(EE_DISCONNECT, DISCONNECT_MARKER); }
void LTC2944_BMS::_clearDisconnectFlag() { EEPROM.update(EE_DISCONNECT, 0x00); }
bool LTC2944_BMS::_readDisconnectFlag()  const { return EEPROM.read(EE_DISCONNECT) == DISCONNECT_MARKER; }

// ──────────────────────────────────────────────────────────────────────────────
// Calibration state machine
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_setLoad(bool en) { digitalWrite(_pinLoadEn, en ? HIGH : LOW); }

void LTC2944_BMS::_enterCalPhase(CalPhase phase) {
    _calPhase = phase; _calPhaseStartMs = millis();
    switch (phase) {
        case CAL_ZERO:
            _zeroSamplesTaken = 0; _zeroAccumRawDelta = 0;
            _calFullConfirmCount = 0;
            _setLoad(false); break;
        case CAL_WAIT:      _setLoad(false); break;
        case CAL_DISCHARGE:
            // Preset ACR to mid-scale before discharge begins. This maximises
            // headroom in both directions and prevents rollover on small or
            // high-IR packs that drain quickly from a low starting ACR position.
            // The empty anchor is saved as whatever value ACR reaches at vEmpty,
            // measured downward from 0x7FFF — always a valid decreasing count.
            _writeACR(0x7FFF);
            _acrRollover = false;
            _calRunStartMs = millis();
            _setLoad(true);
            break;
        case CAL_CHARGE:
            _calPeakChargeV = 0.0f;
            _setLoad(false);
            break;
        default:            _setLoad(false); break;
    }
}

void LTC2944_BMS::_updateCalibration(uint16_t rawCurr, uint16_t rawACR,
                                      float vbat, float tempC, float currentA) {
    if (!_calibrationMode) return;

    // ── Global safety checks — evaluated before any phase logic ─────────────

    // Battery disconnect: voltage below 1V is impossible for any valid pack.
    // Wait 5 seconds to allow accidental reconnect, then reinit and recheck.
    if (vbat < 1.0f) {
        Serial.println(F("# Cal: low voltage — waiting 5s for reconnect."));
        _setLoad(false);   // disable discharge load immediately
        delay(5000);
        if (!reinitChip()) {
            Serial.println(F("# Cal: abort — LTC2944 not responding after 5s."));
            _enterCalPhase(CAL_ABORT);
            return;
        }
        uint16_t rawVolt = 0;
        if (!_readReg16(REG_VOLTAGE_MSB, rawVolt) || _rawToVoltage(rawVolt) < 1.0f) {
            Serial.println(F("# Cal: abort — voltage still invalid after reinit."));
            _enterCalPhase(CAL_ABORT);
            return;
        }
        Serial.println(F("# Cal: voltage restored — continuing."));
        return;
    }

    // Over-temperature: abort active phases
    if ((_calPhase == CAL_DISCHARGE || _calPhase == CAL_CHARGE) && tempC > 55.0f) {
        Serial.println(F("# Cal: abort — over-temperature."));
        _enterCalPhase(CAL_ABORT);
        return;
    }

    // Charger connected during zero or discharge: current offset and empty
    // anchor would be corrupted. Abort immediately.
    if ((_calPhase == CAL_ZERO || _calPhase == CAL_WAIT || _calPhase == CAL_DISCHARGE)
            && currentA > _calChargerOnCurrentA) {
        Serial.println(F("# Cal: abort — charger connected during discharge phase."));
        _enterCalPhase(CAL_ABORT);
        return;
    }

    switch (_calPhase) {
        case CAL_ZERO: {
            // Wait CAL_SETTLE_MS before taking samples. This lets any residual
            // charger tail current or load transient decay to zero so the offset
            // measurement reflects true no-current conditions.
            if ((uint32_t)(millis() - _calPhaseStartMs) < CAL_SETTLE_MS) break;

            _zeroAccumRawDelta += (int32_t)rawCurr - 32767;
            if (++_zeroSamplesTaken >= ZERO_SAMPLES) {
                float avgDelta = (float)_zeroAccumRawDelta / (float)_zeroSamplesTaken;
                _currentOffset_mA = (0.064f / _shuntOhm) * (avgDelta / 32767.0f) * 1000.0f;
                _saveOffset(_currentOffset_mA);
                _enterCalPhase(CAL_WAIT);
            }
            break;
        }
        case CAL_WAIT:
            if ((uint32_t)(millis() - _calPhaseStartMs) >= CAL_SETTLE_MS)
                _enterCalPhase(CAL_DISCHARGE);
            break;
        case CAL_DISCHARGE:
            if (_profile.valid && vbat <= _effectiveVEmpty()) {
                _saveCalEmpty(vbat, rawACR);
                _calFullConfirmCount = 0;
                _enterCalPhase(CAL_CHARGE);
            } else if ((uint32_t)(millis() - _calRunStartMs) > _calTimeoutMs) {
                Serial.println(F("# Cal: abort — discharge timeout."));
                _enterCalPhase(CAL_ABORT);
            }
            break;
        case CAL_CHARGE: {
            // Safety backstop: keep load off throughout the charge phase.
            // _enterCalPhase already called _setLoad(false) but the hardware
            // may not respond instantly. Repeating it each tick costs nothing.
            _setLoad(false);

            // Two detection paths for full-charge:
            //
            // PATH A — CV taper (standard chargers):
            //   charger still connected (currentA > threshold) AND
            //   voltage near full AND current has tapered below _calFullCurrentA
            //   Requires CAL_FULL_CONFIRM_SAMPLES consecutive qualifying readings.
            //
            // PATH B — hard cutoff (chargers that abruptly disconnect at 100%):
            //   charging flag just transitioned YES -> NO  AND
            //   voltage is still near full (hasn't dropped yet)
            //   One sample is sufficient — the voltage is the confirmation.

            float vFull    = _effectiveVFull();
            bool  nearFull = _profile.valid && vbat >= (vFull * CAL_FULL_VOLT_PCT);

            // Track the highest voltage seen during this charge phase.
            // Used by PATH B as a charger-agnostic full indicator: if voltage
            // drops from the observed peak the charger has cut off, regardless
            // of whether the pack reached the profile vFull.
            if (vbat > _calPeakChargeV) _calPeakChargeV = vbat;  // high-water mark

            // nearFull: profile threshold OR voltage has pulled back >20mV from
            // the observed charge peak (charger cut off below profile vFull).
            // The 0.95*vFull guard prevents triggering during early charge when
            // the peak is still low.
            bool peakDropped = (_calPeakChargeV > (vFull * 0.95f))
                            && (vbat < (_calPeakChargeV - 0.020f));
            nearFull = nearFull || peakDropped;

            // PATH B — hard-cutoff detection
            // A genuine hard cutoff means the charger physically disconnected:
            // current must be zero (within deadband), not merely below the
            // taper threshold. This prevents a balance pulse or a brief
            // measurement dip from triggering a false full-charge anchor.
            bool currentZero = (currentA == 0.0f);  // deadband already applied in _rawToCurrent
            // Also require that no full anchor has been saved yet. Without this,
            // a load cutoff at empty (where _cal.vFull is still 0.0f and
            // nearFull would be true for any voltage) could trigger PATH B.
            bool fullNotYetSaved = !_cal.hasFullRaw;

            if (_prevCharging && !_charging && nearFull && currentZero && fullNotYetSaved) {
                Serial.println(F("# Cal: hard-cutoff charger detected — saving full anchor."));
                _saveCalFull(vbat, rawACR);
                _enterCalPhase(CAL_DONE);
                break;
            }

            // PATH A — CV taper detection
            bool chargerOn = (currentA > _calChargerOnCurrentA);
            bool taperedOk = (currentA <= _calFullCurrentA);

            if (chargerOn && nearFull && taperedOk) {
                _calFullConfirmCount++;
                if (_calFullConfirmCount >= CAL_FULL_CONFIRM_SAMPLES) {
                    _saveCalFull(vbat, rawACR);
                    _enterCalPhase(CAL_DONE);
                }
            } else {
                _calFullConfirmCount = 0;
            }

            if ((uint32_t)(millis() - _calPhaseStartMs) > _calTimeoutMs) {
                Serial.println(F("# Cal: abort — charge timeout."));
                _enterCalPhase(CAL_ABORT);
            }
            break;
        }
        default: break;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// EEPROM
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_loadOffset() {
    uint32_t magic = 0; EEPROM.get(EE_MAGIC, magic);
    if (magic == EEPROM_MAGIC) {
        EEPROM.get(EE_OFFSET, _currentOffset_mA);
        if (!isfinite(_currentOffset_mA) || _absf(_currentOffset_mA) > 500.0f)
            _currentOffset_mA = 0.0f;
    } else { _currentOffset_mA = 0.0f; }
}

void LTC2944_BMS::_saveOffset(float o) { EEPROM.put(EE_MAGIC, EEPROM_MAGIC); EEPROM.put(EE_OFFSET, o); }

void LTC2944_BMS::_loadCalibration() {
    _cal = {};
    uint32_t magic = 0; EEPROM.get(EE_CAL_MAGIC, magic);
    if (magic != CAL_MAGIC) return;

    EEPROM.get(EE_CAL_EMPTY_V, _cal.vEmpty);
    EEPROM.get(EE_CAL_FULL_V,  _cal.vFull);
    _cal.valid = true;

    uint16_t m = 0;
    EEPROM.get(EE_CAL_EMPTY_MAGIC, m);
    if (m == EMPTY_ACR_MAGIC) { EEPROM.get(EE_CAL_EMPTY_RAW, _cal.emptyChargeRaw); _cal.hasEmptyRaw = true; }
    EEPROM.get(EE_CAL_FULL_MAGIC, m);
    if (m == FULL_ACR_MAGIC)  { EEPROM.get(EE_CAL_FULL_RAW,  _cal.fullChargeRaw);  _cal.hasFullRaw  = true; }

    // ── Sanity check — auto-reset if data is obviously corrupt ───────────────
    // Any of these conditions indicate a failed or partial calibration run:
    //   1. vEmpty is not a finite number
    //   2. vEmpty is negative or unrealistically high (>80V)
    //   3. If full anchor present: vFull not finite, <= vEmpty, window < 0.5V,
    //      vFull > 80V, or ACR anchors inverted/identical
    // NOTE: vFull checks are only applied when hasFullRaw is true. A partially
    // complete calibration (empty saved, full not yet saved) is valid — vFull
    // will be 0.0f from EEPROM uninitialised memory and must not be validated.
    bool corrupt = false;
    if (!isfinite(_cal.vEmpty))                            corrupt = true;
    if (_cal.vEmpty < 0.0f || _cal.vEmpty > 80.0f)        corrupt = true;
    if (_cal.hasFullRaw) {
        if (!isfinite(_cal.vFull))                         corrupt = true;
        if (_cal.vFull <= _cal.vEmpty)                     corrupt = true;
        if ((_cal.vFull - _cal.vEmpty) < 0.5f)            corrupt = true;
        if (_cal.vFull > 80.0f)                            corrupt = true;
        if (_cal.hasEmptyRaw &&
            _cal.fullChargeRaw == _cal.emptyChargeRaw)     corrupt = true;
        // ACR counts DOWN during discharge so fullChargeRaw must be greater
        // than emptyChargeRaw. Inverted window = old firmware anchor.
        if (_cal.hasEmptyRaw &&
            _cal.fullChargeRaw <= _cal.emptyChargeRaw)     corrupt = true;
    }

    if (corrupt) {
        _cal = {};   // clear in RAM
        // Erase EEPROM magic word so corrupt data is not reloaded next boot
        uint32_t zero = 0;
        EEPROM.put(EE_CAL_MAGIC, zero);
    }
}

void LTC2944_BMS::_saveCalEmpty(float v, uint16_t raw) {
    // Park the ACR register away from the 0x0000 boundary to prevent bottom
    // rollover. When the battery is empty the ACR is near 0x0000 — if charge
    // starts immediately the counter can underflow to 0xFFFF making SOC jump
    // to 100%. Parking at 0x0060 gives ~96 LSB of headroom before rollover.
    //
    // IMPORTANT: we save the parked value (0x0060) as the EEPROM anchor, not
    // the measured raw. The chip register and the saved anchor must always
    // agree so that _estimateCoulombSoc is consistent from the very first read
    // after boot or after flashing new firmware.
    static const uint16_t PARK_EMPTY = 0x0060;
    _writeACR(PARK_EMPTY);

    _cal.vEmpty = v; _cal.emptyChargeRaw = PARK_EMPTY; _cal.hasEmptyRaw = true; _cal.valid = true;
    EEPROM.put(EE_CAL_MAGIC, CAL_MAGIC); EEPROM.put(EE_CAL_EMPTY_V, v);
    EEPROM.put(EE_CAL_EMPTY_RAW, PARK_EMPTY); EEPROM.put(EE_CAL_EMPTY_MAGIC, EMPTY_ACR_MAGIC);
    (void)raw;  // raw (pre-park) discarded — parked value is the reference
}

void LTC2944_BMS::_saveCalFull(float v, uint16_t raw) {
    // Park the ACR register away from the 0xFFFF boundary to prevent top
    // rollover. When the battery is full the ACR is near 0xFFFF — if discharge
    // starts immediately the counter can overflow to 0x0000 making SOC jump
    // to 0%. Parking at 0xFF9F gives ~96 LSB of headroom before rollover.
    //
    // IMPORTANT: we save the parked value (0xFF9F) as the EEPROM anchor, not
    // the measured raw. The chip register and the saved anchor must always
    // agree so that _estimateCoulombSoc is consistent from the very first read
    // after boot or after flashing new firmware.
    static const uint16_t PARK_FULL = 0xFF9F;
    _writeACR(PARK_FULL);

    _cal.vFull = v; _cal.fullChargeRaw = PARK_FULL; _cal.hasFullRaw = true; _cal.valid = true;
    EEPROM.put(EE_CAL_MAGIC, CAL_MAGIC); EEPROM.put(EE_CAL_FULL_V, v);
    EEPROM.put(EE_CAL_FULL_RAW, PARK_FULL); EEPROM.put(EE_CAL_FULL_MAGIC, FULL_ACR_MAGIC);
    (void)raw;  // raw (pre-park) discarded — parked value is the reference
}

// ──────────────────────────────────────────────────────────────────────────────
// Learned model
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_resetLearnedModel(bool) {
    _model = {}; _model.resistance = 0.08f; _model.dirty = true;
    _model.storedChem = (uint8_t)_chem; _model.storedClassCode = _classCode;
}

bool LTC2944_BMS::_loadLearnedModel() {
    LearnedRecord a = {}, b = {};
    EEPROM.get(EE_LEARNED_SLOT_A, a); EEPROM.get(EE_LEARNED_SLOT_B, b);
    bool va = _isRecordValid(a), vb = _isRecordValid(b);
    if (!va && !vb) { _resetLearnedModel(false); return false; }
    const LearnedRecord* n = (va && vb) ? (a.sequence >= b.sequence ? &a : &b) : (va ? &a : &b);
    if (n->storedChem != (uint8_t)_chem || n->storedClassCode != _classCode) {
        _resetLearnedModel(false); return false;
    }
    _model.resistance = n->resistance; _model.confidence = n->confidence;
    _model.valid = (n->confidence > 0.0f); _model.dirty = false;
    _model.samples = n->samples; _model.lastRelaxedVoltage = n->lastRelaxedVoltage;
    _model.lastSeenRawACR = n->lastSeenRawACR;
    _model.storedChem = n->storedChem; _model.storedClassCode = n->storedClassCode;
    return true;
}

bool LTC2944_BMS::_saveLearnedModel(bool force) {
    if (!force && (!_model.dirty || (uint32_t)(millis() - _lastLearnedSaveMs) < LEARNED_SAVE_INTERVAL_MS))
        return false;
    LearnedRecord a = {}, b = {};
    EEPROM.get(EE_LEARNED_SLOT_A, a); EEPROM.get(EE_LEARNED_SLOT_B, b);
    uint32_t seqA = _isRecordValid(a) ? a.sequence : 0;
    uint32_t seqB = _isRecordValid(b) ? b.sequence : 0;
    LearnedRecord out = {};
    out.magic = LEARNED_MAGIC; out.version = LEARNED_VERSION;
    out.length = sizeof(LearnedRecord); out.sequence = max(seqA, seqB) + 1UL;
    out.resistance = _model.resistance; out.confidence = _model.confidence;
    out.samples = _model.samples; out.lastRelaxedVoltage = _model.lastRelaxedVoltage;
    out.lastSeenRawACR = _model.lastSeenRawACR;
    out.storedChem = (uint8_t)_chem; out.storedClassCode = _classCode;
    out.crc = _recordCrc(out);
    if (seqA <= seqB) EEPROM.put(EE_LEARNED_SLOT_A, out);
    else              EEPROM.put(EE_LEARNED_SLOT_B, out);
    _model.dirty = false; _lastLearnedSaveMs = millis();
    return true;
}

void LTC2944_BMS::_updateResistanceLearning(float v, float i, uint32_t now) {
    const float minDI=0.50f, minDV=0.02f, maxR=0.50f, alpha=0.10f;
    const uint32_t window=3000UL;
    if (!_resLearn.baselineValid) {
        if (_currentNearZero(i)) { _resLearn.baselineV=v; _resLearn.baselineI=i; _resLearn.baselineMs=now; _resLearn.baselineValid=true; }
        return;
    }
    if ((uint32_t)(now-_resLearn.baselineMs) < window) {
        if (_currentNearZero(i)) { _resLearn.baselineV=v; _resLearn.baselineI=i; _resLearn.baselineMs=now; }
        return;
    }
    float dI=i-_resLearn.baselineI, dV=_resLearn.baselineV-v;
    if (_absf(dI) >= minDI && _absf(dV) >= minDV) {
        float R=dV/_absf(dI);
        if (R>=0.0f && R<=maxR) {
            _model.resistance = _model.valid ? (1.0f-alpha)*_model.resistance+alpha*R : R;
            _model.samples++; _model.confidence=_clampf((float)_model.samples/12.0f,0.0f,1.0f);
            _model.valid=true; _model.dirty=true;
        }
        _resLearn.baselineValid=false; return;
    }
    if (_currentNearZero(i)) { _resLearn.baselineV=v; _resLearn.baselineI=i; _resLearn.baselineMs=now; _resLearn.baselineValid=true; }
    else if (_absf(i-_resLearn.baselineI)>0.20f) _resLearn.baselineValid=false;
}

void LTC2944_BMS::_updateRelaxedSignature(float v, float i, uint16_t rawACR, uint32_t now) {
    if (!_currentNearZero(i)) return;
    _model.lastRelaxedVoltage = (_model.lastRelaxedVoltage <= 0.01f) ? v
        : 0.95f * _model.lastRelaxedVoltage + 0.05f * v;
    _model.lastSeenRawACR = rawACR; _lastRelaxedSeenMs = now; _model.dirty = true;
}

float LTC2944_BMS::_effectiveResistance() const {
    // Only apply learned IR compensation once confidence is solid (>= 0.5).
    // Below this threshold the model has too few samples to be reliable,
    // and using the 80 mOhm default on a low-IR pack (e.g. large LiFePO4)
    // would cause large SOC jumps when load connects or disconnects.
    return (_model.valid && _model.confidence >= 0.50f) ? _model.resistance : 0.0f;
}

// ──────────────────────────────────────────────────────────────────────────────
// CRC
// ──────────────────────────────────────────────────────────────────────────────

uint8_t LTC2944_BMS::_crc8(const uint8_t* data, size_t len) const {
    uint8_t crc = 0;
    while (len--) {
        uint8_t b = *data++;
        for (uint8_t i=0; i<8; i++) { uint8_t mix=(crc^b)&1; crc>>=1; if(mix) crc^=0x8C; b>>=1; }
    }
    return crc;
}

uint8_t LTC2944_BMS::_recordCrc(const LearnedRecord& r) const { return _crc8((const uint8_t*)&r, sizeof(LearnedRecord)-1); }

bool LTC2944_BMS::_isRecordValid(const LearnedRecord& r) const {
    if (r.magic!=LEARNED_MAGIC || r.version!=LEARNED_VERSION || r.length!=sizeof(LearnedRecord)) return false;
    if (_recordCrc(r)!=r.crc) return false;
    if (!isfinite(r.resistance)||r.resistance<0||r.resistance>0.50f) return false;
    if (!isfinite(r.confidence)||r.confidence<0||r.confidence>1.0f)  return false;
    if (!isfinite(r.lastRelaxedVoltage)||r.lastRelaxedVoltage<0)      return false;
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────────

int   LTC2944_BMS::_clamp(int v, int lo, int hi)        { return v<lo?lo:v>hi?hi:v; }
float LTC2944_BMS::_clampf(float v, float lo, float hi) { return v<lo?lo:v>hi?hi:v; }
float LTC2944_BMS::_absf(float v)                        { return v>=0.0f?v:-v; }
bool  LTC2944_BMS::_currentNearZero(float i) const       { return _absf(i)<0.15f; }
