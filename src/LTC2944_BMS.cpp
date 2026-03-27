#include "LTC2944_BMS.h"

// ──────────────────────────────────────────────────────────────────────────────
// LTC2944 I²C address & registers
// ──────────────────────────────────────────────────────────────────────────────
static const uint8_t LTC2944_ADDR           = 0x64;
static const uint8_t REG_STATUS             = 0x00;
static const uint8_t REG_CONTROL            = 0x01;
static const uint8_t REG_ACC_CHARGE_MSB     = 0x02;
static const uint8_t REG_VOLTAGE_MSB        = 0x08;
static const uint8_t REG_CURRENT_MSB        = 0x0E;
static const uint8_t REG_TEMPERATURE_MSB    = 0x14;

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
static const int EE_LEARNED_SLOT_B  = EE_LEARNED_SLOT_A + (int)sizeof(LTC2944_BMS::LearnedRecord);

static const uint32_t EEPROM_MAGIC       = 0x42C0FFEEul;
static const uint32_t CAL_MAGIC          = 0xCA1BCA1Bul;
static const uint16_t EMPTY_ACR_MAGIC    = 0xA44Cu;
static const uint16_t FULL_ACR_MAGIC     = 0xF44Cu;
static const uint8_t  DISCONNECT_MARKER  = 0xD1;
static const uint16_t LEARNED_MAGIC      = 0x4C42u;
static const uint8_t  LEARNED_VERSION    = 1;
static const uint32_t LEARNED_SAVE_INTERVAL_MS = 30UL * 60UL * 1000UL;

// ──────────────────────────────────────────────────────────────────────────────
// Calibration timing
// ──────────────────────────────────────────────────────────────────────────────
static const uint16_t ZERO_SAMPLES        = 32;
static const uint32_t CAL_SETTLE_MS       = 5000UL;
static const uint32_t MAX_CAL_TIME_MS     = 30UL * 60UL * 1000UL;
static const float    CAL_FULL_CURRENT_A  = 0.10f;
static const float    CAL_FULL_VOLT_PCT   = 0.98f;

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

// Internal profile table stored in flash
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
    {true,CHEM_LIION,  0,N_LIION_1S,   1,false, 3.0f, 3.7f, 4.2f, 2.5f, 4.3f},
    {true,CHEM_LIION,  1,N_LIION_2S,   2,false, 6.0f, 7.4f, 8.4f, 5.0f, 8.6f},
    {true,CHEM_LIION,  2,N_LIION_3S,   3,false, 9.0f,11.1f,12.6f, 7.5f,12.9f},
    {true,CHEM_LIION,  3,N_LIION_4S,   4,false,12.0f,14.8f,16.8f,10.0f,17.2f},
    {true,CHEM_LIION,  4,N_LIION_6S,   6,false,18.0f,22.2f,25.2f,15.0f,25.8f},
    {true,CHEM_LIION,  5,N_LIION_8S,   8,false,24.0f,29.6f,33.6f,20.0f,34.4f},
    {true,CHEM_LIION,  6,N_LIION_12S, 12,false,36.0f,44.4f,50.4f,30.0f,51.6f},
    {true,CHEM_LIION,  7,N_LIION_14S, 14,false,42.0f,51.8f,58.8f,35.0f,60.2f},
    {true,CHEM_LIFEPO4,0,N_LFP_1S,    1,false, 2.8f, 3.2f, 3.6f, 2.5f, 3.8f},
    {true,CHEM_LIFEPO4,1,N_LFP_2S,    2,false, 5.6f, 6.4f, 7.2f, 5.0f, 7.6f},
    {true,CHEM_LIFEPO4,2,N_LFP_3S,    3,false, 8.4f, 9.6f,10.8f, 7.5f,11.4f},
    {true,CHEM_LIFEPO4,3,N_LFP_4S,    4,false,11.2f,12.8f,14.4f,10.0f,15.2f},
    {true,CHEM_LIFEPO4,4,N_LFP_6S,    6,false,16.8f,19.2f,21.6f,15.0f,22.8f},
    {true,CHEM_LIFEPO4,5,N_LFP_8S,    8,false,22.4f,25.6f,28.8f,20.0f,30.4f},
    {true,CHEM_LIFEPO4,6,N_LFP_12S,  12,false,33.6f,38.4f,43.2f,30.0f,45.6f},
    {true,CHEM_LIFEPO4,7,N_LFP_14S,  14,false,39.2f,44.8f,50.4f,35.0f,53.2f},
    {true,CHEM_AGM,    0,N_AGM_12V,   6,true, 10.5f,12.6f,14.4f,10.0f,14.8f},
    {true,CHEM_AGM,    1,N_AGM_24V,  12,true, 21.0f,25.2f,28.8f,20.0f,29.6f},
    {true,CHEM_AGM,    2,N_AGM_48V,  24,true, 42.0f,50.4f,57.6f,40.0f,59.2f},
    {true,CHEM_GEL,    0,N_GEL_12V,   6,true, 10.5f,12.6f,14.2f,10.0f,14.8f},
    {true,CHEM_GEL,    1,N_GEL_24V,  12,true, 21.0f,25.2f,28.4f,20.0f,29.6f},
    {true,CHEM_GEL,    2,N_GEL_48V,  24,true, 42.0f,50.4f,56.8f,40.0f,59.2f},
};
static const uint8_t kProfileCount = sizeof(kProfiles) / sizeof(RawProfile);

// ──────────────────────────────────────────────────────────────────────────────
// Construction & configuration
// ──────────────────────────────────────────────────────────────────────────────

LTC2944_BMS::LTC2944_BMS()
    : _chem(CHEM_LIION)
    , _classCode(0)
    , _shuntOhm(0.010f)
    , _pinLoadEn(BMS_DEFAULT_PIN_LOAD_EN)
    , _pinStatusLed(BMS_DEFAULT_PIN_STATUS_LED)
    , _currentDeadbandA(0.030f)
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
    , _profile{}
    , _profileNameBuf{}
    , _cal{}
    , _model{}
    , _resLearn{}
    , _lastLearnedSaveMs(0)
    , _lastRelaxedSeenMs(0)
{}

void LTC2944_BMS::setProfile(BatteryChem chem, uint8_t classCode) {
    _chem = chem;
    _classCode = classCode;
}

void LTC2944_BMS::setShuntResistor(float ohms)       { _shuntOhm = ohms; }
void LTC2944_BMS::setPinLoadEnable(uint8_t pin)       { _pinLoadEn = pin; }
void LTC2944_BMS::setPinStatusLed(uint8_t pin)        { _pinStatusLed = pin; }
void LTC2944_BMS::setCurrentDeadband(float amps)      { _currentDeadbandA = amps; }

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

    return reinitChip();
}

bool LTC2944_BMS::reinitChip() {
    if (!_writeReg8(REG_CONTROL, 0xFC)) return false;   // automatic scan mode
    delay(50);
    uint8_t ctrl = 0;
    return _readReg8(REG_CONTROL, ctrl);
}

// ──────────────────────────────────────────────────────────────────────────────
// Main loop call
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

    float vbat    = _rawToVoltage(rawVolt);
    float current = _rawToCurrent(rawCurr);
    float tempC   = _rawToTemperature(rawTemp);

    _profileMismatch = !_isVoltagePlausible(vbat);

    // Hysteretic charging flag
    if (!_charging && current >  0.03f) _charging = true;
    else if (_charging && current < 0.01f) _charging = false;

    _updateCalibration(rawCurr, rawACR, vbat, tempC, current);

    if (_checkDisconnectOnMeasurement(vbat, current)) {
        _disconnected = true;
        return false;
    }

    int soc = _estimateSoc(vbat, current, rawACR, _charging);

    _updateResistanceLearning(vbat, current, now);
    _updateRelaxedSignature(vbat, current, rawACR, now);
    _saveLearnedModel(false);

    _meas.soc         = soc;
    _meas.voltage     = vbat;
    _meas.current     = current;
    _meas.temperature = tempC;
    _meas.rawACR      = rawACR;
    _meas.rawCurrent  = rawCurr;
    _meas.charging    = _charging;
    _meas.alarms      = _makeAlarms(vbat, current, tempC, true);

    // Status LED: on during active calibration phases
    bool ledOn = _calibrationMode
              && _calPhase != CAL_DONE
              && _calPhase != CAL_ABORT;
    digitalWrite(_pinStatusLed, ledOn ? HIGH : LOW);

    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Accessors
// ──────────────────────────────────────────────────────────────────────────────

int      LTC2944_BMS::getSoc()         const { return _meas.soc; }
float    LTC2944_BMS::getVoltage()     const { return _meas.voltage; }
float    LTC2944_BMS::getCurrent()     const { return _meas.current; }
float    LTC2944_BMS::getTemperature() const { return _meas.temperature; }
uint16_t LTC2944_BMS::getAlarms()     const { return _meas.alarms; }
bool     LTC2944_BMS::isCharging()    const { return _meas.charging; }

BmsMeasurement LTC2944_BMS::getMeasurement() const { return _meas; }

bool LTC2944_BMS::isBatteryDisconnected() const { return _disconnected; }

// ──────────────────────────────────────────────────────────────────────────────
// Calibration public API
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::startCalibration() {
    _calibrationMode = true;
    _enterCalPhase(CAL_ZERO);
}

void LTC2944_BMS::stopCalibration() {
    _calibrationMode = false;
    _enterCalPhase(CAL_NONE);
}

CalPhase LTC2944_BMS::getCalPhase() const { return _calPhase; }

bool LTC2944_BMS::isCalibrated() const {
    return _cal.valid
        && _cal.hasEmptyRaw
        && _cal.hasFullRaw
        && (_cal.fullChargeRaw != _cal.emptyChargeRaw);
}

void LTC2944_BMS::resetCalibration() {
    _cal = {};
    // Erase EEPROM magic word so calibrationLoad() returns invalid next boot
    uint32_t zero = 0;
    EEPROM.put(EE_CAL_MAGIC, zero);
}

// ──────────────────────────────────────────────────────────────────────────────
// Learned model public API
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

const char* LTC2944_BMS::getProfileName() const {
    return _profileNameBuf;
}

bool LTC2944_BMS::isProfilePlausible() const {
    return !_profileMismatch;
}

// ──────────────────────────────────────────────────────────────────────────────
// I²C helpers
// ──────────────────────────────────────────────────────────────────────────────

bool LTC2944_BMS::_writeReg8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LTC2944_ADDR);
    Wire.write(reg);
    Wire.write(val);
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
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    val = ((uint16_t)msb << 8) | lsb;
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Conversions
// ──────────────────────────────────────────────────────────────────────────────

float LTC2944_BMS::_rawToVoltage(uint16_t raw) const {
    return 70.8f * ((float)raw / 65535.0f);
}

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
    _profile       = {};
    _profile.valid = false;
    _profile.name  = N_INVALID;
    _copyProfileName();
}

void LTC2944_BMS::_copyProfileName() {
    strncpy_P(_profileNameBuf, (PGM_P)_profile.name, sizeof(_profileNameBuf) - 1);
    _profileNameBuf[sizeof(_profileNameBuf) - 1] = '\0';
}

bool LTC2944_BMS::_isVoltagePlausible(float v) const {
    if (!_profile.valid) return false;
    return v >= _profile.vAbsLow && v <= (_profile.vAbsHigh + 0.5f);
}

// ──────────────────────────────────────────────────────────────────────────────
// SOC estimation
// ──────────────────────────────────────────────────────────────────────────────

int LTC2944_BMS::_estimateSoc(float vbat, float currentA, uint16_t rawACR, bool charging) {
    if (!_profile.valid || _profile.seriesCount == 0) return 0;

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

    const float irOhm          = _effectiveResistance();
    const float relaxCurrentA  = 0.05f;
    const float fullCompCurrent = 0.50f;

    float cellV    = vbat / (float)_profile.seriesCount;
    float vEmpty   = (_cal.valid && isfinite(_cal.vEmpty) ? _cal.vEmpty : _profile.vEmpty)
                     / (float)_profile.seriesCount;
    float vNom     = _profile.vNominal / (float)_profile.seriesCount;
    float vFull    = (_cal.valid && isfinite(_cal.vFull)  ? _cal.vFull  : _profile.vFull)
                     / (float)_profile.seriesCount;

    float absI = _absf(currentA);
    float relax;
    if      (absI <= relaxCurrentA)   relax = 0.0f;
    else if (absI >= fullCompCurrent) relax = 1.0f;
    else                              relax = (absI - relaxCurrentA) / (fullCompCurrent - relaxCurrentA);

    float compV = cellV - (currentA * irOhm * relax);
    if (charging) compV -= (chargeCompV * relax);

    // If ACR is at or below calibrated empty, SOC = 0
    if (_cal.valid && _cal.hasEmptyRaw && rawACR <= _cal.emptyChargeRaw) return 0;

    int soc;
    if      (compV <= vEmpty) soc = 0;
    else if (compV >= vFull)  soc = 100;
    else if (compV <= vNom)
        soc = (int)(((compV - vEmpty) / (vNom - vEmpty)) * lowerBandSoc);
    else
        soc = lowerBandSoc +
              (int)(((compV - vNom) / (vFull - vNom)) * (upperBandSoc - lowerBandSoc));

    return _clamp(soc, 0, 100);
}

int LTC2944_BMS::_estimateCoulombSoc(uint16_t rawACR) const {
    if (!_cal.valid || !_cal.hasEmptyRaw || !_cal.hasFullRaw) return -1;
    if (_cal.fullChargeRaw == _cal.emptyChargeRaw) return -1;

    long span = (long)_cal.fullChargeRaw - (long)_cal.emptyChargeRaw;
    long pos  = (long)rawACR             - (long)_cal.emptyChargeRaw;
    int  soc  = (int)((100L * pos) / span);
    return _clamp(soc, 0, 100);
}

int LTC2944_BMS::_adaptiveBlend(int coulombSoc, int voltageSoc, float currentA) const {
    float absI = _absf(currentA);
    float vw;
    if      (absI < 0.2f) vw = 0.50f;
    else if (absI < 1.0f) vw = 0.30f;
    else if (absI < 5.0f) vw = 0.15f;
    else                  vw = 0.05f;

    float cw = 1.0f - vw;
    int   soc = (int)lround((float)coulombSoc * cw + (float)voltageSoc * vw);
    return _clamp(soc, 0, 100);
}

// ──────────────────────────────────────────────────────────────────────────────
// Alarms
// ──────────────────────────────────────────────────────────────────────────────

uint16_t LTC2944_BMS::_makeAlarms(float v, float i, float t, bool sensorOk) const {
    uint16_t a = 0;
    if (!sensorOk)          a |= ALARM_SENSOR_FAIL;
    if (!_profile.valid)    a |= ALARM_NO_PROFILE;
    if (_profile.valid) {
        if (v < _profile.vEmpty)    a |= ALARM_UNDER_VOLTAGE;
        if (v > _profile.vFull)     a |= ALARM_OVER_VOLTAGE;
        if (v < _profile.vAbsLow)   a |= ALARM_ABS_UNDER_V;
        if (v > _profile.vAbsHigh)  a |= ALARM_ABS_OVER_V;
    }
    if (t < -10.0f || t > 60.0f)   a |= ALARM_TEMPERATURE;
    if (i < -5.5f)                  a |= ALARM_OVER_CURRENT;
    if (i >  3.0f)                  a |= ALARM_CHARGE_CURRENT;
    if (_profileMismatch)           a |= ALARM_PROFILE_MISMATCH;
    return a;
}

// ──────────────────────────────────────────────────────────────────────────────
// Disconnect detection
// ──────────────────────────────────────────────────────────────────────────────

bool LTC2944_BMS::_checkDisconnectOnReadFailure() {
    _i2cFailCount++;
    if (_i2cFailCount >= 2) {
        _i2cFailCount = 0;
        reinitChip();
    }
    return false;   // caller may override with PIN_NEW_BAT check if desired
}

bool LTC2944_BMS::_checkDisconnectOnMeasurement(float vbat, float currentA) {
    if (!_profile.valid) return false;

    bool sig = (vbat < _profile.vAbsLow) && (_absf(currentA) < 0.05f);
    if (sig) {
        _disconnectCandidateCount++;
        if (_disconnectCandidateCount >= 2) {
            _writeDisconnectFlag();
            _disconnectCandidateCount = 2;
            return true;
        }
    } else {
        _disconnectCandidateCount = 0;
    }
    return false;
}

void LTC2944_BMS::_writeDisconnectFlag() {
    EEPROM.update(EE_DISCONNECT, DISCONNECT_MARKER);
}

void LTC2944_BMS::_clearDisconnectFlag() {
    EEPROM.update(EE_DISCONNECT, 0x00);
}

bool LTC2944_BMS::_readDisconnectFlag() const {
    return EEPROM.read(EE_DISCONNECT) == DISCONNECT_MARKER;
}

// ──────────────────────────────────────────────────────────────────────────────
// Calibration state machine
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_setLoad(bool en) {
    digitalWrite(_pinLoadEn, en ? HIGH : LOW);
}

void LTC2944_BMS::_enterCalPhase(CalPhase phase) {
    _calPhase        = phase;
    _calPhaseStartMs = millis();

    switch (phase) {
        case CAL_ZERO:
            _zeroSamplesTaken  = 0;
            _zeroAccumRawDelta = 0;
            _setLoad(false);
            break;
        case CAL_WAIT:
            _setLoad(false);
            break;
        case CAL_DISCHARGE:
            _calRunStartMs = millis();
            _setLoad(true);
            break;
        default:
            _setLoad(false);
            break;
    }
}

void LTC2944_BMS::_updateCalibration(uint16_t rawCurr, uint16_t rawACR,
                                      float vbat, float tempC, float currentA) {
    if (!_calibrationMode) return;

    // Safety abort on over-temperature
    if ((_calPhase == CAL_DISCHARGE || _calPhase == CAL_CHARGE) && tempC > 55.0f) {
        _enterCalPhase(CAL_ABORT);
        return;
    }

    switch (_calPhase) {

        case CAL_ZERO: {
            int32_t delta = (int32_t)rawCurr - 32767;
            _zeroAccumRawDelta += delta;
            _zeroSamplesTaken++;
            if (_zeroSamplesTaken >= ZERO_SAMPLES) {
                float avgDelta = (float)_zeroAccumRawDelta / (float)_zeroSamplesTaken;
                float offsetA  = (0.064f / _shuntOhm) * (avgDelta / 32767.0f);
                _currentOffset_mA = offsetA * 1000.0f;
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
            if (_profile.valid && vbat <= _profile.vEmpty) {
                _saveCalEmpty(vbat, rawACR);
                _enterCalPhase(CAL_CHARGE);
            } else if ((uint32_t)(millis() - _calRunStartMs) > MAX_CAL_TIME_MS) {
                _enterCalPhase(CAL_ABORT);
            }
            break;

        case CAL_CHARGE:
            if (_profile.valid
                    && vbat    >= (_profile.vFull * CAL_FULL_VOLT_PCT)
                    && currentA >= 0.0f
                    && currentA <= CAL_FULL_CURRENT_A) {
                _saveCalFull(vbat, rawACR);
                _enterCalPhase(CAL_DONE);
            } else if ((uint32_t)(millis() - _calPhaseStartMs) > MAX_CAL_TIME_MS) {
                _enterCalPhase(CAL_ABORT);
            }
            break;

        default:
            break;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// EEPROM – current offset
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_loadOffset() {
    uint32_t magic = 0;
    EEPROM.get(EE_MAGIC, magic);
    if (magic == EEPROM_MAGIC) {
        EEPROM.get(EE_OFFSET, _currentOffset_mA);
        if (!isfinite(_currentOffset_mA) || _absf(_currentOffset_mA) > 500.0f)
            _currentOffset_mA = 0.0f;
    } else {
        _currentOffset_mA = 0.0f;
    }
}

void LTC2944_BMS::_saveOffset(float offset_mA) {
    EEPROM.put(EE_MAGIC, EEPROM_MAGIC);
    EEPROM.put(EE_OFFSET, offset_mA);
}

// ──────────────────────────────────────────────────────────────────────────────
// EEPROM – calibration
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_loadCalibration() {
    _cal = {};
    uint32_t magic = 0;
    EEPROM.get(EE_CAL_MAGIC, magic);
    if (magic != CAL_MAGIC) return;

    EEPROM.get(EE_CAL_EMPTY_V, _cal.vEmpty);
    EEPROM.get(EE_CAL_FULL_V,  _cal.vFull);
    _cal.valid = true;

    uint16_t emMagic = 0;
    EEPROM.get(EE_CAL_EMPTY_MAGIC, emMagic);
    if (emMagic == EMPTY_ACR_MAGIC) {
        EEPROM.get(EE_CAL_EMPTY_RAW, _cal.emptyChargeRaw);
        _cal.hasEmptyRaw = true;
    }

    uint16_t fuMagic = 0;
    EEPROM.get(EE_CAL_FULL_MAGIC, fuMagic);
    if (fuMagic == FULL_ACR_MAGIC) {
        EEPROM.get(EE_CAL_FULL_RAW, _cal.fullChargeRaw);
        _cal.hasFullRaw = true;
    }
}

void LTC2944_BMS::_saveCalEmpty(float v, uint16_t raw) {
    _cal.vEmpty       = v;
    _cal.emptyChargeRaw = raw;
    _cal.hasEmptyRaw  = true;
    _cal.valid        = true;
    EEPROM.put(EE_CAL_MAGIC,       CAL_MAGIC);
    EEPROM.put(EE_CAL_EMPTY_V,     v);
    EEPROM.put(EE_CAL_EMPTY_RAW,   raw);
    EEPROM.put(EE_CAL_EMPTY_MAGIC, EMPTY_ACR_MAGIC);
}

void LTC2944_BMS::_saveCalFull(float v, uint16_t raw) {
    _cal.vFull        = v;
    _cal.fullChargeRaw = raw;
    _cal.hasFullRaw   = true;
    _cal.valid        = true;
    EEPROM.put(EE_CAL_MAGIC,      CAL_MAGIC);
    EEPROM.put(EE_CAL_FULL_V,     v);
    EEPROM.put(EE_CAL_FULL_RAW,   raw);
    EEPROM.put(EE_CAL_FULL_MAGIC, FULL_ACR_MAGIC);
}

// ──────────────────────────────────────────────────────────────────────────────
// Learned resistance model
// ──────────────────────────────────────────────────────────────────────────────

void LTC2944_BMS::_resetLearnedModel(bool /*newBattery*/) {
    _model.resistance           = 0.08f;
    _model.confidence           = 0.0f;
    _model.valid                = false;
    _model.dirty                = true;
    _model.samples              = 0;
    _model.lastRelaxedVoltage   = 0.0f;
    _model.lastSeenRawACR       = 0;
    _model.storedChem           = (uint8_t)_chem;
    _model.storedClassCode      = _classCode;
}

bool LTC2944_BMS::_loadLearnedModel() {
    LearnedRecord a = {}, b = {};
    EEPROM.get(EE_LEARNED_SLOT_A, a);
    EEPROM.get(EE_LEARNED_SLOT_B, b);

    bool validA = _isRecordValid(a);
    bool validB = _isRecordValid(b);
    if (!validA && !validB) { _resetLearnedModel(false); return false; }

    const LearnedRecord* newest = nullptr;
    if (validA && validB) newest = (a.sequence >= b.sequence) ? &a : &b;
    else                  newest = validA ? &a : &b;

    if (newest->storedChem != (uint8_t)_chem ||
        newest->storedClassCode != _classCode) {
        _resetLearnedModel(false);
        return false;
    }

    _model.resistance         = newest->resistance;
    _model.confidence         = newest->confidence;
    _model.valid              = (newest->confidence > 0.0f);
    _model.dirty              = false;
    _model.samples            = newest->samples;
    _model.lastRelaxedVoltage = newest->lastRelaxedVoltage;
    _model.lastSeenRawACR     = newest->lastSeenRawACR;
    _model.storedChem         = newest->storedChem;
    _model.storedClassCode    = newest->storedClassCode;
    return true;
}

bool LTC2944_BMS::_saveLearnedModel(bool force) {
    if (!force) {
        if (!_model.dirty) return false;
        if ((uint32_t)(millis() - _lastLearnedSaveMs) < LEARNED_SAVE_INTERVAL_MS) return false;
    }

    LearnedRecord a = {}, b = {};
    EEPROM.get(EE_LEARNED_SLOT_A, a);
    EEPROM.get(EE_LEARNED_SLOT_B, b);

    uint32_t seqA   = _isRecordValid(a) ? a.sequence : 0;
    uint32_t seqB   = _isRecordValid(b) ? b.sequence : 0;
    uint32_t newSeq = max(seqA, seqB) + 1UL;

    LearnedRecord out = {};
    out.magic              = LEARNED_MAGIC;
    out.version            = LEARNED_VERSION;
    out.length             = sizeof(LearnedRecord);
    out.sequence           = newSeq;
    out.resistance         = _model.resistance;
    out.confidence         = _model.confidence;
    out.samples            = _model.samples;
    out.lastRelaxedVoltage = _model.lastRelaxedVoltage;
    out.lastSeenRawACR     = _model.lastSeenRawACR;
    out.storedChem         = (uint8_t)_chem;
    out.storedClassCode    = _classCode;
    out.crc                = _recordCrc(out);

    if (seqA <= seqB) EEPROM.put(EE_LEARNED_SLOT_A, out);
    else              EEPROM.put(EE_LEARNED_SLOT_B, out);

    _model.dirty        = false;
    _lastLearnedSaveMs  = millis();
    return true;
}

void LTC2944_BMS::_updateResistanceLearning(float v, float i, uint32_t now) {
    const float minDeltaI   = 0.50f;
    const float minDeltaV   = 0.02f;
    const float maxR        = 0.50f;
    const float alpha       = 0.10f;
    const uint32_t window   = 3000UL;

    if (!_resLearn.baselineValid) {
        if (_currentNearZero(i)) {
            _resLearn.baselineV    = v;
            _resLearn.baselineI    = i;
            _resLearn.baselineMs   = now;
            _resLearn.baselineValid = true;
        }
        return;
    }

    if ((uint32_t)(now - _resLearn.baselineMs) < window) {
        if (_currentNearZero(i)) {
            _resLearn.baselineV = v;
            _resLearn.baselineI = i;
            _resLearn.baselineMs = now;
        }
        return;
    }

    float dI = i - _resLearn.baselineI;
    float dV = _resLearn.baselineV - v;

    if (_absf(dI) >= minDeltaI && _absf(dV) >= minDeltaV) {
        float R = dV / _absf(dI);
        if (R >= 0.0f && R <= maxR) {
            if (!_model.valid)
                _model.resistance = R;
            else
                _model.resistance = (1.0f - alpha) * _model.resistance + alpha * R;
            _model.samples++;
            _model.confidence = _clampf((float)_model.samples / 12.0f, 0.0f, 1.0f);
            _model.valid      = true;
            _model.dirty      = true;
        }
        _resLearn.baselineValid = false;
        return;
    }

    if (_currentNearZero(i)) {
        _resLearn.baselineV    = v;
        _resLearn.baselineI    = i;
        _resLearn.baselineMs   = now;
        _resLearn.baselineValid = true;
    } else if (_absf(i - _resLearn.baselineI) > 0.20f) {
        _resLearn.baselineValid = false;
    }
}

void LTC2944_BMS::_updateRelaxedSignature(float v, float i, uint16_t rawACR, uint32_t now) {
    if (!_currentNearZero(i)) return;
    if (_model.lastRelaxedVoltage <= 0.01f)
        _model.lastRelaxedVoltage = v;
    else
        _model.lastRelaxedVoltage = 0.95f * _model.lastRelaxedVoltage + 0.05f * v;
    _model.lastSeenRawACR   = rawACR;
    _lastRelaxedSeenMs       = now;
    _model.dirty             = true;
}

float LTC2944_BMS::_effectiveResistance() const {
    if (_model.valid && _model.confidence >= 0.10f)
        return _model.resistance;
    return 0.08f;
}

// ──────────────────────────────────────────────────────────────────────────────
// CRC-8 Dallas/Maxim
// ──────────────────────────────────────────────────────────────────────────────

uint8_t LTC2944_BMS::_crc8(const uint8_t* data, size_t len) const {
    uint8_t crc = 0x00;
    while (len--) {
        uint8_t b = *data++;
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t mix = (crc ^ b) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            b >>= 1;
        }
    }
    return crc;
}

uint8_t LTC2944_BMS::_recordCrc(const LearnedRecord& r) const {
    return _crc8((const uint8_t*)&r, sizeof(LearnedRecord) - 1);
}

bool LTC2944_BMS::_isRecordValid(const LearnedRecord& r) const {
    if (r.magic   != LEARNED_MAGIC)     return false;
    if (r.version != LEARNED_VERSION)   return false;
    if (r.length  != sizeof(LearnedRecord)) return false;
    if (_recordCrc(r) != r.crc)         return false;
    if (!isfinite(r.resistance) || r.resistance < 0.0f || r.resistance > 0.50f) return false;
    if (!isfinite(r.confidence) || r.confidence < 0.0f || r.confidence > 1.0f)  return false;
    if (!isfinite(r.lastRelaxedVoltage) || r.lastRelaxedVoltage < 0.0f) return false;
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Static math helpers
// ──────────────────────────────────────────────────────────────────────────────

int   LTC2944_BMS::_clamp(int v, int lo, int hi) {
    return v < lo ? lo : v > hi ? hi : v;
}
float LTC2944_BMS::_clampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}
float LTC2944_BMS::_absf(float v) {
    return v >= 0.0f ? v : -v;
}
bool LTC2944_BMS::_currentNearZero(float i) const {
    return _absf(i) < 0.15f;
}
