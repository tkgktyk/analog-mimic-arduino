/**
 ******************************************************************************
 * @file    AnalogMimic.cpp
 * @author  TAKAGI Katsuyuki
 * @brief   Implementation of Host-Side Helper Functions, I2C Command Packet 
 * Serialization, and Register-Mapping Wrapper APIs.
 *-----------------------------------------------------------------------------
 * Copyright (C) 2026 TAKAGI Katsuyuki
 *
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 ******************************************************************************
 */

#include "AnalogMimic.h"
#include <math.h>

namespace {
    // Constants for I2C communication delays (in microseconds)
    constexpr unsigned long I2C_WRITE_DELAY_US = 100;
}

// =========================================================
// Internal Helper Functions
// =========================================================

// Splits a value into mant (|mant| < 1.0, sent as q0_15) and exp: value = mant * 2^exp
static void calculateMantExp(float inputVal, float* outMant, int8_t* outExp) {
    *outExp = 0;
    *outMant = inputVal;
    while (fabs(*outMant) >= 1.0f && *outExp < 16) {
        *outMant /= 2.0f;
        (*outExp)++;
    }
}

static int16_t floatToQ0_15(float value) {
    float s = roundf(value * 32768.0f);
    if (s >  32767.0f) s =  32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
}

// Big-endian 16-bit payload field
static void put16(uint8_t *buffer, uint8_t offset, int32_t value) {
    buffer[offset + 0] = (uint8_t)((value >> 8) & 0xFF);
    buffer[offset + 1] = (uint8_t)(value & 0xFF);
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Splits a positive coefficient into a 12-bit mantissa (2048 - 4095) and a shift:
// value * 2^(12 - baseShift) = mant >> (shift - baseShift)   (see mimic_registers.h)
static void toMant12Shift(float value, uint8_t baseShift, uint16_t *mant, uint8_t *shift,
                          uint8_t maxShift = 30) {
    int n = 0;
    while (value > 0.0f && value < 0.5f && baseShift + n < maxShift) {
        value *= 2.0f;
        n++;
    }
    long m = (long)roundf(value * 4096.0f);
    if (m > 4095) m = 4095;
    if (m < 0) m = 0;
    *mant  = (uint16_t)m;
    *shift = (uint8_t)(baseShift + n);
}

// 1 - exp(-1 / (fs * tau)): the per-sample decay of a time constant, accurate for long tau
// (1 - exp(-x) loses precision in float when x is small)
static float decayComplement(float tauUs) {
    float x = 1.0e6f / (MIMIC_SAMPLING_HZ * tauUs);
    if (x < 1.0e-2f) return x * (1.0f - x * (0.5f - x / 6.0f));
    return 1.0f - (float)exp(-x);
}

// Number of samples of a period (at least 1, at most 65535)
static uint16_t periodSamples(float freqHz) {
    if (freqHz <= 0.0f) return 65535;
    float n = MIMIC_SAMPLING_HZ / freqHz;
    if (n < 1.0f) n = 1.0f;
    if (n > 65535.0f) n = 65535.0f;
    return (uint16_t)n;
}

// =========================================================
// MimicBase Implementation
// =========================================================

MimicBase::MimicBase(float vddV, uint8_t addressOffset) :
    _i2c_addr(MIMIC_DEFAULT_I2C_ADDR + (addressOffset & 0x03)),
    _wire(nullptr),
    _vdd_v(vddV),
    _global_flags(0),
    _offset_code(0),
    _decimation_N(0) {}

bool MimicBase::begin(TwoWire *wire) {
    _wire = wire;
    // A supply voltage in mV (e.g. 5000) would scale every voltage by 1/1000
    if (!(_vdd_v >= MIMIC_VDD_MIN && _vdd_v <= MIMIC_VDD_MAX)) return false;
    if (readRegister8(MIMIC_REG_SYSTEM_ID) != MIMIC_DEVICE_ID_VALUE) return false;

    _global_flags = 0;
    _offset_code = 0;
    _decimation_N = 0;
    writeOneByte(MIMIC_REG_GLOBAL_FLAGS, _global_flags);
    writeOneByte(MIMIC_REG_GLOBAL_DECIMATION_N, _decimation_N);
    writeTwoBytes(MIMIC_REG_GLOBAL_OFFSET, 0);
    writeTwoBytes(MIMIC_REG_GLOBAL_VREF, (uint16_t)MIMIC_VREF_DEFAULT_RAW_FX3);  // VDD / 2
    writeOneByte(MIMIC_REG_GLOBAL_TEST_PAD, MIMIC_TEST_PAD_OFF);
    return true;
}

// --- Unit conversion ---

int16_t MimicBase::voltsToCode(float volts) const {
    float code = roundf(volts * (float)((MIMIC_ADC_MAX_VALUE + 1) << MIMIC_RAW_FX3_FRAC_BITS) / _vdd_v);
    return (int16_t)clampf(code, -32768.0f, 32767.0f);
}

float MimicBase::codeToVolts(int32_t code) const {
    return (float)code * _vdd_v / (float)((MIMIC_ADC_MAX_VALUE + 1) << MIMIC_RAW_FX3_FRAC_BITS);
}

// --- Internal I2C Communication Helpers ---

uint8_t MimicBase::readRegister8(uint8_t regAddr) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(regAddr);
    if (_wire->endTransmission() != 0) return 0xFF;

    _wire->requestFrom(_i2c_addr, (uint8_t)1);
    if (_wire->available() != 1) return 0xFF;
    return (uint8_t)_wire->read();
}

uint16_t MimicBase::readRegister16(uint8_t regAddr) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(regAddr);
    if (_wire->endTransmission() != 0) return 0xFFFF;

    _wire->requestFrom(_i2c_addr, (uint8_t)2);
    if (_wire->available() != 2) return 0xFFFF;
    uint8_t h = (uint8_t)_wire->read();
    uint8_t l = (uint8_t)_wire->read();
    return (uint16_t)((h << 8) | l);
}

// Reads a constant 32-bit value as two 16-bit reads (high word first)
uint32_t MimicBase::readRegister32(uint8_t regAddr) {
    uint16_t h = readRegister16(regAddr);
    uint16_t l = readRegister16(regAddr + 2);
    return ((uint32_t)h << 16) | l;
}

void MimicBase::writeOneByte(uint8_t addr, uint8_t value) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(addr);
    _wire->write(value);
    _wire->endTransmission();

    // Wait briefly for the module to apply the change (e.g. stop / open the output)
    delayMicroseconds(I2C_WRITE_DELAY_US);
}

void MimicBase::writeTwoBytes(uint8_t addr, uint16_t value) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(addr);
    _wire->write((value >> 8) & 0xFF);
    _wire->write(value & 0xFF);
    _wire->endTransmission();
    delayMicroseconds(I2C_WRITE_DELAY_US);
}

void MimicBase::writeModeAndPayload(uint8_t mode, const uint8_t *payload, uint8_t length) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(MIMIC_REG_MODE_ID);   // Mode ID, then the payload (auto-increment)
    _wire->write(mode);
    for (uint8_t i = 0; i < length && payload != nullptr; i++) {
        _wire->write(payload[i]);
    }
    _wire->endTransmission();
    delayMicroseconds(I2C_WRITE_DELAY_US);
}

// --- Public Register Accessors ---

uint8_t MimicBase::getOneByte(uint8_t addr) {
    return readRegister8(addr);
}

uint16_t MimicBase::getTwoBytes(uint8_t addr) {
    return readRegister16(addr);
}

uint8_t MimicBase::getStatus() {
    return readRegister8(MIMIC_REG_SYSTEM_STATUS);
}

uint16_t MimicBase::getAdcValue(uint8_t *seq) {
    // SEQ, VAL_H, VAL_L in one transaction: the SEQ read latches all three
    _wire->beginTransmission(_i2c_addr);
    _wire->write(MIMIC_REG_SYSTEM_ADC_SEQ);
    if (_wire->endTransmission() != 0) return 0xFFFF;

    _wire->requestFrom(_i2c_addr, (uint8_t)3);
    if (_wire->available() != 3) return 0xFFFF;
    uint8_t s = (uint8_t)_wire->read();
    uint8_t h = (uint8_t)_wire->read();
    uint8_t l = (uint8_t)_wire->read();
    if (seq != nullptr) *seq = s;
    return (uint16_t)((h << 8) | l);
}

float MimicBase::getInputVolts() {
    return codeToVolts(getAdcValue());
}

uint16_t MimicBase::getCpuCycles() {
    return readRegister16(MIMIC_REG_SYSTEM_CPU_CYCLES);
}

uint32_t MimicBase::getBuildDate() {
    return readRegister32(MIMIC_REG_SYSTEM_BUILD_DATE);
}

uint32_t MimicBase::getUnitId() {
    return readRegister32(MIMIC_REG_SYSTEM_UID);
}

// --- Global settings ---

void MimicBase::setOutputOpen(bool openEnabled) {
    if (openEnabled) _global_flags |= MIMIC_FLAG_OUT_OPEN;
    else _global_flags &= ~MIMIC_FLAG_OUT_OPEN;
    writeOneByte(MIMIC_REG_GLOBAL_FLAGS, _global_flags);
}

void MimicBase::setOutputInverted(bool inverted) {
    if (inverted) _global_flags |= MIMIC_FLAG_INV_OUT;
    else _global_flags &= ~MIMIC_FLAG_INV_OUT;
    writeOneByte(MIMIC_REG_GLOBAL_FLAGS, _global_flags);
}

void MimicBase::setDecimation(uint8_t N) {
    if (N > MIMIC_DECIMATION_N_MAX) N = MIMIC_DECIMATION_N_MAX;
    _decimation_N = N;
    writeOneByte(MIMIC_REG_GLOBAL_DECIMATION_N, _decimation_N);
}

void MimicBase::setOutputOffset(float offsetV) {
    _offset_code = voltsToCode(offsetV);
    writeTwoBytes(MIMIC_REG_GLOBAL_OFFSET, (uint16_t)_offset_code);
}

void MimicBase::setVref(float vrefV) {
    setVrefCode(voltsToCode(vrefV));
}

void MimicBase::setVrefCode(int16_t vrefCode) {
    if (vrefCode < 0) vrefCode = 0;
    writeTwoBytes(MIMIC_REG_GLOBAL_VREF, (uint16_t)vrefCode);
}

// -------------------------------------------------------------
// Modes
// -------------------------------------------------------------

void MimicBase::setBypassDSP() {
    writeModeAndPayload(MIMIC_MODE_ID_BYPASS_DSP, nullptr, 0);
}

// --- 1st-order filters ---

// TPT one-pole filter (bilinear transform, same response as the analog RC filter at fc):
// g = tan(pi fc / fs), G = g / (1 + g)
void MimicBase::sendFilter1st(uint8_t type, float fcHz) {
    fcHz = clampf(fcHz, MIMIC_FILTER_FC_MIN_HZ, MIMIC_FILTER_FC_MAX_HZ);
    float g = (float)tan(PI * fcHz / MIMIC_SAMPLING_HZ);
    uint16_t gMant;
    uint8_t  gShift;
    toMant12Shift(g / (1.0f + g), 0, &gMant, &gShift);

    uint8_t payload[4];
    payload[0] = type;
    put16(payload, 1, gMant);
    payload[3] = gShift;
    writeModeAndPayload(MIMIC_MODE_ID_FILTER_1ST, payload, 4);
}

void MimicBase::setFilter1stLPF(float fcHz) { sendFilter1st(MIMIC_FILTER_1ST_LPF, fcHz); }
void MimicBase::setFilter1stHPF(float fcHz) { sendFilter1st(MIMIC_FILTER_1ST_HPF, fcHz); }
void MimicBase::setFilter1stAPF(float fcHz) { sendFilter1st(MIMIC_FILTER_1ST_APF, fcHz); }

// --- Biquad filters ---

// TPT state-variable filter coefficients (bilinear transform, same response as the
// Audio EQ Cookbook biquads): g = tan(pi fc / fs), k = 1 / Q
void MimicBase::sendBiquad(uint8_t type, float fcHz, float Q) {
    fcHz = clampf(fcHz, MIMIC_FILTER_FC_MIN_HZ, MIMIC_FILTER_FC_MAX_HZ);
    Q    = clampf(Q, MIMIC_BIQUAD_Q_MIN, MIMIC_BIQUAD_Q_MAX);

    float g = (float)tan(PI * fcHz / MIMIC_SAMPLING_HZ);
    float k = 1.0f / Q;
    float e = (g * k + g * g) / (1.0f + g * k + g * g);   // 1 - 1 / (1 + g k + g^2)

    uint16_t gMant, eMant;
    uint8_t  gShift, eShift;
    toMant12Shift(g, 0, &gMant, &gShift);
    toMant12Shift(e, 12, &eMant, &eShift);

    uint8_t payload[11];
    payload[0] = type;
    put16(payload, 1, gMant);
    payload[3] = gShift;
    put16(payload, 4, (int32_t)roundf((k + g) * 1024.0f));   // 10 fractional bits
    put16(payload, 6, eMant);
    payload[8] = eShift;
    put16(payload, 9, (int32_t)roundf(k * 2048.0f));         // 11 fractional bits
    writeModeAndPayload(MIMIC_MODE_ID_FILTER_BIQUAD, payload, 11);
}

void MimicBase::setFilterBiquadUnity() {
    sendBiquad(MIMIC_FILTER_BIQUAD_UNITY, 1000.0f, MIMIC_FILTER_Q_BUTTERWORTH);
}

void MimicBase::setFilterBiquadLPF(float fcHz, float Q)   { sendBiquad(MIMIC_FILTER_BIQUAD_LPF, fcHz, Q); }
void MimicBase::setFilterBiquadHPF(float fcHz, float Q)   { sendBiquad(MIMIC_FILTER_BIQUAD_HPF, fcHz, Q); }
void MimicBase::setFilterBiquadBPF(float fcHz, float Q)   { sendBiquad(MIMIC_FILTER_BIQUAD_BPF, fcHz, Q); }
void MimicBase::setFilterBiquadNotch(float fcHz, float Q) { sendBiquad(MIMIC_FILTER_BIQUAD_NOTCH, fcHz, Q); }
void MimicBase::setFilterBiquadAPF(float fcHz, float Q)   { sendBiquad(MIMIC_FILTER_BIQUAD_APF, fcHz, Q); }

// --- PGA ---

void MimicBase::setPGA(float gain) {
    float mant;
    int8_t exp;
    calculateMantExp(gain, &mant, &exp);

    uint8_t payload[3];
    put16(payload, 0, floatToQ0_15(mant));
    payload[2] = (uint8_t)exp;
    writeModeAndPayload(MIMIC_MODE_ID_PGA, payload, 3);
}

// --- Logarithmic / antilogarithmic ---

void MimicBase::sendLog(uint8_t type, float slopeVPerDecade, float unitV) {
    int16_t unit = voltsToCode(unitV);
    if (unit < 1) unit = 1;
    int16_t slope = voltsToCode(slopeVPerDecade);
    if (slope == 0) slope = 1;

    uint8_t payload[5];
    payload[0] = type;
    put16(payload, 1, slope);
    put16(payload, 3, unit);
    writeModeAndPayload(MIMIC_MODE_ID_LOG, payload, 5);
}

void MimicBase::setLog(float slopeVPerDecade, float unitV)     { sendLog(MIMIC_LOG_TYPE_LOG, slopeVPerDecade, unitV); }
void MimicBase::setAntilog(float slopeVPerDecade, float unitV) { sendLog(MIMIC_LOG_TYPE_ANTILOG, slopeVPerDecade, unitV); }

// --- Comparators ---

void MimicBase::sendComparator(uint8_t type, float upperV, float lowerV) {
    uint8_t payload[5];
    payload[0] = type;
    put16(payload, 1, voltsToCode(upperV));
    put16(payload, 3, voltsToCode(lowerV));
    writeModeAndPayload(MIMIC_MODE_ID_COMPARATOR, payload, 5);
}

void MimicBase::setComparatorWindow(float upperV, float lowerV)  { sendComparator(MIMIC_COMPARATOR_WINDOW, upperV, lowerV); }
void MimicBase::setComparatorSchmitt(float upperV, float lowerV) { sendComparator(MIMIC_COMPARATOR_SCHMITT, upperV, lowerV); }

// --- Envelope, peak hold, sample & hold ---

// Payload: [0] polarity, [1-2] 1 - decay mant12, [3] 1 - decay shift (0 = peak hold)
void MimicBase::setEnvelopeFollower(float decayTimeUs, uint8_t polarity) {
    float c = (decayTimeUs <= 0.0f) ? 1.0f : decayComplement(decayTimeUs);   // <= 0: no hold
    uint16_t mant;
    uint8_t  shift;
    toMant12Shift(c, 12, &mant, &shift, 40);

    uint8_t payload[4];
    payload[0] = polarity;
    put16(payload, 1, mant);
    payload[3] = shift;
    writeModeAndPayload(MIMIC_MODE_ID_ENVELOPE_FOLLOWER, payload, 4);
}

void MimicBase::setPeakHold(uint8_t polarity) {
    uint8_t payload[4] = { polarity, 0, 0, 12 };   // 1 - decay = 0: holds the peak
    writeModeAndPayload(MIMIC_MODE_ID_ENVELOPE_FOLLOWER, payload, 4);
}

void MimicBase::setSampleAndHold(float freqHz) {
    uint8_t payload[4];
    put16(payload, 0, periodSamples(freqHz));
    put16(payload, 2, 1);                          // track for 1 sample: sample & hold
    writeModeAndPayload(MIMIC_MODE_ID_SAMPLE_AND_HOLD, payload, 4);
}

void MimicBase::setTrackAndHold(float freqHz, float trackRatio) {
    uint16_t period = periodSamples(freqHz);
    uint16_t track  = (uint16_t)(period * clampf(trackRatio, 0.0f, 1.0f));
    if (track == 0) track = 1;                     // always track at least 1 sample

    uint8_t payload[4];
    put16(payload, 0, period);
    put16(payload, 2, track);
    writeModeAndPayload(MIMIC_MODE_ID_SAMPLE_AND_HOLD, payload, 4);
}

// --- Clippers ---

void MimicBase::sendClipper(uint8_t type, float upperV, float lowerV) {
    uint8_t payload[5];
    payload[0] = type;
    put16(payload, 1, voltsToCode(upperV));
    put16(payload, 3, voltsToCode(lowerV));
    writeModeAndPayload(MIMIC_MODE_ID_CLIPPER, payload, 5);
}

void MimicBase::setClipper(float upperV, float lowerV)     { sendClipper(MIMIC_CLIPPER_HARD, upperV, lowerV); }
void MimicBase::setSoftClipper(float upperV, float lowerV) { sendClipper(MIMIC_CLIPPER_SOFT, upperV, lowerV); }
void MimicBase::setConstantVoltage(float volts)            { sendClipper(MIMIC_CLIPPER_HARD, volts, volts); }
void MimicBase::setDeadZone(float upperV, float lowerV)    { sendClipper(MIMIC_CLIPPER_DEAD_ZONE, upperV, lowerV); }

// --- Integrator / differentiator ---

// value = mant / 2^15 * 2^exp with mant in [2^14, 2^15) (value > 0)
static void toQ15MantExp(float value, int16_t *mant, int8_t *exp) {
    int e = 0;
    if (value > 0.0f) {
        while (value >= 1.0f && e < 60)  { value *= 0.5f; e++; }
        while (value < 0.5f  && e > -60) { value *= 2.0f; e--; }
    }
    long m = (long)roundf(value * 32768.0f);
    if (m > 32767) m = 32767;
    if (m < 0) m = 0;
    *mant = (int16_t)m;
    *exp  = (int8_t)e;
}

// Payload: [0] type, [1-2] K mant q0_15, [3] K exp, [4-5] c mant12, [6] c shift (0: none).
// Per sample the module computes m += c * (K * u - m) (u = in - Vref, or its difference
// for the differentiator); with c = 0 the integrator adds K * u.
void MimicBase::sendIntegrator(uint8_t type, float k, float c) {
    int16_t  kMant;
    int8_t   kExp;
    uint16_t cMant  = 0;
    uint8_t  cShift = 0;
    toQ15MantExp(k, &kMant, &kExp);
    if (c > 0.0f) toMant12Shift(c, 12, &cMant, &cShift, 32);

    uint8_t payload[7];
    payload[0] = type;
    put16(payload, 1, kMant);
    payload[3] = (uint8_t)kExp;
    put16(payload, 4, cMant);
    payload[6] = cShift;
    writeModeAndPayload(MIMIC_MODE_ID_INTEGRATOR, payload, 7);
}

void MimicBase::setIntegrator(float integrationTimeUs, float leakTimeUs) {
    integrationTimeUs = clampf(integrationTimeUs, MIMIC_INTEGRATOR_TIME_MIN_US, MIMIC_INTEGRATOR_TIME_MAX_US);
    float g = 1.0e6f / (MIMIC_SAMPLING_HZ * integrationTimeUs);   // Ts / tau_i (<= 1)
    if (leakTimeUs > 0.0f) {
        leakTimeUs = clampf(leakTimeUs, MIMIC_INTEGRATOR_TIME_MIN_US, MIMIC_INTEGRATOR_TIME_MAX_US);
        float c = decayComplement(leakTimeUs);
        sendIntegrator(MIMIC_INTEGRATOR_INTEGRATOR, g / c, c);    // K = DC gain
    } else {
        sendIntegrator(MIMIC_INTEGRATOR_INTEGRATOR, g, 0.0f);     // no leak
    }
}

void MimicBase::setDifferentiator(float differentiationTimeUs, float limitTimeUs) {
    differentiationTimeUs = clampf(differentiationTimeUs, 0.0f, MIMIC_DIFFERENTIATOR_TIME_MAX_US);
    float k = differentiationTimeUs * 1.0e-6f * MIMIC_SAMPLING_HZ;  // tau_d * fs
    float c = 0.0f;                                                 // 0: no limit (c = 1)
    if (limitTimeUs > 0.0f) {
        c = decayComplement(clampf(limitTimeUs, 0.0f, MIMIC_DIFFERENTIATOR_TIME_MAX_US));
    }
    sendIntegrator(MIMIC_INTEGRATOR_DIFFERENTIATOR, k, c);
}

// --- Multiplier: squarer, square rooter ---

// Payload: [0] type, [1-2] unit raw_fx3 (> 0)
void MimicBase::sendMultiplier(uint8_t type, float unitV) {
    if (!(unitV > 0.0f)) unitV = _vdd_v * 0.25f;   // default: VDD / 4
    int16_t unit = voltsToCode(unitV);
    if (unit < 1) unit = 1;

    uint8_t payload[3];
    payload[0] = type;
    put16(payload, 1, unit);
    writeModeAndPayload(MIMIC_MODE_ID_MULTIPLIER, payload, 3);
}

void MimicBase::setSquarer(float unitV)      { sendMultiplier(MIMIC_MULTIPLIER_SQUARER, unitV); }
void MimicBase::setSquareRooter(float unitV) { sendMultiplier(MIMIC_MULTIPLIER_SQUARE_ROOTER, unitV); }

// --- True RMS ---

// Payload: [0-1] 1 - a mant12, [2] 1 - a shift (13 - 32: 1 - a < 0.5)
void MimicBase::setTrueRms(float averagingTimeUs) {
    averagingTimeUs = clampf(averagingTimeUs, MIMIC_TRUE_RMS_TIME_MIN_US, MIMIC_TRUE_RMS_TIME_MAX_US);
    float c = decayComplement(averagingTimeUs);
    if (c > 0.49f) c = 0.49f;
    uint16_t mant;
    uint8_t  shift;
    toMant12Shift(c, 12, &mant, &shift, 32);

    uint8_t payload[3];
    put16(payload, 0, mant);
    payload[2] = shift;
    writeModeAndPayload(MIMIC_MODE_ID_TRUE_RMS, payload, 3);
}

// --- Rectifiers ---

void MimicBase::setRectifierHalf(uint8_t polarity) {
    uint8_t payload[2] = { MIMIC_RECT_HALF, polarity };
    writeModeAndPayload(MIMIC_MODE_ID_RECTIFIER, payload, 2);
}

void MimicBase::setRectifierFull(uint8_t polarity) {
    uint8_t payload[2] = { MIMIC_RECT_FULL, polarity };
    writeModeAndPayload(MIMIC_MODE_ID_RECTIFIER, payload, 2);
}

// --- Slew rate limiter, delay ---

// Payload: [0-1] maximum change per sample in codes (raw_fx3)
void MimicBase::setSlewRateLimiter(float slewVPerUs) {
    float voltsPerSample = slewVPerUs * 1.0e6f / MIMIC_SAMPLING_HZ;
    float step = roundf(voltsPerSample * (float)((MIMIC_ADC_MAX_VALUE + 1) << MIMIC_RAW_FX3_FRAC_BITS) / _vdd_v);
    if (slewVPerUs > 0.0f && step < 1.0f) step = 1.0f;   // the slowest rate, not a frozen output
    step = clampf(step, 0.0f, 32767.0f);

    uint8_t payload[2];
    put16(payload, 0, (int32_t)step);
    writeModeAndPayload(MIMIC_MODE_ID_SLEW_RATE_LIMITER, payload, 2);
}

void MimicBase::setDigitalDelay(uint16_t delaySamples) {
    if (delaySamples > MIMIC_DELAY_SAMPLES_MAX) delaySamples = MIMIC_DELAY_SAMPLES_MAX;
    uint8_t payload[2];
    put16(payload, 0, delaySamples);
    writeModeAndPayload(MIMIC_MODE_ID_DELAY, payload, 2);
}

// --- Oscillator ---

// Payload: [0] type, [1-4] phase increment (f = inc * fs / 2^32), [5-6] amplitude raw_fx3,
// [7-8] VCO gain mant int16, [9] VCO gain exp int8: phase increment per raw_fx3 of input
// Phase increment per Hz of the oscillator (f = inc * fs / 2^32)
static const float OSC_INC_PER_HZ = 4294967296.0f / MIMIC_SAMPLING_HZ;

// k of the VCO / VF: phase increment per raw_fx3 of the input
float MimicBase::oscFmGain(float hzPerVolt) const {
    return hzPerVolt * OSC_INC_PER_HZ * _vdd_v / (float)((MIMIC_ADC_MAX_VALUE + 1) << MIMIC_RAW_FX3_FRAC_BITS);
}

// The DDS fields shared by the oscillator, modulator, PWM modulator and V/F converter:
// phase increment for freqHz, and k = mant * 2^exp (|mant| in 16384 - 32767 when possible)
static uint32_t oscIncrement(float freqHz) {
    float inc = roundf(clampf(freqHz, 0.0f, MIMIC_OSC_FREQ_MAX_HZ) * OSC_INC_PER_HZ);
    if (inc > 2147483647.0f) inc = 2147483647.0f;
    return (uint32_t)inc;
}
static void oscMantExp(float k, int16_t *mant, int8_t *exp) {
    int8_t e = 0;
    while (fabs(k) > 32767.0f && e < 4) { k *= 0.5f; e++; }
    while (k != 0.0f && fabs(k) < 16384.0f && e > -16) { k *= 2.0f; e--; }
    *mant = (int16_t)clampf(roundf(k), -32768.0f, 32767.0f);
    *exp  = e;
}

// Oscillator, modulator, PWM: [0] waveform, [1-4] inc, [5-6] amplitude, [7-8] k mant, [9] k exp.
// k: VCO: phase increment per raw_fx3 (oscFmGain()); modulator: gain; PWM: not used.
// Always all 10 bytes: the payload registers keep the bytes of the previous mode.
void MimicBase::sendOscillator(uint8_t mode, uint8_t waveform, float freqHz, float amplitudeV, float k) {
    uint32_t inc = oscIncrement(freqHz);
    int16_t amp = voltsToCode(amplitudeV);
    if (amp < 0) amp = 0;
    int16_t mant; int8_t exp;
    oscMantExp(k, &mant, &exp);

    uint8_t payload[10];
    payload[0] = waveform;
    put16(payload, 1, (int32_t)(inc >> 16));
    put16(payload, 3, (int32_t)(inc & 0xFFFF));
    put16(payload, 5, amp);
    put16(payload, 7, mant);
    payload[9] = (uint8_t)exp;
    writeModeAndPayload(mode, payload, 10);
}

void MimicBase::setOscillator(float freqHz, float amplitudeV, uint8_t waveform) {
    sendOscillator(MIMIC_MODE_ID_OSCILLATOR, waveform, freqHz, amplitudeV, 0.0f);   // k = 0: fixed
}

void MimicBase::setVco(float centerFreqHz, float hzPerVolt, float amplitudeV, uint8_t waveform) {
    sendOscillator(MIMIC_MODE_ID_OSCILLATOR, waveform, centerFreqHz, amplitudeV, oscFmGain(hzPerVolt));
}

void MimicBase::setAmModulator(float carrierHz, float carrierV, float depthPerV, uint8_t carrierWaveform) {
    // A (1 + m (in - Vref)) = A + (A m) (in - Vref): k = A m, dimensionless
    sendOscillator(MIMIC_MODE_ID_MODULATOR, carrierWaveform, carrierHz, carrierV, carrierV * depthPerV);
}

void MimicBase::setRingModulator(float carrierHz, float gain, uint8_t carrierWaveform) {
    sendOscillator(MIMIC_MODE_ID_MODULATOR, carrierWaveform, carrierHz, 0.0f, gain);   // A = 0: DSB
}

void MimicBase::setPwmModulator(float carrierHz, float fullScaleV, uint8_t carrierWaveform) {
    sendOscillator(MIMIC_MODE_ID_PWM_MODULATOR, carrierWaveform, carrierHz, fullScaleV, 0.0f);
}

// V/F converter: [0-3] inc (f0), [4-5] k mant, [6] k exp
void MimicBase::setVfConverter(float hzPerVolt, float zeroFreqHz) {
    uint32_t inc = oscIncrement(zeroFreqHz);
    int16_t mant; int8_t exp;
    oscMantExp(oscFmGain(hzPerVolt), &mant, &exp);
    uint8_t payload[7];
    put16(payload, 0, (int32_t)(inc >> 16));
    put16(payload, 2, (int32_t)(inc & 0xFFFF));
    put16(payload, 4, mant);
    payload[6] = (uint8_t)exp;
    writeModeAndPayload(MIMIC_MODE_ID_VF_CONVERTER, payload, 7);
}

void MimicBase::setTestPadOutput(uint8_t source) {
    writeOneByte(MIMIC_REG_GLOBAL_TEST_PAD, source);
}

// Noise: [0] color, [1-2] amplitude
void MimicBase::setNoise(float amplitudeV, uint8_t color) {
    int16_t amp = voltsToCode(amplitudeV);
    if (amp < 0) amp = 0;
    uint8_t payload[3];
    payload[0] = color;
    put16(payload, 1, amp);
    writeModeAndPayload(MIMIC_MODE_ID_NOISE, payload, 3);
}

// -------------------------------------------------------------
// Calibration
// -------------------------------------------------------------

void MimicBase::setGainCal(float gain) {
    // uq1_15 (1.0 = 32768), rounded and limited to 0 - 65535
    float s = clampf(roundf(gain * 32768.0f), 0.0f, 65535.0f);
    writeTwoBytes(MIMIC_REG_NVM_GAIN, (uint16_t)s);
}

void MimicBase::setOffsetCal(float offsetLsb) {
    // raw_fx3 (1/8 LSB)
    float s = clampf(roundf(offsetLsb * (float)(1 << MIMIC_RAW_FX3_FRAC_BITS)), -32768.0f, 32767.0f);
    writeTwoBytes(MIMIC_REG_NVM_OFFSET, (uint16_t)(int16_t)s);
}

void MimicBase::commitCalibrationToNvm() {
    writeOneByte(MIMIC_REG_CMD_PORT, MIMIC_CMD_NVM_COMMIT);
}

void MimicBase::reloadCalibrationFromNvm() {
    writeOneByte(MIMIC_REG_CMD_PORT, MIMIC_CMD_NVM_RELOAD);
}

// =========================================================
// Mimic1x Implementation
// =========================================================

void Mimic1x::setHardwareServoEnabled(bool enabled) {
    if (enabled) _global_flags |= MIMIC_FLAG_SERVO_EN;
    else _global_flags &= ~MIMIC_FLAG_SERVO_EN;
    
    writeOneByte(MIMIC_REG_GLOBAL_FLAGS, _global_flags);
}
