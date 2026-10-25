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

static inline int16_t clamp16(int32_t val) {
    if (val > 32767) return 32767;
    if (val < -32768) return -32768;
    return (int16_t)val;
}

// =========================================================
// MimicBase Implementation
// =========================================================

MimicBase::MimicBase(uint16_t vddMv, uint8_t hardwareJumper) : 
    _i2c_addr(MIMIC_DEFAULT_I2C_ADDR + (hardwareJumper & 0x0F)),
    _wire(nullptr),
    _vdd_mv(vddMv),
    _default_fs_hz(MIMIC_ADC_SAMPLING_HZ),
    _fs_hz(MIMIC_ADC_SAMPLING_HZ),
    _global_flags(0),
    _offset_raw_fx3(0),
    _decimation_N(0) {}

bool MimicBase::begin(TwoWire *wire) {
    _wire = wire;
    _global_flags = 0;
    _offset_raw_fx3 = 0;
    _decimation_N = 0;
    writeOneByte(MIMIC_REG_GLOBAL_FLAGS, _global_flags);
    writeTwoBytes(MIMIC_REG_GLOBAL_VREF, (uint16_t)MIMIC_VREF_DEFAULT_RAW_FX3);  // VDD / 2
    return true;
}

// --- Internal I2C Communication Helpers (DRY Implementation) ---

uint8_t MimicBase::readRegister8(uint8_t regAddr) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(regAddr);
    if (_wire->endTransmission() != 0) return 0xFF; 

    uint8_t received = _wire->requestFrom(_i2c_addr, (uint8_t)1);
    if (_wire->available()) {
         return _wire->read();
    } else {
        Serial.print("[I2C Rx Error! Requested 1, Got ");
        Serial.print(received);
        Serial.println("]");
        return 0xFF; 
    }
}

uint16_t MimicBase::readRegister16(uint8_t regAddr) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(regAddr); 
    if (_wire->endTransmission() != 0) return 0xFFFF; 
  
    uint8_t received = _wire->requestFrom(_i2c_addr, (uint8_t)2); 
    if (_wire->available() == 2) {
        uint8_t h = _wire->read();
        uint8_t l = _wire->read();
        return (h << 8) | l;
    } else {
        return 0xFFFF; 
    }
}

// Reads a constant 32-bit value as two 16-bit reads (high word first)
uint32_t MimicBase::readRegister32(uint8_t regAddr) {
    uint16_t h = readRegister16(regAddr);
    uint16_t l = readRegister16(regAddr + 2);
    return ((uint32_t)h << 16) | l;
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

uint16_t MimicBase::getAdcValue() {
    return readRegister16(MIMIC_REG_SYSTEM_ADC_VAL);
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

// --- Common Registers & Flags ---

void MimicBase::writeOneByte(uint8_t addr, uint8_t value) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(addr);
    _wire->write(value);
    _wire->endTransmission();
    
    // Wait briefly for PY32 to detect flag differences and stop/open the OPA
    delayMicroseconds(I2C_WRITE_DELAY_US);
}

void MimicBase::writeTwoBytes(uint8_t addr, uint16_t value) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(addr);
    _wire->write((value >> 8) & 0xFF);
    _wire->write(value & 0xFF);
    _wire->endTransmission();
    
    // Added delay for timing safety
    delayMicroseconds(I2C_WRITE_DELAY_US);
}

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
    _fs_hz = _default_fs_hz / pow(2, N);

    writeOneByte(MIMIC_REG_GLOBAL_DECIMATION_N, _decimation_N);
}

void MimicBase::setOutputOffset(int16_t offsetMv) {
    _offset_raw_fx3 = mvToRawFx3(offsetMv);

    writeTwoBytes(MIMIC_REG_GLOBAL_OFFSET, (uint16_t)_offset_raw_fx3);
}

void MimicBase::setVref(uint16_t vrefMv) {
    int16_t vref = mvToRawFx3(vrefMv);
    if (vref < 0) vref = 0;

    writeTwoBytes(MIMIC_REG_GLOBAL_VREF, (uint16_t)vref);
}

void MimicBase::writeModeAndPayload(uint8_t mode, const uint8_t *payload, uint8_t length) {
    _wire->beginTransmission(_i2c_addr);
    
    // Specify the target base address
    _wire->write(MIMIC_REG_MODE_ID); 
    // Write the mode ID to the base address. Subsequent bytes will auto-increment.
    _wire->write(mode);                  
    
    // Transmit the payload in a single burst (MCU is idling, so buffer overflow will not occur)
    if (length > 0 && payload != nullptr) {
        for (uint8_t i = 0; i < length; i++) {
            _wire->write(payload[i]);           
        }
    }
    _wire->endTransmission();
    delayMicroseconds(I2C_WRITE_DELAY_US); 
}

// --- Conversion Utilities ---

// mV -> raw_fx3 (1 LSB = VDD / 32768), rounded and clamped to int16
int16_t MimicBase::mvToRawFx3(int32_t mv) {
    int32_t num  = mv * (int32_t)((MIMIC_ADC_MAX_VALUE + 1) << MIMIC_RAW_FX3_FRAC_BITS);
    int32_t half = (int32_t)_vdd_mv / 2;
    int32_t v    = (num >= 0) ? (num + half) / (int32_t)_vdd_mv : -((-num + half) / (int32_t)_vdd_mv);
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

int16_t MimicBase::floatToQ0_15(float value) {
    float s = roundf(value * 32768.0f);
    if (s >  32767.0f) s =  32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
}

void MimicBase::floatToPayloadQ0_15(float value, uint8_t* buffer, uint8_t offset) {
    int16_t v = floatToQ0_15(value);
    buffer[offset + 0] = (v >> 8) & 0xFF; // MSB
    buffer[offset + 1] = v & 0xFF;        // LSB
}

void MimicBase::rawToPayload16(uint16_t raw, uint8_t* buffer, uint8_t offset) {
    buffer[offset + 0] = (raw >> 8) & 0xFF;  // MSB
    buffer[offset + 1] = raw & 0xFF;         // LSB
}

void MimicBase::int16ToPayload(int32_t value, uint8_t* buffer, uint8_t offset) {
    // Safely clip to -32768 to +32767 in case of any overflow during correction calculations
    int16_t q15 = clamp16(value); 
    
    buffer[offset + 0] = (q15 >> 8) & 0xFF; // MSB
    buffer[offset + 1] = q15 & 0xFF;        // LSB
}

// -------------------------------------------------------------
// Functional Mode APIs
// -------------------------------------------------------------

void MimicBase::setBypassDSP() {
    writeModeAndPayload(MIMIC_MODE_ID_BYPASS_DSP, nullptr, 0);
}

// --- 1st Order Filters ---

void MimicBase::setFilter1stLPF(float fc) {
    uint8_t payload[3];
    float alpha = 1.0f - exp(-2.0f * PI * fc / _fs_hz);
    payload[0] = MIMIC_FILTER_1ST_LPF;
    floatToPayloadQ0_15(alpha, payload, 1);
    writeModeAndPayload(MIMIC_MODE_ID_FILTER_1ST, payload, 3);
}

void MimicBase::setFilter1stHPF(float fc) {
    uint8_t payload[3];
    float alpha = 1.0f - exp(-2.0f * PI * fc / _fs_hz);
    payload[0] = MIMIC_FILTER_1ST_HPF;
    floatToPayloadQ0_15(alpha, payload, 1);
    writeModeAndPayload(MIMIC_MODE_ID_FILTER_1ST, payload, 3);
}

// --- Biquad Filters ---

// Splits a positive coefficient into a 12-bit mantissa (2048 - 4095) and a shift:
// value * 2^(12 - baseShift) = mant >> (shift - baseShift)   (see FILTER_BIQUAD in mimic_registers.h)
static void toMant12Shift(float value, uint8_t baseShift, uint16_t *mant, uint8_t *shift) {
    int n = 0;
    while (value > 0.0f && value < 0.5f && baseShift + n < 30) {
        value *= 2.0f;
        n++;
    }
    long m = (long)roundf(value * 4096.0f);
    if (m > 4095) m = 4095;
    if (m < 0) m = 0;
    *mant  = (uint16_t)m;
    *shift = (uint8_t)(baseShift + n);
}

// TPT state-variable filter coefficients (bilinear transform, same response as the
// Audio EQ Cookbook biquads): g = tan(pi fc / fs), k = 1 / Q
void MimicBase::sendBiquad(uint8_t type, float fc, float Q) {
    float fcMin = _fs_hz * MIMIC_BIQUAD_FC_MIN_RATIO;
    float fcMax = _fs_hz * MIMIC_BIQUAD_FC_MAX_RATIO;
    if (fc < fcMin) fc = fcMin;
    if (fc > fcMax) fc = fcMax;
    if (Q < MIMIC_BIQUAD_Q_MIN) Q = MIMIC_BIQUAD_Q_MIN;
    if (Q > MIMIC_BIQUAD_Q_MAX) Q = MIMIC_BIQUAD_Q_MAX;

    float g = (float)tan(PI * fc / _fs_hz);
    float k = 1.0f / Q;
    float e = (g * k + g * g) / (1.0f + g * k + g * g);   // 1 - 1 / (1 + g k + g^2)

    uint16_t gMant, eMant;
    uint8_t  gShift, eShift;
    toMant12Shift(g, 0, &gMant, &gShift);
    toMant12Shift(e, 12, &eMant, &eShift);
    uint16_t g1 = (uint16_t)roundf((k + g) * 1024.0f);   // 10 fractional bits
    uint16_t kq = (uint16_t)roundf(k * 2048.0f);         // 11 fractional bits

    uint8_t payload[11];
    payload[0]  = type;
    payload[1]  = (uint8_t)(gMant >> 8);
    payload[2]  = (uint8_t)(gMant & 0xFF);
    payload[3]  = gShift;
    payload[4]  = (uint8_t)(g1 >> 8);
    payload[5]  = (uint8_t)(g1 & 0xFF);
    payload[6]  = (uint8_t)(eMant >> 8);
    payload[7]  = (uint8_t)(eMant & 0xFF);
    payload[8]  = eShift;
    payload[9]  = (uint8_t)(kq >> 8);
    payload[10] = (uint8_t)(kq & 0xFF);

    writeModeAndPayload(MIMIC_MODE_ID_FILTER_BIQUAD, payload, 11);
}

/**
 * @brief Unity-Gain Biquad (Executes full computation path but passes signal through unchanged)
 * Configures coefficients so that the transfer function H(z) = 1.
 * Used to compare signal quality against bypass mode while maintaining the computation load (212 cycles).
 */
void MimicBase::setFilterBiquadUnity() {
    // Same processing as the other types (profiling), the input is passed through
    sendBiquad(MIMIC_FILTER_BIQUAD_UNITY, 1000.0f, MIMIC_FILTER_Q_BUTTERWORTH);
}

void MimicBase::setFilterBiquadLPF(float fc, float Q) {
    sendBiquad(MIMIC_FILTER_BIQUAD_LPF, fc, Q);
}

void MimicBase::setFilterBiquadHPF(float fc, float Q) {
    sendBiquad(MIMIC_FILTER_BIQUAD_HPF, fc, Q);
}

void MimicBase::setFilterBiquadBPF(float fc, float Q) {
    sendBiquad(MIMIC_FILTER_BIQUAD_BPF, fc, Q);
}

void MimicBase::setFilterBiquadNotch(float fc, float Q) {
    sendBiquad(MIMIC_FILTER_BIQUAD_NOTCH, fc, Q);
}

// --- PGA ---

void MimicBase::setPGA(float gain) {
    uint8_t payload[3];
    float mant;
    int8_t exp;
    
    calculateMantExp(gain, &mant, &exp);
    
    floatToPayloadQ0_15(mant, payload, 0);
    payload[2] = (uint8_t)exp;

    writeModeAndPayload(MIMIC_MODE_ID_PGA, payload, 3);
}

// --- Logarithmic / Antilogarithmic ---

void MimicBase::sendLog(uint8_t type, int16_t slopeMvPerDecade, uint16_t unitMv) {
    uint8_t payload[5];
    int16_t unit = mvToRawFx3(unitMv);
    if (unit < 1) unit = 1;
    payload[0] = type;
    int16ToPayload(mvToRawFx3(slopeMvPerDecade), payload, 1);
    int16ToPayload(unit, payload, 3);
    writeModeAndPayload(MIMIC_MODE_ID_LOG, payload, 5);
}

void MimicBase::setLog(int16_t slopeMvPerDecade, uint16_t unitMv) {
    sendLog(MIMIC_LOG_TYPE_LOG, slopeMvPerDecade, unitMv);
}

void MimicBase::setAntilog(int16_t slopeMvPerDecade, uint16_t unitMv) {
    sendLog(MIMIC_LOG_TYPE_ANTILOG, slopeMvPerDecade, unitMv);
}

// --- Comparators ---

void MimicBase::setComparatorWindow(uint16_t upperMv, uint16_t lowerMv) {
    uint8_t payload[5];
    payload[0] = MIMIC_COMPARATOR_WINDOW;
    int16ToPayload(mvToRawFx3(upperMv), payload, 1);
    int16ToPayload(mvToRawFx3(lowerMv), payload, 3);
    writeModeAndPayload(MIMIC_MODE_ID_COMPARATOR, payload, 5);
}

void MimicBase::setComparatorSchmitt(uint16_t upperMv, uint16_t lowerMv) {
    uint8_t payload[5];
    payload[0] = MIMIC_COMPARATOR_SCHMITT;
    int16ToPayload(mvToRawFx3(upperMv), payload, 1);
    int16ToPayload(mvToRawFx3(lowerMv), payload, 3);
    writeModeAndPayload(MIMIC_MODE_ID_COMPARATOR, payload, 5);
}

// --- Waveform Analysis ---

void MimicBase::setEnvelopeFollower(float decayTimeUs, uint8_t polarity) {
    uint8_t payload[3]; 
    float tau = decayTimeUs / 1000000.0f;
    float decayFactor = (tau <= 0.0f) ? 0.0f : exp(-1.0f / (_fs_hz * tau));
    
    floatToPayloadQ0_15(decayFactor, payload, 0);
    payload[2] = polarity;
    writeModeAndPayload(MIMIC_MODE_ID_ENVELOPE_FOLLOWER, payload, 3);
}

void MimicBase::setPeakHold(uint8_t polarity) {
    uint8_t payload[3];

    floatToPayloadQ0_15(1.0f, payload, 0);
    payload[2] = polarity;
    writeModeAndPayload(MIMIC_MODE_ID_ENVELOPE_FOLLOWER, payload, 3);
}

// --- Sample & Hold ---

void MimicBase::setSampleAndHold(float freqHz) {
    uint8_t payload[4];

    // Safety guard: Limit frequencies below zero or exceeding system Fs
    if (freqHz <= 0.0f) freqHz = 0.1f; 
    if (freqHz > _fs_hz) freqHz = _fs_hz;

    // Calculate processing interval (Period) from target frequency (Hz)
    uint16_t periodSamples = (uint16_t)(_fs_hz / freqHz);
    if (periodSamples == 0) periodSamples = 1;

    rawToPayload16(periodSamples, payload, 0);
    rawToPayload16(1, payload, 2); // Track width = 1 sample (S&H definition)

    writeModeAndPayload(MIMIC_MODE_ID_SAMPLE_AND_HOLD, payload, 4);
}

// --- Track & Hold ---

void MimicBase::setTrackAndHold(float freqHz, float trackRatio) {
    uint8_t payload[4];

    if (freqHz <= 0.0f) freqHz = 0.1f;
    if (freqHz > _fs_hz) freqHz = _fs_hz;

    uint16_t periodSamples = (uint16_t)(_fs_hz / freqHz);
    if (periodSamples == 0) periodSamples = 1;

    // Determine Track samples based on ratio (0.0 ~ 1.0)
    if (trackRatio < 0.0f) trackRatio = 0.0f;
    if (trackRatio > 1.0f) trackRatio = 1.0f;
    
    uint16_t trackSamples = (uint16_t)(periodSamples * trackRatio);
    if (trackSamples == 0) trackSamples = 1; // Always track at least 1 sample

    rawToPayload16(periodSamples, payload, 0);
    rawToPayload16(trackSamples, payload, 2);

    writeModeAndPayload(MIMIC_MODE_ID_SAMPLE_AND_HOLD, payload, 4);
}

// --- Clipper ---

void MimicBase::setClipper(uint16_t upperMv, uint16_t lowerMv) {
    uint8_t payload[4];
    int16ToPayload(mvToRawFx3(upperMv), payload, 0);
    int16ToPayload(mvToRawFx3(lowerMv), payload, 2);
    writeModeAndPayload(MIMIC_MODE_ID_CLIPPER, payload, 4);
}

// --- Calculus ---

void MimicBase::setMathDerivative(float timeConstantUs) {
    uint8_t payload[4];
    float tau = timeConstantUs / 1000000.0f;
    float scale = tau * _fs_hz; 
    
    float mant;
    int8_t exp;
    
    calculateMantExp(scale, &mant, &exp);
    
    payload[0] = MIMIC_MATH_DERIVATIVE;
    floatToPayloadQ0_15(mant, payload, 1);
    payload[3] = (uint8_t)exp;
    writeModeAndPayload(MIMIC_MODE_ID_MATH, payload, 4);
}

void MimicBase::setMathIntegral(float timeConstantUs, int8_t inputShift) {
    uint8_t payload[4];
    float tau = timeConstantUs / 1000000.0f;
    float leakFactor = (tau <= 0.0f) ? 1.0f : exp(-1.0f / (_fs_hz * tau));
    
    payload[0] = MIMIC_MATH_INTEGRAL;
    floatToPayloadQ0_15(leakFactor, payload, 1);
    payload[3] = (uint8_t)inputShift;
    
    writeModeAndPayload(MIMIC_MODE_ID_MATH, payload, 4);
}

void MimicBase::setRectifierHalf(uint8_t polarity) {
    uint8_t payload[2] = { MIMIC_RECT_HALF, polarity };
    writeModeAndPayload(MIMIC_MODE_ID_RECTIFIER, payload, 2);
}

void MimicBase::setRectifierFull(uint8_t polarity) {
    uint8_t payload[2] = { MIMIC_RECT_FULL, polarity };
    writeModeAndPayload(MIMIC_MODE_ID_RECTIFIER, payload, 2);
}

void MimicBase::setSlewRateLimiter(float maxChangePerSampleRatio) {
    uint8_t payload[2];
    // Max change per sample in raw_fx3 (1 LSB = 8), relative to full scale (4096 LSB)
    // E.g., 0.1 (10%) -> 409.6 * 8 = 3276.8 -> 3277
    if (maxChangePerSampleRatio < 0.0f) maxChangePerSampleRatio = 0.0f;
    float step = roundf(maxChangePerSampleRatio * (float)((MIMIC_ADC_MAX_VALUE + 1) << MIMIC_RAW_FX3_FRAC_BITS));
    int16ToPayload((int32_t)step, payload, 0);
    writeModeAndPayload(MIMIC_MODE_ID_SLEW_RATE_LIMITER, payload, 2);
}

void MimicBase::setDigitalDelay(uint16_t delaySamples) {
    uint8_t payload[2];
    rawToPayload16(delaySamples, payload, 0);
    writeModeAndPayload(MIMIC_MODE_ID_DELAY, payload, 2);
}

// -------------------------------------------------------------
// Calibration APIs
// -------------------------------------------------------------

/**
 * @brief Converts a floating-point gain multiplier to a Q15 unsigned integer and writes it to the NVM register.
 * @param gain Gain multiplier (e.g., 1.0023, 0.994).
 */
void MimicBase::setGainCal(float gain) {
    // Convert float (multiplier) to uq1_15 (1.0 = 32768), rounded and clamped to 0 - 65535
    float s = roundf(gain * 32768.0f);
    if (s < 0.0f)     s = 0.0f;
    if (s > 65535.0f) s = 65535.0f;

    writeTwoBytes(MIMIC_REG_NVM_GAIN, (uint16_t)s);
}

/**
 * @brief Writes the offset calibration to the NVM register (RAM) via I2C.
 * @param offsetLsb Offset in output LSB (e.g., -6.44), stored as raw_fx3 (1/8 LSB resolution).
 */
void MimicBase::setOffsetCal(float offsetLsb) {
    float s = roundf(offsetLsb * (float)(1 << MIMIC_RAW_FX3_FRAC_BITS));
    if (s >  32767.0f) s =  32767.0f;
    if (s < -32768.0f) s = -32768.0f;

    writeTwoBytes(MIMIC_REG_NVM_OFFSET, (uint16_t)(int16_t)s);
}

/**
 * @brief Commits the calibration parameters currently in RAM to the MCU's internal FLASH memory for permanent storage.
 */
void MimicBase::commitCalibrationToNvm(void) {
    // Issue the COMMIT command (0xA5) to the Command Port register
    writeOneByte(MIMIC_REG_CMD_PORT, MIMIC_CMD_NVM_COMMIT);
}

/**
 * @brief Discards current RAM calibration parameters and reloads the saved values from the MCU's internal FLASH memory.
 */
void MimicBase::reloadCalibrationFromNvm(void) {
    // Issue the RELOAD command (0x5A) to the Command Port register
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
