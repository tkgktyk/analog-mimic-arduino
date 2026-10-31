/**
 ******************************************************************************
 * @file    AnalogMimic.h
 * @author  TAKAGI Katsuyuki
 * @brief   Arduino library for controlling the Analog Mimic module over I2C.
 *-----------------------------------------------------------------------------
 * Copyright (C) 2026 TAKAGI Katsuyuki
 *
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 ******************************************************************************
 *
 * Units (the same in every function):
 *   voltage V (float), time us, frequency Hz, slew rate V/us, gain and Q dimensionless.
 *   Voltages are converted to the module's codes with the supply voltage given to the
 *   constructor. "Code" functions take or return raw_fx3 (ADC / DAC code x 8).
 *
 * Times and frequencies refer to the module's sampling rate of 200 kHz. Decimation
 * (setDecimation(N)) slows the module down by 2^N: every time becomes 2^N times longer
 * and every frequency 2^N times lower.
 */

#ifndef ANALOG_MIMIC_H
#define ANALOG_MIMIC_H

#include <Arduino.h>
#include <Wire.h>

// Include shared register definitions (Single Source of Truth)
#include "mimic_registers.h"

// Supply voltage in V (or pass the measured value, e.g. 4.98f)
constexpr float MIMIC_VDD_3V3 = 3.3f;
constexpr float MIMIC_VDD_5V  = 5.0f;

// Accepted supply voltage range of begin(); a value outside (e.g. 5000) is rejected
constexpr float MIMIC_VDD_MIN = 1.0f;
constexpr float MIMIC_VDD_MAX = 6.0f;

// Sampling rate of the module (times and frequencies refer to this rate)
constexpr float MIMIC_SAMPLING_HZ = (float)MIMIC_ADC_SAMPLING_HZ;

// =========================================================
// Filter Constants (Q-Factors)
// =========================================================
constexpr float MIMIC_FILTER_Q_BUTTERWORTH = 0.7071f;
constexpr float MIMIC_FILTER_Q_BESSEL      = 0.5773f;
constexpr float MIMIC_FILTER_Q_CHEBYSHEV   = 1.0f;

// Filter limits (values outside are clamped)
constexpr float MIMIC_FILTER_FC_MIN_HZ = MIMIC_SAMPLING_HZ * 1.0e-5f;  // 2 Hz
constexpr float MIMIC_FILTER_FC_MAX_HZ = MIMIC_SAMPLING_HZ * 0.24f;    // 48 kHz
constexpr float MIMIC_BIQUAD_Q_MIN     = 0.1f;
constexpr float MIMIC_BIQUAD_Q_MAX     = 10.0f;

// Delay line length
constexpr uint16_t MIMIC_DELAY_SAMPLES_MAX = 4095;

// =========================================================
// MimicBase (Core interface for all variants)
// =========================================================
/**
 * @brief Core interface for the Analog Mimic module.
 *        Converts physical units (V, us, Hz) to the module's fixed-point registers.
 */
class MimicBase
{
public:
  virtual ~MimicBase() = default;

  /**
   * @brief Initializes the library and the module's global settings
   *        (output flags, offset, decimation, Vref = VDD / 2).
   * @param wire Pointer to the TwoWire (I2C) instance. Defaults to &Wire.
   * @return false if the supply voltage given to the constructor is outside
   *         MIMIC_VDD_MIN - MIMIC_VDD_MAX (e.g. given in mV), or the module does not answer.
   */
  bool begin(TwoWire *wire = &Wire);

  // --- Unit conversion ---

  /** @brief Supply voltage given to the constructor, in V. */
  float getVdd() const { return _vdd_v; }

  /**
   * @brief Converts a voltage to a code (raw_fx3: 1 code = VDD / 32768).
   * @param volts Voltage in V. The result is rounded and limited to -32768 - 32767.
   */
  int16_t voltsToCode(float volts) const;

  /** @brief Converts a code (raw_fx3) to a voltage in V. */
  float codeToVolts(int32_t code) const;

  // --- Read Section ---

  /** @brief Reads an 8-bit register (for debugging). */
  uint8_t getOneByte(uint8_t addr);

  /** @brief Reads a 16-bit register (for debugging). */
  uint16_t getTwoBytes(uint8_t addr);

  /** @brief Reads the status flags (MIMIC_STATUS_*); error flags are cleared by the read. */
  uint8_t getStatus();

  /**
   * @brief Reads the DSP input: the latest ADC sample, or the average of 2^N samples
   *        when decimated.
   * @param seq If not null, receives the sequence number of the value (counts updates,
   *            wraps at 256), read together with the value.
   * @return Code (raw_fx3: ADC code x 8, 0 - 32760).
   */
  uint16_t getAdcValue(uint8_t *seq = nullptr);

  /** @brief Reads the DSP input as a voltage in V (see getAdcValue()). */
  float getInputVolts();

  /** @brief Reads the peak CPU cycles of the sampling ISR since the last read (360 per sample). */
  uint16_t getCpuCycles();

  /** @brief Reads the firmware build date as hex 0xYYYYMMDD (print with HEX). */
  uint32_t getBuildDate();

  /**
   * @brief Reads the 32-bit unit ID (FNV-1a hash of the MCU unique ID).
   * @return Unit ID. Fixed for each chip; different chips give different values
   *         with high probability (not guaranteed unique).
   */
  uint32_t getUnitId();

  // --- Global settings ---

  /** @brief Disconnects the output (high impedance) when true. */
  void setOutputOpen(bool openEnabled);

  /** @brief Inverts the output (VDD - out) when true. */
  void setOutputInverted(bool inverted);

  /**
   * @brief Slows the module down by 2^N: the processing rate becomes 200 kHz / 2^N
   *        (input averaged, output interpolated).
   *        Every time of the current mode becomes 2^N times longer and every frequency
   *        2^N times lower; the settings themselves are not recalculated.
   * @param N 0 (no decimation) to MIMIC_DECIMATION_N_MAX (16); larger values are clamped.
   */
  void setDecimation(uint8_t N);

  /**
   * @brief Adds a DC offset to the output.
   * @param offsetV Offset in V (can be negative). Resolution: VDD / 32768.
   */
  void setOutputOffset(float offsetV);

  /**
   * @brief Sets the common reference voltage (Vref), the signal ground of the module.
   * @param vrefV Reference voltage in V (0 to VDD). Defaults to VDD / 2 after begin().
   * @note  Every mode with a signal center works around Vref: 1st filters, Biquad, PGA,
   *        envelope, derivative, integral and rectifiers. Vref is an input: changing it
   *        does not reset the filter states (a slow HPF moves to the new Vref gradually).
   */
  void setVref(float vrefV);

  /** @brief Sets Vref as a code (raw_fx3, 0 - 32767), e.g. for calibration. */
  void setVrefCode(int16_t vrefCode);

  // --- 0. Bypass ---

  /** @brief Passes the input to the output (voltage follower). */
  void setBypassDSP();

  // --- 1. Filters (fc: MIMIC_FILTER_FC_MIN_HZ - MIMIC_FILTER_FC_MAX_HZ) ---

  /** @brief 1st-order low-pass filter (RC filter). @param fcHz Cutoff frequency in Hz. */
  void setFilter1stLPF(float fcHz);

  /** @brief 1st-order high-pass filter (CR filter). @param fcHz Cutoff frequency in Hz. */
  void setFilter1stHPF(float fcHz);

  /**
   * @brief 1st-order all-pass filter (phase shifter).
   *        Gain 1 at every frequency; the phase goes from 0 to -180 degrees, -90 at fc.
   * @param fcHz Frequency of the -90 degree phase shift in Hz.
   */
  void setFilter1stAPF(float fcHz);

  /**
   * @brief Biquad that passes the input unchanged with the same CPU load as the other
   *        Biquad filters (for profiling).
   */
  void setFilterBiquadUnity();

  /**
   * @brief 2nd-order low-pass filter.
   * @param fcHz Cutoff frequency in Hz.
   * @param Q    Quality factor (MIMIC_BIQUAD_Q_MIN - MIMIC_BIQUAD_Q_MAX). Default: Butterworth.
   */
  void setFilterBiquadLPF(float fcHz, float Q = MIMIC_FILTER_Q_BUTTERWORTH);

  /** @brief 2nd-order high-pass filter. @param fcHz Cutoff frequency in Hz. @param Q Quality factor. */
  void setFilterBiquadHPF(float fcHz, float Q = MIMIC_FILTER_Q_BUTTERWORTH);

  /** @brief 2nd-order band-pass filter, 0 dB at fc. @param fcHz Center frequency in Hz. @param Q Quality factor. */
  void setFilterBiquadBPF(float fcHz, float Q = 1.0f);

  /** @brief 2nd-order notch (band-stop) filter. @param fcHz Center frequency in Hz. @param Q Quality factor. */
  void setFilterBiquadNotch(float fcHz, float Q = 1.0f);

  /**
   * @brief 2nd-order all-pass filter (phase shifter).
   *        Gain 1 at every frequency; the phase goes from 0 to -360 degrees, -180 at fc.
   * @param fcHz Center frequency in Hz.
   * @param Q    Quality factor (steepness of the phase change).
   */
  void setFilterBiquadAPF(float fcHz, float Q = MIMIC_FILTER_Q_BUTTERWORTH);

  // --- 2. Amplifier ---

  /**
   * @brief Programmable gain amplifier: out = Vref + gain * (in - Vref).
   * @param gain Gain (negative inverts), |gain| < 32768.
   */
  void setPGA(float gain);

  // --- 3. Logarithmic / antilogarithmic amplifier ---

  /**
   * @brief Logarithmic amplifier: out = Vref + slope * log10(in / unit).
   * @param slopeVPerDecade Output change per decade of the input in V (negative = inverting).
   * @param unitV Input voltage (from 0 V) at which the output equals Vref.
   * @note  The input is a magnitude measured from 0 V; inputs near 0 V saturate low.
   */
  void setLog(float slopeVPerDecade = 1.0f, float unitV = 0.5f);

  /**
   * @brief Antilogarithmic (exponential) amplifier: out = unit * 10^((in - Vref) / slope).
   * @param slopeVPerDecade Input change per decade of the output in V (same as setLog()).
   * @param unitV Output voltage (from 0 V) when the input equals Vref.
   * @note  With the same Vref, slope and unit, setAntilog() is the inverse of setLog().
   */
  void setAntilog(float slopeVPerDecade = 1.0f, float unitV = 0.5f);

  // --- 4. Comparators (digital output) ---

  /**
   * @brief Window comparator: HIGH when the input is above upper or below lower.
   * @param upperV Upper threshold in V. @param lowerV Lower threshold in V.
   */
  void setComparatorWindow(float upperV, float lowerV);

  /**
   * @brief Schmitt trigger: HIGH above upper, LOW below lower, otherwise unchanged.
   * @param upperV Upper threshold in V. @param lowerV Lower threshold in V.
   */
  void setComparatorSchmitt(float upperV, float lowerV);

  // --- 5. Envelope, peak hold, sample & hold ---

  /**
   * @brief Envelope follower (AM detector): follows the peak above Vref (positive) or
   *        below Vref (negative) and decays toward Vref.
   * @param decayTimeUs Decay time constant in us (RC of the detector).
   * @param polarity MIMIC_POLARITY_POSITIVE or MIMIC_POLARITY_NEGATIVE.
   */
  void setEnvelopeFollower(float decayTimeUs, uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /** @brief Peak hold (envelope follower without decay). @param polarity MIMIC_POLARITY_*. */
  void setPeakHold(uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /** @brief Sample and hold. @param freqHz Sampling frequency in Hz. */
  void setSampleAndHold(float freqHz);

  /**
   * @brief Track and hold.
   * @param freqHz Frequency of the track / hold cycle in Hz.
   * @param trackRatio Part of the period spent tracking (0.0 - 1.0).
   */
  void setTrackAndHold(float freqHz, float trackRatio = 0.5f);

  // --- 6. Clippers ---

  /**
   * @brief Hard clipper: passes the input between the limits and clips outside.
   * @param upperV Upper limit in V. @param lowerV Lower limit in V.
   */
  void setClipper(float upperV, float lowerV);

  /**
   * @brief Soft clipper (smooth saturation, like a diode or transistor limiter).
   *        Gain 1 at the center of the limits; the output reaches a limit smoothly when the
   *        input is 1.5 times the center-to-limit distance away from the center.
   * @param upperV Upper output limit in V. @param lowerV Lower output limit in V.
   */
  void setSoftClipper(float upperV, float lowerV);

  // --- 7. Differentiator / integrator, rectifiers, slew rate, delay ---

  /**
   * @brief Differentiator: out = Vref + tau * d(in)/dt.
   * @param timeConstantUs Time constant tau (RC of the differentiator) in us.
   */
  void setMathDerivative(float timeConstantUs);

  /**
   * @brief Leaky integrator (integrator with a resistor across the capacitor).
   * @param leakTimeConstantUs Time constant of the leak in us; 0 gives a pure integrator.
   * @param inputShift Input scale 2^inputShift (<= 1): every sample (5 us) the output moves
   *        by (in - Vref) * 2^inputShift, i.e. an integration time of 5 us * 2^-inputShift.
   */
  void setMathIntegral(float leakTimeConstantUs, int8_t inputShift = -2);

  /**
   * @brief Half-wave rectifier folding at Vref.
   * @param polarity MIMIC_POLARITY_POSITIVE: max(in, Vref), MIMIC_POLARITY_NEGATIVE: min(in, Vref).
   */
  void setRectifierHalf(uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /**
   * @brief Full-wave rectifier (absolute value around Vref).
   * @param polarity MIMIC_POLARITY_POSITIVE: Vref + |in - Vref|, MIMIC_POLARITY_NEGATIVE: Vref - |in - Vref|.
   */
  void setRectifierFull(uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /**
   * @brief Slew rate limiter, like the slew rate of an op-amp.
   * @param slewVPerUs Maximum rate of change of the output in V/us. The smallest step is
   *        VDD / 32768 per sample (about 0.03 V/ms at 5 V); 0 freezes the output.
   */
  void setSlewRateLimiter(float slewVPerUs);

  /**
   * @brief Delay line (dead time).
   * @param delaySamples Delay in samples (0 - MIMIC_DELAY_SAMPLES_MAX); 1 sample = 5 us.
   */
  void setDigitalDelay(uint16_t delaySamples);

  // --- Calibration (output gain and offset, stored in the module's flash) ---

  /**
   * @brief Sets the output gain calibration (RAM; use commitCalibrationToNvm() to store).
   * @param gain Gain multiplier (e.g., 1.0023), 0 to < 2.0.
   */
  void setGainCal(float gain);

  /**
   * @brief Sets the output offset calibration (RAM; use commitCalibrationToNvm() to store).
   * @param offsetLsb Offset in output LSB (VDD / 4096, e.g. -6.44); resolution 1/8 LSB.
   */
  void setOffsetCal(float offsetLsb);

  /** @brief Stores the calibration in RAM to the module's flash. */
  void commitCalibrationToNvm();

  /** @brief Reloads the calibration from the module's flash. */
  void reloadCalibrationFromNvm();

protected:
  MimicBase(float vddV, uint8_t addressOffset = 0);

  uint8_t _i2c_addr;
  TwoWire *_wire;
  float _vdd_v;

  // State retention variables
  uint8_t _global_flags;
  int16_t _offset_code;      // User output offset (raw_fx3)
  uint8_t _decimation_N;

  uint8_t readRegister8(uint8_t regAddr);
  uint16_t readRegister16(uint8_t regAddr);
  uint32_t readRegister32(uint8_t regAddr);

  void writeModeAndPayload(uint8_t mode, const uint8_t *payload, uint8_t length);
  void writeOneByte(uint8_t addr, uint8_t value);
  void writeTwoBytes(uint8_t addr, uint16_t value);

  void sendFilter1st(uint8_t type, float fcHz);
  void sendBiquad(uint8_t type, float fcHz, float Q);
  void sendLog(uint8_t type, float slopeVPerDecade, float unitV);
  void sendComparator(uint8_t type, float upperV, float lowerV);
  void sendClipper(uint8_t type, float upperV, float lowerV);
};

// =========================================================
// Variant Classes
// =========================================================

/**
 * @brief Standard variant of Analog Mimic (mimic0x).
 * @param vddV Module supply voltage in V (MIMIC_VDD_5V, MIMIC_VDD_3V3 or a measured value).
 * @param addressOffset I2C address offset set by the A1 / A0 jumpers (0 - 3: 0x40 - 0x43).
 */
class Mimic0x : public MimicBase
{
public:
  explicit Mimic0x(float vddV, uint8_t addressOffset = 0) : MimicBase(vddV, addressOffset) {}
};

/**
 * @brief Extended AFE variant of Analog Mimic (mimic1x).
 * Features an external OPA and DC servo capabilities for advanced sensor interfacing.
 */
class Mimic1x : public MimicBase
{
public:
  explicit Mimic1x(float vddV, uint8_t addressOffset = 0) : MimicBase(vddV, addressOffset) {}

  /**
   * @brief Enables or disables the hardware DC servo function.
   * @note Only available on mimic1x variants with the secondary internal DAC routed to the AFE.
   * @param enabled True to enable the hardware DC servo.
   */
  void setHardwareServoEnabled(bool enabled);
};

#endif // ANALOG_MIMIC_H
