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

// Oscillator frequency limit (fs / 2)
constexpr float MIMIC_OSC_FREQ_MAX_HZ = MIMIC_SAMPLING_HZ * 0.5f;

// Time constant limits (values outside are clamped)
constexpr float MIMIC_TRUE_RMS_TIME_MIN_US       = 10.0f;     // 2 samples
constexpr float MIMIC_TRUE_RMS_TIME_MAX_US       = 10.0e6f;   // 10 s
constexpr float MIMIC_INTEGRATOR_TIME_MIN_US     = 5.0f;      // 1 sample
constexpr float MIMIC_INTEGRATOR_TIME_MAX_US     = 10.0e6f;   // 10 s (integration and leak)
constexpr float MIMIC_DIFFERENTIATOR_TIME_MAX_US = 10.0e6f;   // 10 s (differentiation and limit)

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
   * @note  Every input is taken relative to Vref: filters, PGA, envelope, rectifiers,
   *        derivative, integral, square, square root, RMS, log / antilog and the VCO.
   *        Vref is an input: changing it does not reset the filter states (a slow HPF
   *        moves to the new Vref gradually). When modules are chained, set the Vref of the
   *        next module to the output offset of the previous one.
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

  // --- 3. Comparators (digital output) ---

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

  // --- 4. Envelope follower and peak hold ---

  /**
   * @brief Envelope follower (AM detector): follows the peak above Vref (positive) or
   *        below Vref (negative) and decays toward Vref.
   * @param decayTimeUs Decay time constant in us (RC of the detector).
   * @param polarity MIMIC_POLARITY_POSITIVE or MIMIC_POLARITY_NEGATIVE.
   */
  void setEnvelopeFollower(float decayTimeUs, uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /** @brief Peak hold (envelope follower without decay). @param polarity MIMIC_POLARITY_*. */
  void setPeakHold(uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  // --- 5. Clippers ---

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

  /**
   * @brief Constant voltage source: the output stays at volts whatever the input (a clipper
   *        whose upper and lower limits are both volts). Vref is not changed, and the
   *        output calibration applies as for any signal.
   * @param volts Output voltage in V. The analog stage does not reach within about 0.1 V
   *        of 0 V and of VDD.
   */
  void setConstantVoltage(float volts);

  // --- 6. Integrator / differentiator, rectifiers ---

  /**
   * @brief Integrator, like an op-amp integrator (input R, feedback C):
   *        out = Vref + (1 / tau_i) * integral of (in - Vref) dt. A leak (a resistor across
   *        the capacitor) makes it a 1st-order low-pass with a DC gain of leak / integration
   *        time, and keeps it from drifting to the limit.
   * @param integrationTimeUs Integration time constant tau_i (R_in * C) in us: a constant
   *        input of 1 V moves the output by 1 V every tau_i
   *        (MIMIC_INTEGRATOR_TIME_MIN_US - MIMIC_INTEGRATOR_TIME_MAX_US).
   * @param leakTimeUs Leak time constant (R_leak * C) in us; 0 (default): no leak.
   */
  void setIntegrator(float integrationTimeUs, float leakTimeUs = 0.0f);

  /**
   * @brief Differentiator, like an op-amp differentiator (input C, feedback R) with a
   *        resistor in series with the capacitor: out = Vref + tau_d * d(in)/dt for changes
   *        slower than the limit time, and a gain of tau_d / limit above it (it does not
   *        amplify noise without bound).
   * @param differentiationTimeUs Differentiation time constant tau_d (R_f * C) in us
   *        (up to MIMIC_DIFFERENTIATOR_TIME_MAX_US).
   * @param limitTimeUs Band-limit time constant (R_series * C) in us; 10 us (default) is
   *        about 16 kHz. 0: no limit (the difference of two samples).
   */
  void setDifferentiator(float differentiationTimeUs, float limitTimeUs = 10.0f);

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

  // ===================================================================================
  // Experimental modes
  //   Implemented in the firmware, but not yet verified on hardware and in application
  //   examples. They are public in MimicExperimental<> (AnalogMimicExperimental.h);
  //   the API may change before they are released here.
  // ===================================================================================

  // --- Time: sample & hold, slew rate, delay ---

  /** @brief Sample and hold. @param freqHz Sampling frequency in Hz. */
  void setSampleAndHold(float freqHz);

  /**
   * @brief Track and hold.
   * @param freqHz Frequency of the track / hold cycle in Hz.
   * @param trackRatio Part of the period spent tracking (0.0 - 1.0).
   */
  void setTrackAndHold(float freqHz, float trackRatio = 0.5f);

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

  // --- Signal sources and modulators ---

  /**
   * @brief Oscillator (function generator): out = Vref + amplitude * wave(2 pi f t).
   * @param freqHz Frequency in Hz (0 - MIMIC_OSC_FREQ_MAX_HZ). 0 gives a constant Vref
   *        (a voltage source set by setVref()).
   * @param amplitudeV Peak amplitude in V.
   * @param waveform MIMIC_OSC_WAVE_SINE (default), _TRIANGLE, _SQUARE or _SAWTOOTH. Every
   *        waveform starts at Vref and rises; the square is high in the first half period.
   * @note  The phase continues when the frequency, the amplitude or the waveform is changed.
   *        The square and the sawtooth jump within one sample, so their harmonics above
   *        fs / 2 fold back (alias); this is small at low frequencies.
   */
  void setOscillator(float freqHz, float amplitudeV, uint8_t waveform = MIMIC_OSC_WAVE_SINE);

  /**
   * @brief Voltage-controlled oscillator (sine): f = f0 + k * (in - Vref), limited to
   *        0 - MIMIC_OSC_FREQ_MAX_HZ (linear FM; put an antilog module in front for 1 V/oct).
   * @param centerFreqHz Frequency f0 in Hz when the input equals Vref.
   * @param hzPerVolt Sensitivity k in Hz/V (negative: the frequency falls as the input rises).
   * @param amplitudeV Peak amplitude in V.
   * @param waveform MIMIC_OSC_WAVE_* (default: sine), as for setOscillator().
   */
  void setVco(float centerFreqHz, float hzPerVolt, float amplitudeV,
              uint8_t waveform = MIMIC_OSC_WAVE_SINE);

  /**
   * @brief AM modulator: out = Vref + carrierV * (1 + depth * (in - Vref)) * carrier(2 pi f t).
   *        The amplitude of the internal carrier follows the input, like a multiplier-based
   *        AM modulator (MC1496); below -100 % (1 + depth * (in - Vref) < 0) the carrier is
   *        inverted, as with a multiplier.
   * @param carrierHz Carrier frequency in Hz. @param carrierV Carrier amplitude without input in V.
   * @param depthPerV Modulation depth per volt of the input (1 / V): 1.0 gives 100 % at 1 V.
   * @param carrierWaveform MIMIC_OSC_WAVE_* (default: sine).
   */
  void setAmModulator(float carrierHz, float carrierV, float depthPerV,
                      uint8_t carrierWaveform = MIMIC_OSC_WAVE_SINE);

  /**
   * @brief Ring (balanced, DSB) modulator: out = Vref + gain * (in - Vref) * carrier(2 pi f t).
   *        The input times the internal carrier: the sum and difference frequencies, without
   *        the carrier. A square carrier gives a chopper (polarity switching) modulator.
   * @param carrierHz Carrier frequency in Hz. @param gain Gain at the carrier peak (default 1).
   * @param carrierWaveform MIMIC_OSC_WAVE_* (default: sine).
   */
  void setRingModulator(float carrierHz, float gain = 1.0f,
                        uint8_t carrierWaveform = MIMIC_OSC_WAVE_SINE);

  /**
   * @brief PWM modulator (comparator against an internal carrier), digital output: HIGH while
   *        the input is above the carrier. The duty cycle goes from 0 % at Vref - fullScaleV
   *        to 100 % at Vref + fullScaleV (50 % at Vref).
   * @param carrierHz PWM frequency in Hz. Each edge falls on a sample (5 us), so the duty
   *        resolution is about carrierHz / 200 kHz (0.5 % at 1 kHz).
   * @param fullScaleV Carrier amplitude in V. @param carrierWaveform MIMIC_OSC_WAVE_TRIANGLE
   *        (default: both edges move) or MIMIC_OSC_WAVE_SAWTOOTH (one edge fixed).
   */
  void setPwmModulator(float carrierHz, float fullScaleV,
                       uint8_t carrierWaveform = MIMIC_OSC_WAVE_TRIANGLE);

  /**
   * @brief V/F converter (LM331), digital output: a square wave (50 % duty) whose frequency
   *        is zeroFreqHz + hzPerVolt * (in - Vref), limited to 0 - MIMIC_OSC_FREQ_MAX_HZ.
   *        For a sensor whose zero is 0 V, call setVref(0.0f). Each edge falls on a sample,
   *        so a single period varies by +/-5 us; the average frequency is exact.
   * @param hzPerVolt Sensitivity in Hz/V. @param zeroFreqHz Frequency at Vref in Hz (default 0).
   */
  void setVfConverter(float hzPerVolt, float zeroFreqHz = 0.0f);

  /**
   * @brief Noise generator (input not used): out = Vref + noise.
   * @param amplitudeV MIMIC_NOISE_WHITE: uniform within +/-amplitudeV (rms amplitudeV / sqrt 3).
   *        MIMIC_NOISE_PINK: -3 dB/octave, rms amplitudeV / 3 (peaks beyond are clipped).
   * @param color MIMIC_NOISE_WHITE (default) or MIMIC_NOISE_PINK.
   * @note  At 200 kHz the pink spectrum holds from about 25 Hz to fs / 2 (Voss-McCartney,
   *        12 rows); decimation moves the band down by 2^N.
   */
  void setNoise(float amplitudeV, uint8_t color = MIMIC_NOISE_WHITE);

  // --- Test pad ---

  /**
   * @brief Selects the signal on the test pad (PA2 on mimic01).
   * @param source MIMIC_TEST_PAD_OFF (not driven, after begin()) or
   *        MIMIC_TEST_PAD_SAMPLE_CLOCK: a 100 kHz square wave that toggles at every sample,
   *        a reference for the output timing (with an oscillator square at
   *        MIMIC_OSC_FREQ_MAX_HZ on the analog output, or a V/F converter at that frequency
   *        on the digital output, the output also changes at every sample).
   */
  void setTestPadOutput(uint8_t source);

  // --- Nonlinear: logarithm, multiplier, true RMS, dead zone ---

  /**
   * @brief Logarithmic amplifier: out = Vref + slope * log10((in - Vref) / unit).
   * @param slopeVPerDecade Output change per decade of the input in V (negative = inverting).
   * @param unitV Input above Vref at which the output equals Vref.
   * @note  The input is a magnitude above Vref; inputs near or below Vref saturate low.
   *        For a sensor whose zero is 0 V (a photodiode), call setVref(0.0f). After an
   *        envelope follower module, use the same Vref in both modules.
   */
  void setLog(float slopeVPerDecade = 1.0f, float unitV = 0.5f);

  /**
   * @brief Antilogarithmic (exponential) amplifier: out = unit * 10^((in - Vref) / slope).
   * @param slopeVPerDecade Input change per decade of the output in V (same as setLog()).
   * @param unitV Output voltage (from 0 V) when the input equals Vref.
   * @note  The output is a magnitude from 0 V (lift it with setOutputOffset() if needed).
   *        With the same slope and unit, setAntilog() is the inverse of setLog() when
   *        the Vref of setLog() equals the output offset of setAntilog().
   */
  void setAntilog(float slopeVPerDecade = 1.0f, float unitV = 0.5f);

  /**
   * @brief Squarer (an analog multiplier with both inputs tied):
   *        out = (in - Vref)^2 / unit, a magnitude from 0 V.
   *        Squaring a sine doubles its frequency; the output is also proportional to power.
   * @param unitV Scale voltage, like the 10 V of an analog multiplier (> 0). 0 (default)
   *        uses VDD / 4: an input of Vref +/- VDD / 2 gives VDD.
   */
  void setSquarer(float unitV = 0.0f);

  /**
   * @brief Square rooter (an analog multiplier in the feedback of an op-amp):
   *        out = sqrt((in - Vref) * unit), 0 below Vref, a magnitude from 0 V.
   *        For a differential pressure sensor, set Vref to its zero output: the output is
   *        proportional to the flow.
   * @param unitV Scale voltage (> 0). 0 (default) uses VDD / 4. A squarer and a square
   *        rooter with the same unit give |in - Vref| back.
   */
  void setSquareRooter(float unitV = 0.0f);

  /**
   * @brief True RMS-to-DC converter: out = sqrt(mean((in - Vref)^2)), a magnitude from 0 V
   *        (gain 1). Like the averaging capacitor of an RMS converter IC, the time
   *        constant sets the trade-off between the ripple at low frequencies and the
   *        response time.
   * @param averagingTimeUs Averaging time constant in us
   *        (MIMIC_TRUE_RMS_TIME_MIN_US - MIMIC_TRUE_RMS_TIME_MAX_US).
   * @note  Use it without decimation: decimation averages the input before the DSP, and
   *        the processing does not fit in one sample period.
   */
  void setTrueRms(float averagingTimeUs);

  /**
   * @brief Dead zone (a diode dead-band circuit): Vref while the input is between the
   *        limits, and Vref + (in - limit) beyond a limit (slope 1, no step). Small signals
   *        and noise around the center are removed; larger ones pass shifted by the limit.
   * @param upperV Upper limit in V. @param lowerV Lower limit in V (absolute, like setClipper()).
   */
  void setDeadZone(float upperV, float lowerV);

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
  void sendIntegrator(uint8_t type, float k, float c);
  void sendMultiplier(uint8_t type, float unitV);
  float oscFmGain(float hzPerVolt) const;
  void sendOscillator(uint8_t mode, uint8_t waveform, float freqHz, float amplitudeV, float k);
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
