/**
 ******************************************************************************
 * @file    AnalogMimic.h
 * @author  TAKAGI Katsuyuki
 * @brief   Host-Side Helper Functions and Arduino Library API for 
 * Communicating with the Analog Mimic Hardware via I2C.
 *-----------------------------------------------------------------------------
 * Copyright (C) 2026 TAKAGI Katsuyuki
 *
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 ******************************************************************************
 */

#ifndef ANALOG_MIMIC_H
#define ANALOG_MIMIC_H

#include <Arduino.h>
#include <Wire.h>

// Include shared register definitions (Single Source of Truth)
#include "mimic_registers.h"

// Constants for Power Supply Voltage Selection
constexpr uint16_t MIMIC_VDD_3V3 = 3300;
constexpr uint16_t MIMIC_VDD_5V  = 5000;

// =========================================================
// Filter Constants (Q-Factors)
// =========================================================
constexpr float MIMIC_FILTER_Q_BUTTERWORTH = 0.7071f;

// Biquad (2nd-order) filter limits; fc relative to the current sampling rate
constexpr float MIMIC_BIQUAD_FC_MIN_RATIO = 1.0e-5f;  // fs / 100000 (2 Hz at 200 kHz)
constexpr float MIMIC_BIQUAD_FC_MAX_RATIO = 0.24f;    // just below fs / 4
constexpr float MIMIC_BIQUAD_Q_MIN        = 0.1f;
constexpr float MIMIC_BIQUAD_Q_MAX        = 10.0f;
constexpr float MIMIC_FILTER_Q_BESSEL      = 0.5773f;
constexpr float MIMIC_FILTER_Q_CHEBYSHEV   = 1.0f;

// =========================================================
// MimicBase (Core interface for all variants)
// =========================================================
/**
 * @brief Core interface for the Analog Mimic module.
 * * This class abstracts the low-level I2C communication and Q15 fixed-point
 * arithmetic, allowing users to configure the hardware-deterministic DSP 
 * engine using intuitive physical units (Hz, mV, microseconds, etc.).
 */
class MimicBase
{
public:
  virtual ~MimicBase() = default;

  /**
   * @brief Initializes the Mimic module and synchronizes initial states.
   * @param wire Pointer to the TwoWire (I2C) instance. Defaults to &Wire.
   * @return true if initialization is successful.
   */
  bool begin(TwoWire *wire = &Wire);

  // --- Read Section ---

  /**
   * @brief Reads a raw 8-bit value from a specific I2C register.
   * @param addr Register address.
   * @return The 8-bit register value.
   */
  uint8_t getOneByte(uint8_t addr);

  /**
   * @brief Reads a raw 16-bit value from a specific I2C register.
   * @param addr Register address.
   * @return The 16-bit register value.
   */
  uint16_t getTwoBytes(uint8_t addr);

  /**
   * @brief Retrieves the current status flag of the module.
   * @return 8-bit status code.
   */
  uint8_t getStatus();

  /**
   * @brief Reads the current raw 12-bit ADC value (0-4095).
   * @return 12-bit raw ADC reading.
   */
  uint16_t getAdcValue();

  /**
   * @brief Retrieves the CPU cycle count consumed by the current DSP routine.
   * @return Number of clock cycles executed in the Primary ISR.
   */
  uint16_t getCpuCycles();

  /**
   * @brief Reads the firmware build date.
   * @return Date as hex 0xYYYYMMDD (e.g. 0x20261021); print with HEX to read it.
   */
  uint32_t getBuildDate();

  /**
   * @brief Reads the 32-bit unit ID (FNV-1a hash of the MCU unique ID).
   * @return Unit ID. Fixed for each chip; different chips give different values
   *         with high probability (not guaranteed unique).
   */
  uint32_t getUnitId();

  // --- Global Configurations (Pipeline Pre/Post-Processing) ---

  /**
   * @brief Enables or disables the output (Hi-Z state if supported).
   * @param openEnabled True to open/disable output, False to enable.
   */
  void setOutputOpen(bool openEnabled);

  /**
   * @brief Applies an absolute hardware-level polarity inversion to the final output.
   * @param inverted True to invert the DAC output (4095 - val), False for normal.
   */
  void setOutputInverted(bool inverted);

  /**
   * @brief Lowers the processing rate to 200 kHz / 2^N (input averaged, output interpolated).
   *        Call before setting a mode: coefficients are calculated from this rate.
   * @param N 0 (no decimation) to MIMIC_DECIMATION_N_MAX (16); larger values are clamped.
   */
  void setDecimation(uint8_t N);

  /**
   * @brief Applies a manual DC offset to the output signal.
   * @param offsetMv Offset voltage in millivolts (can be negative). Resolution: VDD / 32768.
   */
  void setOutputOffset(int16_t offsetMv);

  /**
   * @brief Sets the common reference voltage (Vref), the signal ground of the module.
   * @param vrefMv Reference voltage in mV (0 to VDD). Defaults to VDD / 2 after begin().
   * @note  Every mode with a signal center works around Vref: 1st LPF/HPF, Biquad, PGA,
   *        envelope, derivative, integral and rectifiers. Vref is an input: changing it
   *        does not reset the filter states (a slow HPF moves to the new Vref gradually).
   */
  void setVref(uint16_t vrefMv);

  // --- 0. Bypass DSP ---

  /**
   * @brief Bypasses all DSP processing, acting as a simple voltage follower.
   */
  void setBypassDSP();

  // --- 1. Filtering ---

  /**
   * @brief Configures a 1st-order Low-Pass Filter (RC filter equivalent).
   * @param fc Cutoff frequency in Hz.
   */
  void setFilter1stLPF(float fc);

  /**
   * @brief Configures a 1st-order High-Pass Filter.
   * @param fc Cutoff frequency in Hz.
   */
  void setFilter1stHPF(float fc);

  /**
   * @brief Configures a Unity-Gain Biquad. Passes signal unchanged but consumes 
   * the exact same CPU cycles as other Biquad filters for profiling purposes.
   * @note  All Biquad filters: fc is limited to fs / 100000 - 0.24 fs (2 Hz - 48 kHz at
   *        200 kHz) and Q to 0.1 - 10. Values outside are clamped.
   */
  void setFilterBiquadUnity();

  /**
   * @brief Configures a 2nd-order Biquad Low-Pass Filter.
   * @param fc Cutoff frequency in Hz.
   * @param Q Quality factor. Defaults to Butterworth (0.7071).
   */
  void setFilterBiquadLPF(float fc, float Q = MIMIC_FILTER_Q_BUTTERWORTH);

  /**
   * @brief Configures a 2nd-order Biquad High-Pass Filter.
   * @param fc Cutoff frequency in Hz.
   * @param Q Quality factor. Defaults to Butterworth (0.7071).
   */
  void setFilterBiquadHPF(float fc, float Q = MIMIC_FILTER_Q_BUTTERWORTH);

  /**
   * @brief Configures a 2nd-order Biquad Band-Pass Filter.
   * @param fc Center frequency in Hz.
   * @param Q Quality factor (bandwidth control). Defaults to 1.0.
   */
  void setFilterBiquadBPF(float fc, float Q = 1.0f);

  /**
   * @brief Configures a 2nd-order Biquad Notch (Band-Stop) Filter.
   * @param fc Center frequency in Hz.
   * @param Q Quality factor (notch width control). Defaults to 1.0.
   */
  void setFilterBiquadNotch(float fc, float Q = 1.0f);

  // --- 2. Programmable Gain Amplifier (PGA) ---

  /**
   * @brief Configures the Programmable Gain Amplifier: out = Vref + gain * (in - Vref).
   * @param gain Linear gain multiplier (e.g., 5.5 for 5.5x amplification).
   * @note  The amplification center is the common Vref (see setVref()).
   */
  void setPGA(float gain);

  // --- 3. Logarithmic / Antilogarithmic Amplifier ---

  /**
   * @brief Configures a Logarithmic Amplifier: out = Vref + slope * log10(in / unit).
   * @param slopeMvPerDecade Output change per decade of the input in mV (negative = inverting).
   * @param unitMv Input voltage (from 0 V) at which the output equals Vref.
   * @note  The input is a magnitude measured from 0 V; inputs near 0 V saturate low.
   */
  void setLog(int16_t slopeMvPerDecade = 1000, uint16_t unitMv = 500);

  /**
   * @brief Configures an Antilogarithmic (exponential) Amplifier: out = unit * 10^((in - Vref) / slope).
   * @param slopeMvPerDecade Input change per decade of the output in mV (same as setLog()).
   * @param unitMv Output voltage (from 0 V) when the input equals Vref.
   * @note  With the same Vref, slope and unit, setAntilog() is the inverse of setLog().
   */
  void setAntilog(int16_t slopeMvPerDecade = 1000, uint16_t unitMv = 500);

  // --- 4. Comparators ---

  /**
   * @brief Configures a Window Comparator.
   * @param upperMv Upper threshold voltage in mV.
   * @param lowerMv Lower threshold voltage in mV.
   */
  void setComparatorWindow(uint16_t upperMv, uint16_t lowerMv);

  /**
   * @brief Configures a Schmitt Trigger with hysteresis.
   * @param upperMv Upper threshold voltage in mV (Trigger ON).
   * @param lowerMv Lower threshold voltage in mV (Trigger OFF).
   */
  void setComparatorSchmitt(uint16_t upperMv, uint16_t lowerMv);

  // --- 5. Waveform Analysis ---

  /**
   * @brief Configures an Envelope Follower (AM detector).
   * @param decayTimeUs Release/decay time constant in microseconds.
   * @param polarity MIMIC_POLARITY_POSITIVE (peak) or MIMIC_POLARITY_NEGATIVE (trough).
   */
  void setEnvelopeFollower(float decayTimeUs, uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /**
   * @brief Configures a Peak Hold circuit (Envelope follower with infinite decay).
   * @param polarity MIMIC_POLARITY_POSITIVE (max hold) or MIMIC_POLARITY_NEGATIVE (min hold).
   */
  void setPeakHold(uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /**
   * @brief Configures a Sample and Hold (S&H) circuit.
   * @param freqHz Sampling frequency in Hz.
   */
  void setSampleAndHold(float freqHz);

  /**
   * @brief Configures a Track and Hold (T&H) circuit.
   * @param freqHz Trigger frequency in Hz.
   * @param trackRatio Ratio of the period spent tracking the signal (0.0 to 1.0).
   */
  void setTrackAndHold(float freqHz, float trackRatio = 0.5f);

  // --- 6. Clipping (Limiter) ---

  /**
   * @brief Configures a Hard Clipper (Limiter).
   * @param upperMv Upper voltage limit in mV.
   * @param lowerMv Lower voltage limit in mV.
   */
  void setClipper(uint16_t upperMv, uint16_t lowerMv);

  // --- 7. Calculus (Math) ---

  /**
   * @brief Configures an Analog Differentiator.
   * @param timeConstantUs RC time constant in microseconds.
   */
  void setMathDerivative(float timeConstantUs);

  /**
   * @brief Configures a Leaky Integrator.
   * @param timeConstantUs RC time constant in microseconds.
   * @param inputShift Bit-shift value for the input to prevent rapid saturation.
   */
  void setMathIntegral(float timeConstantUs, int8_t inputShift = -2);

  /**
   * @brief Configures a Half-Wave Rectifier folding at the common Vref.
   * @param polarity MIMIC_POLARITY_POSITIVE: max(in, Vref), MIMIC_POLARITY_NEGATIVE: min(in, Vref).
   */
  void setRectifierHalf(uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /**
   * @brief Configures a Full-Wave Rectifier (absolute value around Vref).
   * @param polarity MIMIC_POLARITY_POSITIVE: Vref + |in - Vref|, MIMIC_POLARITY_NEGATIVE: Vref - |in - Vref|.
   */
  void setRectifierFull(uint8_t polarity = MIMIC_POLARITY_POSITIVE);

  /**
   * @brief Configures a Slew Rate Limiter.
   * @param maxChangePerSampleRatio Maximum allowed voltage change per sample (0.0 to 1.0 of full scale).
   */
  void setSlewRateLimiter(float maxChangePerSampleRatio);

  /**
   * @brief Configures a pure Digital Delay line (Ring Buffer).
   * @param delaySamples Number of samples to delay (Max 4095).
   * @note Actual delay time = delaySamples * (1.0 / SamplingRate).
   */
  void setDigitalDelay(uint16_t delaySamples);

  /**
   * @brief Sets the output gain calibration (RAM; use commitCalibrationToNvm() to store).
   * @param gain Gain multiplier (e.g., 1.0023). Stored as uq1_15 (0 to < 2.0).
   */
  void setGainCal(float gain);

  /**
   * @brief Sets the output offset calibration (RAM; use commitCalibrationToNvm() to store).
   * @param offsetLsb Offset in output LSB (e.g., -6.44). Stored as raw_fx3 (1/8 LSB).
   */
  void setOffsetCal(float offsetLsb);

  void commitCalibrationToNvm();

  void reloadCalibrationFromNvm();

protected:
  MimicBase(uint16_t vddMv, uint8_t hardwareJumper = 0);

  uint8_t _i2c_addr;
  TwoWire *_wire;
  uint16_t _vdd_mv;
  float _default_fs_hz;
  float _fs_hz;

  // State retention variables
  uint8_t _global_flags;
  int16_t _offset_raw_fx3;   // User output offset (raw_fx3)
  uint8_t _decimation_N;

  uint8_t readRegister8(uint8_t regAddr);
  uint16_t readRegister16(uint8_t regAddr);
  uint32_t readRegister32(uint8_t regAddr);

  void writeModeAndPayload(uint8_t mode, const uint8_t *payload, uint8_t length);
  void writeOneByte(uint8_t addr, uint8_t value);
  void writeTwoBytes(uint8_t addr, uint16_t value);

  int16_t mvToRawFx3(int32_t mv);
  void sendLog(uint8_t type, int16_t slopeMvPerDecade, uint16_t unitMv);
  int16_t floatToQ0_15(float value);
  void floatToPayloadQ0_15(float value, uint8_t *buffer, uint8_t offset);
  void rawToPayload16(uint16_t raw, uint8_t *buffer, uint8_t offset);
  void int16ToPayload(int32_t value, uint8_t *buffer, uint8_t offset);
  void sendBiquad(uint8_t type, float fc, float Q);
};

// =========================================================
// Variant Classes
// =========================================================

/**
 * @brief Standard variant of Analog Mimic (mimic0x).
 * Uses internal OPAs for AFE without external hardware modding capabilities.
 */
class Mimic0x : public MimicBase
{
public:
  explicit Mimic0x(uint16_t vddMv, uint8_t hardwareJumper = 0) : MimicBase(vddMv, hardwareJumper) {}
};

/**
 * @brief Extended AFE variant of Analog Mimic (mimic1x).
 * Features an external OPA and DC servo capabilities for advanced sensor interfacing.
 */
class Mimic1x : public MimicBase
{
public:
  explicit Mimic1x(uint16_t vddMv, uint8_t hardwareJumper = 0) : MimicBase(vddMv, hardwareJumper) {}
  
  /**
   * @brief Enables or disables the hardware DC servo function.
   * @note Only available on mimic1x variants with the secondary internal DAC routed to the AFE.
   * @param enabled True to enable the hardware DC servo.
   */
  void setHardwareServoEnabled(bool enabled);
};

#endif // ANALOG_MIMIC_H
