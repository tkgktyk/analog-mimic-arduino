# Analog Mimic for Arduino (analog-mimic-arduino)

A cross-architecture C++ library for **Analog Mimic**—a drop-in, programmable analog front-end (AFE) prototyping platform.

This library abstracts complex low-level register maps and fixed-point arithmetic into an intuitive, high-level API. It allows developers to dynamically reconfigure the physical Analog Mimic module on the fly using a single I²C master bus.

---

## Key Features
* **Zero-Hassle Control:** Low-level I²C register transactions and fixed-point scaling are completely abstracted away.
* **Physical Units:** Switch emulation modes (filters, amplifiers, comparators) and set their parameters in V, µs, Hz and V/µs, like the datasheet of the analog part.
* **Unified Register Mapping:** Features native, single-source integration with `mimic_registers.h` shared directly with the device firmware to eliminate protocol mismatch.
* **Cross-Architecture Ready:** Lightweight and highly compatible (`architectures=*`), running seamlessly on AVR, SAMD, ESP32, and STM32/RP2040 host platforms.

## Target Hardware
This library is fully compatible with the entire Analog Mimic ecosystem:
* **mimic0x (Standard Type):** Base model for generalized mixed-signal prototyping.
* **mimic1x (AFE Extended Type):** Enhanced variant for specialized sensor interfacing.

*For the hardware design files, core MCU firmware, and underlying architectural specifications, please visit the primary repository: [tkgktyk/analog-mimic](https://github.com/tkgktyk/analog-mimic).*

---

## Getting Started

### 1. Installation
* **Via Arduino Library Manager (Recommended):** Search for `AnalogMimic` in the Arduino IDE Library Manager and click **Install**.
* **Manual Installation:** Download this repository as a `.zip` file, and import it via *Sketch -> Include Library -> Add .ZIP Library...*

### 2. Basic Usage
Include the header, create an instance with the module supply voltage **in volts**, and call `begin()`:

```cpp
#include <Wire.h>
#include <AnalogMimic.h>

// Module supply voltage in V (MIMIC_VDD_5V, MIMIC_VDD_3V3 or a measured value, e.g. 4.98f)
Mimic0x mimic(MIMIC_VDD_5V);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    if (!mimic.begin(&Wire)) {
        Serial.println("Analog Mimic not found");
    }

    // 1st-order low-pass filter, fc = 1 kHz
    mimic.setFilter1stLPF(1000.0f);
}

void loop() {
    // The module keeps running on its own; change the settings at any time
}
```

`begin()` returns `false` when the module does not answer, or when the supply voltage is outside 1 - 6 V (for example 5000, given in mV by mistake).

---

## Things to Know Before Writing a Sketch

The API reference is in [`src/AnalogMimic.h`](src/AnalogMimic.h). This section covers the behaviour that applies across all functions.

### Units
Every function uses the same units, chosen like the datasheet of the analog part each mode emulates:

| Quantity | Unit | Examples |
|---|---|---|
| Voltage | V (`float`) | `setVref(2.5f)`, `setComparatorSchmitt(3.0f, 2.0f)`, `setLog(1.0f, 0.5f)` |
| Time | µs | `setEnvelopeFollower(50000.0f)` (50 ms), `setMathDerivative(100.0f)` |
| Frequency | Hz | `setFilterBiquadLPF(1000.0f)`, `setSampleAndHold(500.0f)` |
| Slew rate | V/µs | `setSlewRateLimiter(0.01f)` |
| Gain, Q, ratios | none | `setPGA(2.0f)`, `setFilterBiquadBPF(1000.0f, 5.0f)` |
| Delay | samples (1 sample = 5 µs) | `setDigitalDelay(100)` |

### Voltages are relative to the supply (ratiometric)
The module's ADC and DAC both use the module supply (VDD) as their reference. One LSB is `VDD / 4096`, so 0 V reads as 0 and VDD as 4096 (the highest code is 4095).

Voltages are converted with the supply voltage given to the constructor. If the real supply differs from this value, every voltage setting scales with it, so pass the measured supply voltage when absolute accuracy matters. For a sensor whose output is itself proportional to VDD (a potentiometer, a ratiometric sensor), the scaling cancels out.

Inside the module, voltages are codes with 1/8 LSB resolution (`raw_fx3`, 1 code = VDD / 32768). `voltsToCode()` and `codeToVolts()` convert between them; `getAdcValue()` and `setVrefCode()` work with codes directly (for calibration and testing).

### Absolute voltages and the common reference (Vref)
* **Thresholds and limits are absolute** (measured from GND): `setComparatorWindow()`, `setComparatorSchmitt()`, `setClipper()`, `setSoftClipper()` and the `unitV` of `setLog()` / `setAntilog()`.
* **Signal processing is centred on Vref.** Modes that have a signal centre work as `out = Vref + H(in - Vref)`: PGA, filters, rectifiers, envelope follower, log / antilog, derivative / integral.
* Vref defaults to **VDD / 2** after `begin()`. Change it with `setVref(volts)` (for example, `setVref(0.0f)` for a single-supply signal referenced to GND).
* Changing Vref does not reset the state of filters and other modes, so the output moves smoothly, like an analog circuit whose reference has shifted.

### Changing settings while running
* The output keeps its previous value while a new setting is being applied.
* The output is disconnected briefly only when the output route changes (analog output ↔ comparator digital output).
* `begin()` resets the global settings (output flags, offset, decimation, Vref). It does not change the current mode or the calibration.

### Sampling rate and decimation
* The module samples at **200 kHz**. Times and frequencies refer to this rate.
* **Filters:** `fc` can be set from 2 Hz to 48 kHz (`MIMIC_FILTER_FC_MIN_HZ` - `MIMIC_FILTER_FC_MAX_HZ`), and `Q` from 0.1 to 10; values outside are clamped. Decimation is not needed for low cutoffs.
* **Decimation (advanced):** `setDecimation(N)` slows the whole module down by 2^N (N = 0 to 16): the processing rate becomes `200 kHz / 2^N`, input samples are averaged and the output is interpolated. **Every time becomes 2^N times longer and every frequency 2^N times lower**, for the current mode and the next ones; the settings are not recalculated. For example, `setFilterBiquadLPF(1000.0f)` with `setDecimation(4)` gives fc = 62.5 Hz. Use it for long delays, slow S&H, very long time constants or averaging. The output continues from its current value when N is changed.

### Calibration
Each module is calibrated at production, and the gain and offset are stored in its flash memory. You normally do not need `setGainCal()`, `setOffsetCal()` or `commitCalibrationToNvm()`. Writing new values to NVM overwrites the factory calibration.

### Firmware and library versions
The register map is shared between the firmware and this library (`mimic_registers.h`), so use a library and firmware from the same period. `getBuildDate()` returns the firmware build date as `0xYYYYMMDD`:

```cpp
Serial.println(mimic.getBuildDate(), HEX);   // e.g. 20261031
Serial.println(mimic.getUnitId(), HEX);      // ID of this module
```

### I²C address
The module answers at 0x40 - 0x43, selected by the A1 / A0 solder jumpers on the bottom side (read at power-up). Pass the offset to the constructor: `Mimic0x mimic(MIMIC_VDD_5V, 1);` for 0x41.

### Reading the module output with the host ADC
The module output has a series resistor. Many microcontroller ADCs assume a low source impedance, so a short sampling time can make readings slightly low (for example, about 0.2 % on the Arduino UNO R4). If you measure the output with the host ADC, lengthen the sampling time or buffer the signal.

Example for the Arduino UNO R4 (RA4M1), called once after the first `analogRead()`:

```cpp
(void)analogRead(A1);            // Opens the ADC
for (uint8_t ch = 0; ch < 8; ch++) {
  R_ADC0->ADSSTR[ch] = 255;      // Longest sampling time (about 5.3 us)
}
```

## Releases and Firmware Compatibility

The library and the module firmware share the register map (`src/mimic_registers.h`).
Use a library release together with firmware built within the range listed for it.

| Library | Firmware build date | Notes |
|---|---|---|
| 0.2.0 | 20261031 or later | Voltages in V, slew rate in V/µs, decimation scales time; all-pass filters and soft clipper; new mode payloads |
| 0.1.0 | 20261025 - 20261030 | First release |

Check the firmware of your module with `getBuildDate()`:

```cpp
Serial.println(mimic.getBuildDate(), HEX);   // e.g. 20261031
```

Firmware built outside the range does not work correctly with that release.
Firmware: [tkgktyk/analog-mimic](https://github.com/tkgktyk/analog-mimic).

### Updating sketches from 0.1.0
* **Voltages are in V, not mV:** the constructor (`Mimic0x mimic(5.0f)`, or `MIMIC_VDD_5V`, which is now 5.0f) and every voltage argument. A sketch that still passes mV compiles, so check every call: `setVref(2500)` must become `setVref(2.5f)`.
* `setSlewRateLimiter()` takes V/µs instead of a ratio per sample.
* `setDecimation()` no longer changes the meaning of the settings: times and frequencies of the module are scaled by 2^N.
* `setMathIntegral()`: the first argument is the leak time constant; 0 gives a pure integrator.
* `getAdcValue()` returns a code with 1/8 LSB resolution (ADC code × 8); `getInputVolts()` returns volts.
* The second constructor argument is the I²C address offset (0 - 3).

---

## Development with PlatformIO

This repository includes a PlatformIO setup for building and uploading the examples directly from the library source. Changes to `src/` are reflected immediately without reinstalling the library.

### Requirements
* [Visual Studio Code](https://code.visualstudio.com/) with the [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) extension
* Open the repository root folder (the one containing `platformio.ini`) in VS Code.

### Build / Upload the Example You Are Editing
1. Open any example sketch, e.g. `examples/Modes/Modes.ino`.
2. Press **Ctrl+Shift+B** (**Cmd+Shift+B** on macOS).
3. Select one of the tasks:
   * **PIO: Build current example** — builds the example in the folder of the open `.ino` file.
   * **PIO: Upload current example** — builds and uploads it to the board.

> **Note:** The PlatformIO toolbar buttons (✓ / →) build the environment selected in the status bar, not the file you have open. Use **Ctrl+Shift+B** to target the open sketch.

### Command Line
```sh
pio run -e Modes              # Build a specific example
pio run -e Modes -t upload    # Build and upload
pio run                       # Build all examples
```

### Adding a New Example
1. Create `examples/<Name>/<Name>.ino` (the folder and sketch names must match).
2. Add an environment to `platformio.ini`:
```ini
   [env:<Name>]
   custom_example = <Name>
```
   This is only required for command-line builds and `pio run`. The **Ctrl+Shift+B** tasks work without it.
   