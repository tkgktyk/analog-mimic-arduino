# Analog Mimic for Arduino (analog-mimic-arduino)

A cross-architecture C++ library for **Analog Mimic**—a drop-in, programmable analog front-end (AFE) prototyping platform.

This library abstracts complex low-level register maps and fixed-point arithmetic into an intuitive, high-level API. It allows developers to dynamically reconfigure the physical Analog Mimic module on the fly using a single I²C master bus.

---

## Key Features
* **Zero-Hassle Control:** Low-level I²C register transactions and Q15 fixed-point mathematical scaling are completely abstracted away.
* **Intuitive High-Level API:** Dynamically switch emulation modes (filters, amplifiers, comparators) and fine-tune physical parameters (cutoff frequencies, gains) with simple function calls.
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
* **Via Arduino Library Manager (Recommended):** Search for `Analog Mimic` in the Arduino IDE Library Manager and click **Install**.
* **Manual Installation:** Download this repository as a `.zip` file, and import it via *Sketch -> Include Library -> Add .ZIP Library...*

### 2. Basic Usage
Include the header, create an instance with the module supply voltage, and call `begin()`:

```cpp
#include <Wire.h>
#include <AnalogMimic.h>

// Module supply voltage in mV (MIMIC_VDD_5V, MIMIC_VDD_3V3 or a measured value)
Mimic0x mimic(MIMIC_VDD_5V);

void setup() {
    Wire.begin();
    mimic.begin(&Wire);

    // 1st-order low-pass filter, fc = 1 kHz
    mimic.setFilter1stLPF(1000.0f);
}

void loop() {
    // The module keeps running on its own; change the settings at any time
}
```

---

## Things to Know Before Writing a Sketch

The API reference is in [`src/AnalogMimic.h`](src/AnalogMimic.h). This section covers the behaviour that applies across all functions.

### Voltages are relative to the supply (ratiometric)
The module's ADC and DAC both use the module supply (VDD) as their reference. One LSB is `VDD / 4096`, so 0 V reads as 0 and VDD as 4096 (the highest code is 4095).

Functions that take millivolts convert them using the supply voltage given to the constructor:

```cpp
Mimic0x mimic(MIMIC_VDD_5V);   // or MIMIC_VDD_3V3, or the measured value in mV, e.g. 4980
```

If the real supply differs from this value, all mV settings scale with it. Pass the measured supply voltage when absolute accuracy matters.

### Absolute voltages and the common reference (Vref)
* **Thresholds and limits are absolute** (measured from GND): `setComparatorWindow()`, `setComparatorSchmitt()`, `setClipper()` and the `unitMv` of `setLog()` / `setAntilog()`.
* **Signal processing is centred on Vref.** Modes that have a signal centre work as `out = Vref + H(in - Vref)`: PGA, Biquad filters, rectifiers, envelope follower, log / antilog, derivative / integral.
* Vref defaults to **VDD / 2** after `begin()`. Change it with `setVref(mV)` (for example, `setVref(0)` for a single-supply signal referenced to GND).
* Changing Vref does not reset the state of filters and other modes, so the output moves smoothly, like an analog circuit whose reference has shifted.

### Changing settings while running
* The output keeps its previous value while a new setting is being applied.
* The output is disconnected briefly only when the output route changes (analog output ↔ comparator digital output).
* `begin()` resets the global settings (output flags, offset, decimation, Vref). It does not change the current mode or the calibration.

### Sampling rate and decimation
* The module samples at **200 kHz**. Functions that take a frequency or a time (filters, envelope follower, S&H, derivative / integral) calculate their coefficients from the current sampling rate.
* `setDecimation(N)` lowers the processing rate to `200 kHz / 2^N` (N = 0 to 16), for long delays, slow S&H, long time constants or averaging. Input samples are averaged and the output is interpolated. The output continues from its current value when N is changed. **Call `setDecimation()` before setting a mode**; the coefficients are calculated when the mode is set.
* **Biquad filters:** `fc` can be set from 1/100000 of the sampling rate (2 Hz at 200 kHz) to 0.24 of it, and `Q` from 0.1 to 10; values outside are clamped. Decimation is not needed for low cutoffs.

### Calibration
Each module is calibrated at production, and the gain and offset are stored in its flash memory. You normally do not need `setGainCal()`, `setOffsetCal()` or `commitCalibrationToNvm()`. Writing new values to NVM overwrites the factory calibration.

### Firmware and library versions
The register map is shared between the firmware and this library (`mimic_registers.h`), so use a library and firmware from the same period. `getBuildDate()` returns the firmware build date as `0xYYYYMMDD`:

```cpp
Serial.println(mimic.getBuildDate(), HEX);   // e.g. 20261021
Serial.println(mimic.getUnitId(), HEX);      // ID of this module
```

### Reading the module output with the host ADC
The module output has a series resistor. Many microcontroller ADCs assume a low source impedance, so a short sampling time can make readings slightly low (for example, about 0.2 % on the Arduino UNO R4). If you measure the output with the host ADC, lengthen the sampling time or buffer the signal.

Example for the Arduino UNO R4 (RA4M1), called once after the first `analogRead()`:

```cpp
(void)analogRead(A1);            // Opens the ADC
for (uint8_t ch = 0; ch < 8; ch++) {
  R_ADC0->ADSSTR[ch] = 255;      // Longest sampling time (about 5.3 us)
}
```

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
