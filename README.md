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
Include the header and initialize the Mimic instance over your preferred Wire (I²C) interface:

```cpp
#include <Wire.h>
#include <AnalogMimic.h>

// Create a Mimic instance (Default I2C address)
Mimic0x mimic(MIMIC_VDD_5V);

void setup() {
    Wire.begin();
    mimic.begin(&Wire);

    // Dynamic reconfiguration example: Set to Low-Pass Filter at 1.0 kHz
    mimic.setFilter1stLPF(1000);
}

void loop() {
    // Read runtime virtual telemetry / register status if needed
}

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
