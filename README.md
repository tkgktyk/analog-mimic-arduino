# Analog Mimic for Arduino (analog-mimic-arduino)

A cross-architecture C++ library for **Analog Mimic**—a drop-in, programmable analog front-end (AFE) prototyping platform.

This library abstracts complex low-level register maps and fixed-point arithmetic into an intuitive, high-level API. It allows developers to dynamically reconfigure the physical Analog Mimic module on the fly using a single I²C master bus.

---

## Key Features
* **Zero-Hassle Control:** Low-level I²C register transactions and Q15 fixed-point mathematical scaling are completely abstracted away.
* **Intuitive High-Level API:** Dynamically switch emulation modes (filters, amplifiers, comparators) and fine-tune physical parameters (cutoff frequencies, gains) with simple function calls.
* **Unified Register Mapping:** Features native, single-source integration with `mimic_registers.h` shared directly with the device firmware to eliminate protocol mismatch.
* **Cross-Architecture Ready:** Lightweight and highly compatible (`architectures=*`), running seamlessly on AVR, SAMD, ESP32, and STM32/RP2340 host platforms.

## Target Hardware
This library is fully compatible with the entire Analog Mimic ecosystem:
* **mimic0x (Standard Type):** Base model for generalized mixed-signal prototyping.
* **mimic1x (AFE Extended Type):** Enhanced variant for specialized sensor interfacing.

*For the hardware design files, core MCU firmware, and underlying architectural specifications, please visit the primary repository: [tkgktyk/analog-mimic](https://github.com/tkgktyk/analog-mimic).*

---

## Getting Started

### 1. Installation
* **Via Arduino Library Manager (Recommended):** Search for `Mimic` in the Arduino IDE Library Manager and click **Install**.
* **Manual Installation:** Download this repository as a `.zip` file, and import it via *Sketch -> Include Library -> Add .ZIP Library...*

### 2. Basic Usage
Include the header and initialize the Mimic instance over your preferred Wire (I²C) interface:

```cpp
#include <Wire.h>
#include <Mimic.h>

// Create a Mimic instance (Default I2C address)
Mimic mimic(MIMIC_VDD_5V);

void setup() {
    Wire.begin();
    mimic.begin(Wire);

    // Dynamic reconfiguration example: Set to Low-Pass Filter at 1.0 kHz
    mimic.setFilter1stLPF(1000);
}

void loop() {
    // Read runtime virtual telemetry / register status if needed
}
