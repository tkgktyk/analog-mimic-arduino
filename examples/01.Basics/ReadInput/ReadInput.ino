// ReadInput: reading the module input from the Arduino
//
// The module also works as a measuring front end: getInputVolts() returns the voltage
// at its input, and getAdcValue() the raw value (ADC code x 8, 1/8 LSB) with a
// sequence number that counts the updates. With setDecimation(N), the value is the
// average of 2^N samples.
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Open the Serial Plotter (115200 baud): a slow sine wave measured by the module.

#include <Wire.h>
#include <AnalogMimic.h>
#include <MimicTestSignal.h>

Mimic0x mimic(MIMIC_VDD_5V);
MimicTestSignal testSignal;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mimic.begin(&Wire)) {
    Serial.println("Analog Mimic not found");
    while (1);
  }
  testSignal.begin(MimicTestSignal::SINE, 1.0f);  // 1 Hz, +-1.25 V around 2.5 V
  mimic.setBypassDSP();
  mimic.setDecimation(8);  // average of 256 samples (1.28 ms) per value
}

void loop() {
  uint8_t seq;
  uint16_t raw = mimic.getAdcValue(&seq);  // the value and its sequence number together
  float volts = mimic.codeToVolts(raw);  // the same as getInputVolts(), without a second read

  Serial.print("input_V:");
  Serial.print(volts, 4);
  Serial.print(" seq:");
  Serial.println(seq);
  delay(20);
}
