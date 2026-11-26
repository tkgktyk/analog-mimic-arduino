// Finds the Analog Mimic modules on the I2C bus (addresses 0x40 - 0x43, set by the
// address jumpers) and checks that each one answers as a Mimic (WHO_AM_I).
//
// Each address is probed by writing one byte (the register pointer 0, harmless).
// The usual scanner (beginTransmission + endTransmission with no data) does not work on
// the Arduino UNO R4: its Wire reports an ACK for every address when no data is sent,
// and requestFrom() returns the requested count even without an ACK. Only the Mimic
// addresses are probed, because writing a byte to an unknown part can be taken as a
// command by some parts.
#include <Wire.h>
#include "AnalogMimic.h"

const uint8_t FIRST_ADDR = MIMIC_DEFAULT_I2C_ADDR;      // 0x40: no jumper closed
const uint8_t LAST_ADDR = MIMIC_DEFAULT_I2C_ADDR + 3;   // 0x43: A1 and A0 closed

// One byte written to the register pointer: 0 when the address is acknowledged.
uint8_t probe(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)MIMIC_REG_SYSTEM_ID);
  return Wire.endTransmission();
}

// Reads WHO_AM_I (the pointer was set by probe()); -1 when nothing is received.
int readWhoAmI(uint8_t address) {
  if (Wire.requestFrom(address, (uint8_t)1) != 1 || !Wire.available()) return -1;
  return Wire.read();
}

void scan() {
  Serial.println("\nScanning 0x40 - 0x43 (address jumpers: A1 = bit 1, A0 = bit 0; closed = 1)");
  int found = 0;
  for (uint8_t address = FIRST_ADDR; address <= LAST_ADDR; address++) {
    uint8_t error = probe(address);
    Serial.print("0x");
    Serial.print(address, HEX);
    if (error != 0) {
      Serial.println(": no answer");
      continue;
    }
    int id = readWhoAmI(address);
    if (id == MIMIC_DEVICE_ID_VALUE) {
      Serial.println(": Analog Mimic");
      found++;
    } else {
      Serial.print(": a device, but not a Mimic (WHO_AM_I = ");
      Serial.print(id);
      Serial.println(")");
    }
  }
  Serial.print(found);
  Serial.println(found == 1 ? " module found" : " modules found");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}  // wait for the serial monitor (boards with native USB)
  Wire.begin();
  scan();
  Serial.println("\nPress Enter to scan again (after changing the jumpers, power the module again).");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c != '\r' && c != '\n') return;  // other characters are ignored
  delay(20);                            // the rest of CR LF arrives
  while (Serial.available()) Serial.read();
  scan();
  Serial.println("\nPress Enter to scan again.");
}
