/*
 * ContinuousRead - I2C load test for Analog Mimic
 *
 * Drives back-to-back status reads over I2C so that the emulator can be
 * observed under sustained host traffic. Intended for the "host active"
 * measurement condition: the sample engine keeps running while the bus is
 * continuously busy.
 *
 * The measurement loop contains no delay() and no Serial output. A summary
 * is printed once per REPORT_INTERVAL_MS, which is short enough relative to
 * the reporting period to leave the bus essentially saturated.
 *
 * Set SILENT_MODE to 1 to remove Serial from the build entirely.
 */

#include <Wire.h>
#include <AnalogMimic.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define SILENT_MODE          0        // 1: no Serial at all during the test
#define I2C_CLOCK_HZ         100000UL // must match the device configuration
#define REPORT_INTERVAL_MS   5000UL

// Communication error status definition
const uint8_t MIMIC_COMM_ERROR = 0xFF;

// Create an instance of Mimic0x
Mimic0x mimic(MIMIC_VDD_5V);

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------
static unsigned long reads_ok    = 0;
static unsigned long reads_error = 0;
static unsigned long window_start_ms = 0;
static unsigned long window_reads    = 0;

void setup() {
#if !SILENT_MODE
  Serial.begin(115200);
#endif

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  if (mimic.begin(&Wire)) {
#if !SILENT_MODE
    Serial.println(F("Mimic Library Initialized."));
#endif
  }

  // Configure once, before the measurement starts. No reconfiguration is
  // issued during the loop, so the device stays in its free-running state
  // and only status reads appear on the bus.
  mimic.setBypassDSP();
  delay(50);
  mimic.setComparatorSchmitt(0,0);
  delay(50);
  mimic.setBypassDSP();

#if !SILENT_MODE
  Serial.println(F("Starting continuous read. No further writes will be issued."));
#endif

  window_start_ms = millis();
}

void loop() {
  // --- measurement loop: no delay, no Serial ------------------------------
  uint8_t status = mimic.getStatus();

  if (status == MIMIC_COMM_ERROR) {
    reads_error++;
  } else {
    reads_ok++;
  }
  window_reads++;

  // --- periodic summary ---------------------------------------------------
  unsigned long now = millis();
  unsigned long elapsed = now - window_start_ms;

  if (elapsed >= REPORT_INTERVAL_MS) {
#if !SILENT_MODE
    // reads per second over the window
    unsigned long rate = (window_reads * 1000UL) / elapsed;

    Serial.print(F("reads/s="));
    Serial.print(rate);
    Serial.print(F("  total_ok="));
    Serial.print(reads_ok);
    Serial.print(F("  errors="));
    Serial.println(reads_error);
#endif
    window_start_ms = now;
    window_reads    = 0;
  }
}
