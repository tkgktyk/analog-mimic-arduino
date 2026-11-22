/**
 * CpuCycles: peak CPU cycles of the module's sampling ISR for every mode.
 *
 * One sample period is 360 cycles (72 MHz / 200 kHz); about 60 - 70 cycles must be
 * left for the I2C interrupt and the main loop. For each mode the sketch prints:
 *   Transition: peak while the new setting is written over I2C
 *   Steady    : peak during 1 s without I2C traffic (the load of the mode itself)
 *   Deadline  : MISS if a sample was not finished in time during that 1 s
 *               (MIMIC_STATUS_DEADLINE_MISS; older firmware always shows ok)
 * The whole list runs twice: without decimation and with decimation N = 1.
 */
#include <Wire.h>
#include <AnalogMimic.h>

Mimic0x mimic(MIMIC_VDD_5V);

constexpr uint16_t CYCLE_BUDGET = 360;

// =========================================================
// Modes (arguments in V, us, Hz, V/us)
// =========================================================
void testBypassDSP()      { mimic.setBypassDSP(); }
void testOutputOffset()   { mimic.setBypassDSP(); mimic.setOutputOffset(0.5f); }

void testFilter1stLPF()   { mimic.setFilter1stLPF(1000.0f); }
void testFilter1stHPF()   { mimic.setFilter1stHPF(1000.0f); }
void testFilter1stAPF()   { mimic.setFilter1stAPF(1000.0f); }
void testFilterBqLPF()    { mimic.setFilterBiquadLPF(1000.0f); }
void testFilterBqHPF()    { mimic.setFilterBiquadHPF(1000.0f); }
void testFilterBqBPF()    { mimic.setFilterBiquadBPF(1000.0f); }
void testFilterBqNotch()  { mimic.setFilterBiquadNotch(1000.0f); }
void testFilterBqAPF()    { mimic.setFilterBiquadAPF(1000.0f); }

void testPGA()            { mimic.setPGA(2.0f); }

void testCompWindow()     { mimic.setComparatorWindow(3.0f, 2.0f); }
void testCompSchmitt()    { mimic.setComparatorSchmitt(3.0f, 2.0f); }

void testPeakHold()       { mimic.setPeakHold(MIMIC_POLARITY_POSITIVE); }
void testEnvelope()       { mimic.setEnvelopeFollower(50000.0f); }       // 50 ms decay
void testClipper()        { mimic.setClipper(4.0f, 1.0f); }
void testSoftClipper()    { mimic.setSoftClipper(4.0f, 1.0f); }

void testDifferentiator() { mimic.setDifferentiator(100.0f); }           // tau_d 100 us, limit 10 us
void testIntegrator()     { mimic.setIntegrator(1000.0f); }              // tau_i 1 ms, no leak
void testIntegratorLeak() { mimic.setIntegrator(1000.0f, 100000.0f); }   // leak 100 ms
void testRectHalf()       { mimic.setRectifierHalf(); }
void testRectFull()       { mimic.setRectifierFull(); }

struct ModeTest {
  const char *name;
  void (*set)();
};

const ModeTest testModes[] = {
  {"BYPASS_DSP",          testBypassDSP},
  {"OUTPUT_OFFSET",       testOutputOffset},
  {"FILTER_1ST_LPF",      testFilter1stLPF},
  {"FILTER_1ST_HPF",      testFilter1stHPF},
  {"FILTER_1ST_APF",      testFilter1stAPF},
  {"FILTER_BIQUAD_LPF",   testFilterBqLPF},
  {"FILTER_BIQUAD_HPF",   testFilterBqHPF},
  {"FILTER_BIQUAD_BPF",   testFilterBqBPF},
  {"FILTER_BIQUAD_NOTCH", testFilterBqNotch},
  {"FILTER_BIQUAD_APF",   testFilterBqAPF},
  {"PGA",                 testPGA},
  {"COMPARATOR_WINDOW",   testCompWindow},
  {"COMPARATOR_SCHMITT",  testCompSchmitt},
  {"PEAK_HOLD",           testPeakHold},
  {"ENVELOPE",            testEnvelope},
  {"CLIPPER",             testClipper},
  {"SOFT_CLIPPER",        testSoftClipper},
  {"DIFFERENTIATOR",      testDifferentiator},
  {"INTEGRATOR",          testIntegrator},
  {"INTEGRATOR_LEAK",     testIntegratorLeak},
  {"RECTIFIER_HALF",      testRectHalf},
  {"RECTIFIER_FULL",      testRectFull},
};
const size_t NUM_MODES = sizeof(testModes) / sizeof(testModes[0]);

static void printHex32(uint32_t v)
{
  for (int8_t shift = 28; shift >= 0; shift -= 4) {
    Serial.print((v >> shift) & 0xF, HEX);
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(2000);  // Time to open the serial monitor
  Serial.println(F("\n=== Analog Mimic CPU Cycle Profiler ==="));

  // Retry until the module answers (e.g. it is inserted after the sketch started)
  while (!mimic.begin(&Wire)) {
    Serial.println(F("Mimic not found (WHO_AM_I). Check the connection; retrying..."));
    Wire.end();   // A bus disturbed by inserting the module can leave Wire stuck
    Wire.begin();
    delay(1000);
  }
  Serial.print(F("Firmware build: ")); printHex32(mimic.getBuildDate());
  Serial.print(F("  Unit ID: "));      printHex32(mimic.getUnitId());
  Serial.println();
  Serial.print(F("Budget: ")); Serial.print(CYCLE_BUDGET);
  Serial.println(F(" cycles per sample (leave about 60 - 70 for I2C and the main loop)"));

  (void)mimic.getCpuCycles();   // Clear the start-up peak
  Serial.println(F("---------------------------------------"));
}

static void profile()
{
  Serial.println(F("mode                  transition  steady  deadline"));
  for (size_t i = 0; i < NUM_MODES; i++) {
    testModes[i].set();
    delay(100);
    uint16_t transition = mimic.getCpuCycles();   // Peak including the I2C writes above

    (void)mimic.getStatus();                       // Clear the error flags of the transition
    delay(1000);                                   // No I2C traffic: load of the mode itself
    uint16_t steady = mimic.getCpuCycles();
    bool miss = (mimic.getStatus() & MIMIC_STATUS_DEADLINE_MISS) != 0;

    char line[56];
    snprintf(line, sizeof(line), "%-20s  %10u  %6u  %s", testModes[i].name, transition, steady,
             miss ? "MISS" : "ok");
    Serial.println(line);

    mimic.setOutputOffset(0.0f);   // Reset the offset of OUTPUT_OFFSET
    delay(100);
  }
}

void loop()
{
  mimic.setDecimation(0);
  Serial.println(F("\n=== Decimation N = 0 ==="));
  profile();

  mimic.setDecimation(1);
  Serial.println(F("\n=== Decimation N = 1 ==="));
  profile();

  mimic.setDecimation(0);
  mimic.setBypassDSP();
  Serial.println(F("\n=== Profiling complete ==="));
  while (true) {
    delay(1000);
  }
}
