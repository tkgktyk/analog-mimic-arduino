/**
 ******************************************************************************
 * @file    AnalogMimicExperimental.h
 * @author  TAKAGI Katsuyuki
 * @brief   Experimental functions of the Analog Mimic module.
 *-----------------------------------------------------------------------------
 * Copyright (C) 2026 TAKAGI Katsuyuki
 *
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 ******************************************************************************
 *
 * Experimental functions are implemented in the firmware, but not yet verified on hardware
 * and in application examples. Their API may change (or they may be removed) before they
 * are released in AnalogMimic.h. The functions are documented in AnalogMimic.h.
 *
 * In 0.3.0 every mode was released; only the test pad is experimental now. New
 * functions will come here first.
 *
 * Usage:
 *   #include <AnalogMimicExperimental.h>
 *   MimicExperimental<Mimic0x> mimic(MIMIC_VDD_5V);   // every function of Mimic0x, too
 *
 * When a function is released, it becomes public in MimicBase and its line here
 * is removed; sketches using MimicExperimental<> keep working unchanged.
 */

#ifndef ANALOG_MIMIC_EXPERIMENTAL_H
#define ANALOG_MIMIC_EXPERIMENTAL_H

#include "AnalogMimic.h"

template <class Base>   // Mimic0x or Mimic1x
class MimicExperimental : public Base
{
public:
  using Base::Base;

  // Test pad
  using Base::setTestPadOutput;
};

#endif // ANALOG_MIMIC_EXPERIMENTAL_H
