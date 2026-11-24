"""Wiring of the library examples: Arduino UNO R4 and one Analog Mimic.

The DAC of the UNO R4 (A0) makes the test signal for the module input (AIN);
an oscilloscope shows the input (CH1) and the module output AOUT (CH2).
"""
import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from mimic_parts import setup_drawing, mimic0x, uno_r4, probe
import schemdraw.elements as elm

with setup_drawing(show=False) as d:
    uno = d.add(uno_r4())
    m = d.add(mimic0x().anchor("SCL").at((uno.SCL[0] + 6, uno.SCL[1])))
    for a, b in [("SCL", "SCL"), ("SDA", "SDA"), ("GND", "GND"), ("5V", "VCC")]:
        d.add(elm.Wire("-").at(getattr(uno, a)).to(getattr(m, b)))

    # Test signal: A0 under the module and up to AIN; CH1 on it
    y = uno.A0[1] - 1.5
    x1 = uno.A0[0] + 1.0
    xr = m.AIN[0] + 1.5
    d.add(elm.Line().at(uno.A0).to((x1, uno.A0[1])))
    d.add(elm.Line().to((x1, y)))
    d.add(elm.Line().to((xr, y)))
    d.add(elm.Line().to((xr, m.AIN[1])))
    d.add(elm.Line().to(m.AIN))
    d.add(elm.Dot().at((x1 + 1.0, y)))
    d.add(elm.Line().at((x1 + 1.0, y)).down(1.0))
    d.add(probe("CH1 (input)"))

    # Output: AOUT to CH2
    d.add(elm.Line().at(m.AOUT).to((m.AOUT[0] - 1.5, m.AOUT[1])))
    d.add(elm.Line().down(0.8))
    d.add(probe("CH2 (output)"))

    out = pathlib.Path(__file__).with_suffix(".svg")
    d.save(str(out))
print("wrote", out)
