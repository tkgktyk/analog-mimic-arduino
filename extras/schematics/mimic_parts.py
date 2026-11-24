"""Schematic parts for the Analog Mimic examples (schemdraw).

The module and the host boards are drawn as blocks with the pins that the
examples use, not as pictures of the boards. When a pin of the hardware
changes, change it here and regenerate the drawings.

    pip install schemdraw
"""
import schemdraw
import schemdraw.elements as elm

# SVG backend: text stays text (small files, no font warnings)
schemdraw.use("svg")

FONT = "sans-serif"
PIN_SPACING = 1.0


def setup_drawing(**kwargs):
    """A drawing with the common style of the examples."""
    d = schemdraw.Drawing(**kwargs)
    # White background: readable in the dark theme of GitHub too
    d.config(font=FONT, fontsize=12, unit=2.5, lw=1.4, bgcolor="white")
    return d


def mimic(label="Analog Mimic"):
    """Analog Mimic module (Mimic01).

    left (host side), from the top: VCC, GND, SDA, SCL, AOUT
        (level with 5V, GND, SDA, SCL, A0 of uno_r4(), so the wires are straight);
    right (input side), in the middle: the 3-pin analog input header AIN, V, GND
        (anchors AIN, V, GND_S; the same order as sensor3()).

    VCC is the supply input (with reverse-polarity protection). V is a supply OUTPUT
    for a sensor (VCC after the protection): do not supply the module from it. The I2C
    pull-up resistors (10 kOhm) are on the module. The V pad of the SWD header is not
    drawn.
    """
    pins = [  # listed from the bottom
        elm.IcPin(name="AOUT", side="left", anchorname="AOUT"),
        elm.IcPin(name="SCL", side="left", anchorname="SCL"),
        elm.IcPin(name="SDA", side="left", anchorname="SDA"),
        elm.IcPin(name="GND", side="left", anchorname="GND"),
        elm.IcPin(name="VCC", side="left", anchorname="VCC"),
        elm.IcPin(name="GND", side="right", anchorname="GND_S", slot="2/5"),
        elm.IcPin(name="V", side="right", anchorname="V", slot="3/5"),
        elm.IcPin(name="AIN", side="right", anchorname="AIN", slot="4/5"),
    ]
    return elm.Ic(pins=pins, pinspacing=PIN_SPACING, edgepadW=1.0, edgepadH=0.5,
                  leadlen=0.5).label(label, loc="top", fontsize=13)


def uno_r4(label="Arduino UNO R4", extra_pins=()):
    """Arduino UNO R4 (Minima / WiFi) with the pins used by the examples.

    Anchors: A0, SCL, SDA, GND, 5V (right), level with AOUT, SCL, SDA, GND, VCC of
    mimic() so that the wires are straight when the two blocks are side by side.
    extra_pins: more pins on the right side, e.g. ("D2",), listed above A0.
    """
    names = ["A0", "SCL", "SDA", "GND", "5V"]
    pins = [elm.IcPin(name=n, side="right", anchorname=n) for n in names]
    pins += [elm.IcPin(name=n, side="right", anchorname=n) for n in extra_pins]
    return elm.Ic(pins=pins, pinspacing=PIN_SPACING, edgepadW=1.0, edgepadH=0.5,
                  leadlen=0.5).label(label, loc="top", fontsize=13)


def probe(label):
    """An oscilloscope probe point: an open terminal with a label."""
    return elm.Dot(open=True).label(label, loc="right")


def sensor3(label="Sensor"):
    """A 3-pin sensor module (signal, supply, ground) for the analog input header.

    Anchors: OUT, V, GND (left), in the same order as the header of mimic(), so
    that the wires are straight when it is placed to the right of the module.
    """
    pins = [
        elm.IcPin(name="GND", side="left", anchorname="GND"),
        elm.IcPin(name="V", side="left", anchorname="V"),
        elm.IcPin(name="OUT", side="left", anchorname="OUT"),
    ]
    return elm.Ic(pins=pins, pinspacing=PIN_SPACING, edgepadW=0.8, edgepadH=0.5,
                  leadlen=0.5).label(label, loc="top", fontsize=13)

