# CP-sensing circuit simulations

[Falstad CircuitJS](https://www.falstad.com/circuit/) simulation exports from designing the
Control Pilot sensing/generation front-end - the analog circuit that turns the ESP32's
optocoupler-driven +-12V PWM into the CP line, and feeds it back through a resistor divider
into the ADC that `get_cp_line_state()` in [main.c](../../../main/main.c) reads.

Open any of them by pasting the file contents into
[the CircuitJS simulator](https://www.falstad.com/circuit/circuitjs.html) (File > Import from Text).

Numbered in design order; `04-...-final` is the version the built circuit is based on.
