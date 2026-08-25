# ev-charger

An ESP32-based EV charger (EVSE) implementing the [IEC 61851](https://en.wikipedia.org/wiki/IEC_61851) Control
Pilot protocol: it generates the CP square wave, senses the voltage the car pulls it down to, drives a
contactor once negotiation succeeds, and is controlled and monitored over MQTT.

**This switches mains power into a vehicle. It has no independent safety certification. Build and use
entirely at your own risk.**

## Hardware

- ESP32-WROOM-32U
- KiCad schematic/PCB: [`pcb/charger/`](pcb/charger/)
- CP line: +-12V PWM generated via two optocouplers (`PC817`), sensed back through a resistor divider into
  the ADC. Design sims for that front-end are in [`pcb/charger/simulations/`](pcb/charger/simulations/).
- A 12V solid-state relay enables the CP line driver; a second relay closes the contactor once charging starts

## How it works

`main/main.c` runs a state machine (`charger_state_t`):

| State | CP output | Meaning |
|---|---|---|
| `0` INITIALIZING | 0V | booted, no command received yet |
| `1` OFF | 0V | idle, no car |
| `2` WAIT_CAR | steady +12V | ready, waiting for a vehicle to pull CP down |
| `3` NEGOTIATE | +-12V PWM | vehicle connected, advertising max current |
| `4` CHARGING | +-12V PWM, contactor closed | actively charging |
| `5` ERROR | 0V | a fault was detected - see below |

A 1kHz hardware timer (`gptimer`) drives the CP PWM and samples the ADC at the midpoint of each half-cycle,
classifying the line into IEC 61851 states A-F. A task-level event handler debounces those readings and
drives the state machine above.

### MQTT interface

| Topic | Direction | Payload |
|---|---|---|
| `ev_charger/control` | subscribe | `OFF` or `0` to stop; an integer `6`-`24` to request that many amps |
| `ev_charger/state` | publish (retained) | the numeric `charger_state_t` value from the table above |

Sending `OFF` or `0` also clears a latched ERROR state, so it doubles as a fault reset - no need to power
cycle the board. Any other invalid command (out-of-range current, garbage input) puts the charger into
ERROR.

If you want the charger to resume automatically after a power outage without republishing manually,
publish your control command with the MQTT **retain** flag set - the broker will replay it as soon as the
ESP32 reconnects and subscribes.

### Status LED

A single LED on GPIO2 encodes state so you can read charger status without a laptop:

| State | Pattern |
|---|---|
| INITIALIZING | fast blink, 200ms/200ms |
| OFF | brief pulse every 2s (heartbeat) |
| WAIT_CAR | slow breathing, ~3s cycle |
| NEGOTIATE | fast blink, 100ms/100ms |
| CHARGING | solid on |
| ERROR | *N* short pulses, then a pause, repeating - see fault codes below |

#### ERROR blink codes

| Blinks | Cause |
|---|---|
| 1 | Invalid MQTT command (current outside 6-24A) |
| 2 | No 12V/9V seen while waiting for a vehicle |
| 3 | No -12V half-cycle seen when negotiation should start |
| 4 | -12V half-cycle disappeared mid-charge |
| 5 | CP settled on an unexpected voltage while charging |

## Building

Requires [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) v5.2.1,
target `esp32`.

1. Copy `main/secrets.h.example` to `main/secrets.h` and fill in your WiFi and MQTT details - `secrets.h`
   is gitignored and never committed.
2. If using VS Code with the ESP-IDF extension, copy `.vscode/settings.json.example` to
   `.vscode/settings.json` and adjust the paths/COM port for your machine.
3. Build and flash:

```bash
idf.py build
idf.py -p <PORT> flash monitor
```

## Repo layout

```
main/main.c            firmware
main/secrets.h.example WiFi/MQTT credential template (copy to secrets.h)
pcb/charger/            KiCad schematic and PCB
pcb/charger/simulations/ CP-sensing circuit design sims
```
