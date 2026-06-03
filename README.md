# pimotorcontrol

Raspberry Pi motor control with position feedback. Great for use with automotive wiper motors, AC motors with separate directional windings, and a Raspberry Pi relay hat.

# Background

This was originally written to automate the swinging door on a chicken coop, using a 4-lead automotive windshield wiper motor.

With a Raspberry Pi relay hat controlled by GPIO, two relays are able to change the direction of the motor by swapping the polarity. Many wiper motors have a built-in park switch, which provides a simple way to count motor turns for basic positional feedback.

The codebase has since grown to support analog position feedback via an MCP3008 ADC (for potentiometer-equipped motors), configurable relay wiring modes for AC motors, and a command-line interface flexible enough to drive a variety of motor control use cases.

# Features

- Works with DC reversible motors (relay polarity swap) and AC motors with separate forward/reverse leads (enable + direction relay wiring)
- Two position feedback mechanisms: pulse counting (park switch, hall effect, optical) or analog voltage via MCP3008 ADC (potentiometer)
- Incremental voltage-based positioning: move a configurable voltage delta per step, with fully-open and fully-closed voltage limits
- Position reported as both raw voltage and percentage open, with tolerance-aware clamping at limits
- Configurable relay wiring mode at instantiation — no hardware-specific code changes required
- Configurable GPIO pin numbers at instantiation — supports non-default relay hat pin assignments
- Journal file tracks system state across power cycles and process restarts, enabling recovery from interrupted operations
- Journal writes use explicit disk sync without blocking the motor feedback loop (via `concurrent.futures`)
- Dry-run `fake` mode for testing behavior without engaging GPIO
- Flexible command-line interface covering all motor control modes
- Designed to be imported and used from other code, with all hardware-specific parameters passed at instantiation

# Hardware support

## Relay wiring modes

Two relay wiring modes are supported, selected at instantiation via `relay_mode`:

**`independent` (default)** — original behavior. CH1 and CH2 each connect one motor lead to a supply rail. The motor is stopped by setting both leads to the same potential. Suitable for DC motors such as automotive wiper motors.

**`enable_direction`** — for motors with a shared neutral/common wire and two separate directional leads, such as single-phase AC motors with separate forward and reverse windings. CH1 is the neutral enable relay; CH2 is the direction selector relay. The neutral relay is always dropped before the direction relay changes state, preventing the motor from attempting to reverse under load.

## Position feedback mechanisms

### Pulse-based (original)

The motor's park switch or other feedback mechanism drives a GPIO input pin. The code counts transitions from low to high, stopping after a configured number of pulses. Suitable for wiper motors, motors with hall effect sensors, or any mechanism that produces discrete pulses per revolution.

### Analog voltage via MCP3008 ADC

For motors equipped with a potentiometer, the wiper voltage is read via an MCP3008 8-channel 10-bit ADC over SPI. This enables true analog position feedback: the motor runs until the ADC reading reaches a target voltage, rather than counting pulses.

The MCP3008 connects to the Raspberry Pi's hardware SPI bus:

| MCP3008 pin | Raspberry Pi |
|---|---|
| VDD, VREF | 3.3V or 5V (match to your reference voltage) |
| AGND, DGND | GND |
| CLK | GPIO 11 (SPI0 SCLK) |
| DOUT | GPIO 9 (SPI0 MISO) |
| DIN | GPIO 10 (SPI0 MOSI) |
| CS | GPIO 8 (SPI0 CE0) |

Enable SPI on the Pi via `raspi-config` or `/boot/config.txt` before use.

# Classes and key concepts

## `MCP3008PositionSensor`

A simple wrapper around `spidev` that reads a single MCP3008 channel and converts the 10-bit result to a voltage using the configured reference voltage.

```python
sensor = MCP3008PositionSensor(channel=0, vref=3.3)
voltage = sensor.read()   # returns float in volts
sensor.close()
```

Any object with a `.read()` method returning a float voltage can be substituted, making it straightforward to mock for testing.

## `pimc`

The main motor controller class. Instantiate with your hardware parameters and call action methods or use the CLI.

```python
from pimotorcontrol import MCP3008PositionSensor, pimc

sensor = MCP3008PositionSensor(channel=0, vref=3.3)

motor = pimc(
    journal_filename="pimc_status",
    relay_mode="enable_direction",
    ch1_pin=21,
    ch2_pin=20,
    position_sensor=sensor,
    voltage_fully_closed=1.2,
    voltage_fully_open=3.8,
    voltage_increment=0.4,
    voltage_tolerance=0.1,
    maxtime=5,
)
```

### Pulse-based operation

Pass `open_pulses` and `close_pulses` at instantiation. Use `open()` and `close()` methods or the corresponding CLI actions.

### Analog voltage operation

Pass a `position_sensor`, `voltage_fully_closed`, `voltage_fully_open`, and `voltage_increment`. Use `open_increment()`, `close_increment()`, or the voltage-based CLI actions.

The `voltage_fully_closed` and `voltage_fully_open` values define the physical travel range. `voltage_increment` is how far to move per step. `voltage_tolerance` is the acceptable margin for declaring a target reached — important for aging potentiometers and real-world mechanical systems that cannot reach an exact voltage repeatably.

### Position percentage

`voltage_to_pct(voltage, voltage_fully_closed, voltage_fully_open, voltage_tolerance=0.0)` is a module-level helper that converts a voltage reading to a percentage open (0.0–100.0), clamped at the limits. When `voltage_tolerance` is provided, readings within tolerance of either limit are reported as exactly 0% or 100%, consistent with the at-limit decisions made by `is_fully_open()` and `is_fully_closed()`.

```python
from pimotorcontrol import voltage_to_pct

pct = voltage_to_pct(2.4, voltage_fully_closed=1.2, voltage_fully_open=3.8, voltage_tolerance=0.1)
```

`pimc` also provides `read_position_pct()` as a convenience method for callers that want the percentage without needing a cached voltage reading.

### Direction fault detection

When using analog position feedback, the motor is monitored while running. After a configurable settling period (`direction_settle_seconds`), if the voltage moves more than `direction_fault_threshold` volts in the wrong direction, the motor is stopped and a fault is reported. This catches wiring errors, relay failures, and mechanical issues early.

`direction_fault_threshold` must be greater than `voltage_tolerance`. If it is not, normal ADC noise near the target voltage could trigger false faults. A warning is logged at instantiation if this condition is violated.

### Journaling and recovery

System state is written to a journal file after each operation. On startup with `resume=True`, any interrupted operation is completed before proceeding. This is useful in systems that cannot self-home after a power interruption.

# Command-line interface

`pimotorcontrol.py` can be run directly as a command-line motor controller. `--relay-mode` is required (no default is assumed, as an incorrect value can have physical consequences depending on wiring).

```
python3 pimotorcontrol.py --relay-mode <mode> <action> [options]
```

## Actions

| Action | Description | Key arguments |
|---|---|---|
| `open-pulses` | Open by pulse count | `--open-pulses` |
| `close-pulses` | Close by pulse count | `--close-pulses` |
| `open-seconds` | Open by timed run | `--open-seconds` |
| `close-seconds` | Close by timed run | `--close-seconds` |
| `open-voltage` | Open by voltage delta from current position | `--open-voltage`, `--vref` |
| `close-voltage` | Close by voltage delta from current position | `--close-voltage`, `--vref` |
| `fully-open-voltage` | Drive to fully open voltage position | `--fully-open-voltage`, `--vref` |
| `fully-close-voltage` | Drive to fully closed voltage position | `--fully-closed-voltage`, `--vref` |
| `status` | Print current journal status | |

## Key options

| Option | Description | Default |
|---|---|---|
| `--relay-mode` | `independent` or `enable_direction` | **required** |
| `--vref` | ADC reference voltage (required for voltage actions) | none |
| `--fully-open-voltage` | Voltage at fully open position | none |
| `--fully-closed-voltage` | Voltage at fully closed position | none |
| `--open-voltage` | Positive voltage delta for open-voltage action | none |
| `--close-voltage` | Positive voltage delta for close-voltage action | none |
| `--voltage-tolerance` | Acceptable margin for reaching a target voltage | 0.1V |
| `--direction-fault-threshold` | Wrong-direction movement that triggers a fault | 0.1V |
| `--direction-settle` | Seconds before direction checking begins | 0.1s |
| `--poll-interval` | Seconds between ADC reads | 0.05s |
| `--open-pulses` | Pulse count for open operation | 11 |
| `--close-pulses` | Pulse count for close operation | 11 |
| `--open-seconds` | Seconds to run for open operation | none |
| `--close-seconds` | Seconds to run for close operation | none |
| `--max-time` | Maximum motor runtime per operation | 30s |
| `--ch1-pin` | BCM GPIO pin for relay channel 1 | 26 |
| `--ch2-pin` | BCM GPIO pin for relay channel 2 | 20 |
| `--journal-filename` | Path to journal file | `pimc_status` |
| `--adc-channel` | MCP3008 input channel | 0 |
| `--resume` | Resume any prior interrupted operation | off |
| `--fake` | Dry-run mode, no GPIO interaction | off |
| `--debug` | Verbose debug logging | off |

Note: `--open-voltage` and `--close-voltage` always take a positive absolute value. The direction of movement is implied by the action — the code subtracts the delta internally for close operations.

## CLI examples

Read current ADC voltage (useful for measuring fully-open and fully-closed positions):

```
python3 -c "from pimotorcontrol import MCP3008PositionSensor; s = MCP3008PositionSensor(vref=3.3); print(s.read())"
```

Drive to fully open position using ADC feedback:

```
python3 pimotorcontrol.py \
  --relay-mode enable_direction \
  --vref 3.3 \
  --fully-open-voltage 3.8 \
  --fully-closed-voltage 1.2 \
  --voltage-tolerance 0.1 \
  --ch1-pin 21 --ch2-pin 20 \
  fully-open-voltage
```

Open by a voltage increment from current position:

```
python3 pimotorcontrol.py \
  --relay-mode enable_direction \
  --vref 3.3 \
  --fully-open-voltage 3.8 \
  --fully-closed-voltage 1.2 \
  --open-voltage 0.4 \
  --ch1-pin 21 --ch2-pin 20 \
  open-voltage
```

Open by pulse count, dry-run mode:

```
python3 pimotorcontrol.py --relay-mode independent --fake --open-pulses 5 open-pulses
```

Open by timed run (useful for AC motors without position feedback during initial testing):

```
python3 pimotorcontrol.py --relay-mode enable_direction --ch1-pin 21 --ch2-pin 20 --open-seconds 2 open-seconds
```

Resume any interrupted operation, then close:

```
python3 pimotorcontrol.py --relay-mode independent --resume --close-pulses 3 close-pulses
```

# Design notes

## Tolerance and direction fault threshold interaction

`voltage_tolerance` and `direction_fault_threshold` interact in a subtle but important way. If `direction_fault_threshold` is less than or equal to `voltage_tolerance`, ADC noise near the target voltage (which is within tolerance and should trigger success) could trigger a direction fault before the success check runs. A warning is logged at instantiation if this condition is detected. Setting `direction_fault_threshold` somewhat larger than `voltage_tolerance` is recommended.

## Settling period

After the motor starts, a short settling period (`direction_settle_seconds`) passes before direction fault checking begins. This accounts for relay contact bounce, motor startup lag, and initial ADC noise. The settling period should be greater than `poll_interval_seconds` so that at least one ADC sample is taken during settling before fault checking starts.

## Journal file

The journal file records the current system state as a plain text string: `open`, `closed`, `opening N` (N pulses remaining), `closing N`, or `failed opening`/`failed closing`. It is synced to disk explicitly after each write to reduce the risk of data loss on power interruption.

# Dependencies

- `RPi.GPIO` — GPIO control
- `spidev` — SPI communication with MCP3008 (required for analog position feedback only)

Both are typically available on Raspberry Pi OS. SPI must be enabled via `raspi-config` or `/boot/config.txt` for MCP3008 use.

# Possible future work

- Unified abstraction for move completion feedback (pulse-based and voltage-based share a common structure that could be generalized)
- Interrupt-driven position feedback for momentary switches (versus polling)
- Configuration file support for persistent hardware parameters
- Additional position feedback mechanisms (limit switches as sentinel voltage values via the existing `position_sensor` interface)
