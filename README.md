# ODrive 3.6 CAN Emulator

ESP32-S3 firmware that emulates two ODrive 3.6 axes over CAN for telescope ALT/AZM control software testing.

This emulator targets the ODrive 3.6 / firmware v0.5.6 CAN-simple heartbeat layout and implements a lightweight motion model, encoder feedback, error reporting, and a configurable notch filter command.

## Hardware

- Board: ESP32-S3 DevKitC-1
- CAN TX pin: GPIO 17
- CAN RX pin: GPIO 18
- CAN bitrate: 500 kbit/s
- Emulated nodes:
  - ALT axis: node ID `0`
  - AZM axis: node ID `1`

An external CAN transceiver is still required between the ESP32 TWAI pins and the CAN bus.

## Build and Flash

This project uses PlatformIO with the Arduino framework.

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

The active PlatformIO environment is `esp32-s3-devkitc-1` in `platformio.ini`.

## What It Emulates

- ODrive heartbeat messages every 200 ms
- Encoder estimate messages every 200 ms while an axis is in closed-loop control
- Position-command response with a slew-limited rigid-body move plus a damped flexible resonance term
- IQ, temperature, and bus voltage/current telemetry
- Axis, motor, encoder, and controller error reporting
- Clear-errors and estop behavior
- Runtime notch filter coefficient updates over CAN

## CAN Message Format

CAN IDs use the standard ODrive CAN-simple format:

```text
can_id = (node_id << 5) | command_id
```

### Heartbeat

Command ID: `0x01`

Payload:

| Byte(s) | Field |
| --- | --- |
| 0-3 | `axis_error` (`uint32_t`) |
| 4 | `axis_state` (`uint8_t`) |
| 5 | `motorFlags` (`1` if `motor_error != 0`, else `0`) |
| 6 | `encoderFlags` (`1` if `encoder_error != 0`, else `0`) |
| 7 | `controllerFlags`, bit 0 = controller error present, bit 7 = `trajectory_done` |

This matches the ODrive 3.6 v0.5.6 `CANSimple::send_heartbeat()` packing.

## Supported Commands

| Command | ID | Notes |
| --- | --- | --- |
| `CMD_GET_VERSION` | `0x00` | Defined, not currently handled |
| `CMD_HEARTBEAT` | `0x01` | Sent periodically by emulator |
| `CMD_ESTOP` | `0x02` | Forces `AXIS_STATE_ERROR` and sets `axis_error` bit 0 |
| `CMD_GET_MOTOR_ERROR` | `0x03` | Returns 32-bit motor error mask |
| `CMD_GET_ENCODER_ERROR` | `0x04` | Returns 32-bit encoder error mask |
| `CMD_SET_AXIS_STATE` | `0x07` | Supports IDLE / CLOSED_LOOP_CONTROL state transitions |
| `CMD_GET_ENCODER` | `0x09` | Returns position and velocity floats |
| `CMD_SET_CONTROLLER` | `0x0B` | Parsed and logged |
| `CMD_SET_INPUT_POS` | `0x0C` | Sets target position, clears `trajectory_done` |
| `CMD_SET_INPUT_VEL` | `0x0D` | Parsed and logged |
| `CMD_SET_INPUT_TORQUE` | `0x0E` | Parsed and logged |
| `CMD_SET_LIMITS` | `0x0F` | Parsed and logged |
| `CMD_GET_IQ` | `0x14` | Returns `iq_setpoint`, `iq_measured` |
| `CMD_GET_TEMP` | `0x15` | Returns FET and motor temperatures |
| `CMD_GET_BUS_VI` | `0x17` | Returns bus voltage and current |
| `CMD_CLEAR_ERRORS` | `0x18` | Clears all injected/runtime errors and returns axis to IDLE |
| `CMD_SET_LINEAR_COUNT` | `0x19` | Directly sets position and resets sim/notch history |
| `CMD_SET_POS_GAIN` | `0x1A` | Parsed and logged |
| `CMD_SET_VEL_GAINS` | `0x1B` | Stores `vel_gain` and `vel_int_gain` |
| `CMD_GET_CONTROLLER_ERROR` | `0x1D` | Returns 32-bit controller error mask |
| `CMD_SET_NOTCH` | `0x1E` | Updates notch coefficients and enable state |

## Motion Model

When an axis is in `AXIS_STATE_CLOSED_LOOP_CONTROL`, `SET_INPUT_POS` updates a target position and the emulator simulates measured motion in `updateSimulatedMotor()`:

- A slew-limited rigid-body step toward the target using `SIM_SLEW_RATE_TURNS_PER_SEC`
- A second-order flexible resonance term using per-axis natural frequencies and damping
- Optional notch filtering applied to the commanded target before the simulated plant

Key simulation constants are near the top of `src/main.cpp`:

- `SIM_SLEW_RATE_TURNS_PER_SEC`
- `SIM_POSITION_TOLERANCE_TURNS`
- `SIM_RES_FREQ_ALT_HZ`
- `SIM_RES_FREQ_AZM_HZ`
- `SIM_RES_DAMPING`
- `SIM_RES_DRIVE_GAIN`
- `ALT_MIN_TURNS`, `ALT_MAX_TURNS`
- `AZM_MIN_TURNS`, `AZM_MAX_TURNS`

`trajectory_done` is set false when a new position command arrives, then set true once position error and velocity fall below the settle thresholds.

## Notch Filter Command

`CMD_SET_NOTCH` expects a 5-byte payload:

```text
byte 0   = coefficient index
bytes 1-4 = float32 value
```

Supported indices:

| Index | Meaning |
| --- | --- |
| 0 | pending `b0` and marks pending coefficients valid |
| 1 | pending `b1` |
| 2 | pending `b2` |
| 3 | pending `a1` |
| 4 | pending `a2` |
| 5 | enable/disable notch (`0.0f` = disable, nonzero = enable); if pending coefficients are valid, they are committed before enabling |

Notch filter history is reset whenever the filter is enabled/disabled or `SET_LINEAR_COUNT` is received.

## Compile-Time Error Injection

Fault injection is configured directly in `src/main.cpp`:

```cpp
#define ALT_INJECT_ERRORS      1
#define AZM_INJECT_ERRORS      1

#define ALT_MOTOR_ERROR        0x00000001
#define ALT_ENCODER_ERROR      0x00000001
#define ALT_CONTROLLER_ERROR   0x00000002

#define AZM_MOTOR_ERROR        0x00000002
#define AZM_ENCODER_ERROR      0x00000002
#define AZM_CONTROLLER_ERROR   0x00000004
```

At startup, if a node's inject switch is nonzero, the emulator loads that axis's motor, encoder, and controller error masks. If any of those masks are nonzero, it also sets:

- `axis_error = AXIS_ERROR_INVALID_STATE`
- `axis_state = AXIS_STATE_ERROR`

`CMD_CLEAR_ERRORS` clears all four error fields and sets the axis back to `AXIS_STATE_IDLE`. Injected startup faults are not automatically re-applied until the firmware is restarted.

Use `src/ODriveEnums.h` as the reference for realistic motor, encoder, and controller error bitmasks.

## Notes and Limitations

- This is an emulator, not a full ODrive firmware model. Several commands are accepted only for logging and do not yet alter control behavior.
- `GET_VERSION` is defined but not implemented in the current switch statement.
- `SET_INPUT_VEL`, `SET_INPUT_TORQUE`, `SET_LIMITS`, and `SET_POS_GAIN` are currently parsed/logged but do not drive the simulated plant.
- Axis travel limits are defined but not currently enforced in the motion update path.
- Verbose serial logging is enabled by `kVerboseCommandLog = true` in `src/main.cpp`.

