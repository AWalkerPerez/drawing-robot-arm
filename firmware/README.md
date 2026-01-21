# Firmware

Arduino firmware for the 2-DOF drawing robot arm.  
It receives joint angle commands from MATLAB over Serial, runs a PID controller for each motor, and streams back the measured joint angles.

## Location
- Main sketch: `firmware/pid_controller/pid_controller.ino`

## Hardware used
- **Board:** Arduino **Uno R4**
- **Motor driver:** L298N dual H-bridge
- **Motors:** 2× DC geared motors with **quadrature encoder outputs** (A/B)

## How to upload
1. Open `pid_controller.ino` in the Arduino IDE.
2. Select your board:
   - *Arduino Uno R4 Minima* or *Arduino Uno R4 WiFi* (whichever you have)
3. Select the correct **Port**.
4. Click **Upload**.
5. Open Serial Monitor (9600 baud) if you want to view telemetry.

## Pin mapping (as used in the code)

### Motor encoders (quadrature)
| Motor | Encoder A | Encoder B |
|------:|-----------|-----------|
| M1 | A3 | A2 |
| M2 | A1 | D11 |

> Note: Encoder A pins use interrupts via `attachInterrupt(...)`. If you change boards/pins, make sure the new pins support interrupts.

### L298N direction pins
| Motor | IN1 | IN2 |
|------:|-----|-----|
| M1 | D4 | D5 |
| M2 | D7 | D8 |

### L298N enable (PWM) pins
| Motor | ENA / ENB | Arduino pin |
|------:|------------|-------------|
| M1 | ENA | D6 |
| M2 | ENB | D9 |

## PID gains (where to change them)
At the top of `pid_controller.ino` you’ll see:

- `Kp_m1`, `Ki_m1`, `Kd_m1`
- `Kp_m2`, `Ki_m2`, `Kd_m2`

Update these values to tune the response.  
There is also an integral windup clamp:

- `windupGuard`

## Serial protocol (MATLAB ↔ Arduino)

### Command format (MATLAB → Arduino)
The Arduino expects joint targets (in **degrees**) in this format:

`C<theta1>,<theta4>;`

Example:
`C45.00,10.00;`

Notes:
- The leading `C` is required.
- Values are parsed as floats.
- The command must end with `;`

### Telemetry format (Arduino → MATLAB)
The Arduino continuously streams:

`c<i>,<theta1_meas>,<theta4_meas>\r\n`

Example:
`c12,44.63,9.92`

Where:
- `i` is a counter
- `theta1_meas`, `theta4_meas` are the measured joint angles (degrees)

## Using with MATLAB
The MATLAB GUI sends commands in the correct `C..., ...;` format and parses `c...` telemetry.

- GUI script: `matlab/gui/GUI.m`
- **Important:** update the COM port in `GUI.m` (it’s currently hardcoded, e.g. `COM11`).

## Troubleshooting
- **Nothing moves**
  - Confirm L298N has 12V power
  - Confirm **common ground** (Arduino GND ↔ L298N GND)
  - Check ENA/ENB pins are connected to PWM pins (D6, D9)
- **One motor spins the wrong direction**
  - Swap OUT wires for that motor on the L298N, or invert direction logic in code
- **No serial data in MATLAB**
  - Confirm baud rate is **9600**
  - Confirm correct COM port and that Arduino IDE Serial Monitor is closed (it can “steal” the port)
