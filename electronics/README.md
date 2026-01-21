# Electronics

This folder contains the wiring + component notes for the 2-DOF drawing robot arm.

## What’s in this folder
- `parts_list.md` — exact bill of materials (with quantities)
- `wiring_diagram.png` — wiring diagram for the final build
- `arduino_uno_r4_pinout.png` — Arduino Uno R4 pin reference
- `l298n_pinout.png` — L298N reference pinout (if included)
- `motor_dimensions.png` — motor dimensions (if included)

## System overview
- **Controller:** Arduino Uno R4
- **Motor driver:** L298N dual H-bridge
- **Motors:** 2× DC geared motors with potentiometer feedback
- **Power:** 12V DC supply (12V, 5A)

## Wiring summary (high level)
### Power
1. Connect the **12V supply** to the L298N:
   - Supply **+12V** → L298N `+12V`
   - Supply **GND** → L298N `GND`
2. Connect **Arduino GND** to **L298N GND** (common ground).

> Important: Motors must be powered from the 12V rail through the L298N. Do **not** power motors from the Arduino 5V pin.

### Motors
- Motor 1 → L298N `OUT1` / `OUT2`
- Motor 2 → L298N `OUT3` / `OUT4`

If a motor spins the wrong way, either swap the two motor wires on the output, or invert direction in firmware.

## Arduino pin reference
Use `arduino_uno_r4_pinout.png` as a quick reference for:
- PWM-capable pins (for ENA/ENB)
- Analog inputs (for potentiometer feedback)
- Power pins (5V / GND)

### Control signals (Arduino → L298N)
The L298N uses:
- `IN1, IN2` for Motor 1 direction
- `IN3, IN4` for Motor 2 direction
- `ENA, ENB` for motor speed (PWM)

**Exact pin mapping:** see `firmware/pid_controller/pid_controller.ino` (recommended source of truth).
If you want, you can also document it here once final:

| Signal | L298N pin | Arduino pin |
|-------:|-----------|-------------|
| Motor 1 PWM | ENA | (fill) |
| Motor 1 dir | IN1 | (fill) |
| Motor 1 dir | IN2 | (fill) |
| Motor 2 PWM | ENB | (fill) |
| Motor 2 dir | IN3 | (fill) |
| Motor 2 dir | IN4 | (fill) |

### Potentiometer feedback (per motor)
Each motor has a potentiometer used for position feedback:
- Pot **VCC** → Arduino **5V**
- Pot **GND** → Arduino **GND**
- Pot **wiper/output** → Arduino **analog input** (e.g., `A0` for motor 1, `A1` for motor 2)

Again, check the `.ino` for the exact analog pins used.

## L298N module notes (common gotchas)
- Many L298N boards have a **5V regulator jumper** (often labelled `5V_EN`).
  - Safest setup: power the L298N logic from the Arduino 5V and only use the 12V rail for motors.
  - If you change jumper settings, double-check the module’s labels so you don’t back-power the Arduino.

- The L298N can get warm. Make sure the heatsink has airflow.

## Troubleshooting
- **Motors don’t move at all**
  - Check common ground (Arduino GND ↔ L298N GND)
  - Confirm 12V is present at the driver input
  - Confirm ENA/ENB are enabled (PWM pin not floating)

- **One motor moves, the other doesn’t**
  - Swap motor outputs to isolate whether it’s wiring vs motor vs code
  - Check that the second motor’s ENB/IN3/IN4 pins match firmware

- **Noisy / jittery feedback**
  - Ensure pot ground is solid
  - Keep pot wires away from motor power wires where possible
  - Shorter wiring helps (direct wiring is good)

## See also
- Firmware: `firmware/`
- MATLAB control/GUI: `matlab/`
- Full write-up: `report/Final_Applied_Robotics_Report.pdf`

