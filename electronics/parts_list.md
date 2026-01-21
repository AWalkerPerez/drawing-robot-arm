# Bill of Materials (BOM) — Drawing Robot Arm (2-DOF)

Quantities below match the physical robot build in this repo.

## Core electronics
- **1× Arduino Uno R4**
- **1× L298N dual H-bridge motor driver module**
- **2× DC geared motors with integrated potentiometer feedback** (5-wire)
- **1× 12V DC power supply (12V, 5A, 60W)**

## Wiring & connectors (direct-wired)
- **1× USB cable** (PC → Arduino, for programming + serial)
- **1× DC power connector / barrel jack (2.1 mm, 5A rated)** :contentReference[oaicite:1]{index=1}
- **Hook-up wire** (22–26 AWG recommended)
- **Solder + heatshrink** (or electrical tape)
- **Cable ties** (strain relief / cable management)

## Notes
- Ensure **common ground** between the 12V supply, L298N GND, and Arduino GND.
- Motors are powered from the **12V rail** via the L298N.
- Motor potentiometer feedback lines connect to Arduino analog inputs (see `firmware/README.md` for pin mapping).
