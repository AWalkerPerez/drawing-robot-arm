# Environment

This project is implemented using **MATLAB** and **Arduino** (no Python dependencies required).

## Tested setup
- **OS:** Windows 10/11 (recommended)
- **MATLAB:** R2020b+ (any recent version should work)
- **Arduino IDE:** 2.x
- **Board:** Arduino **Uno R4** (Minima or WiFi)
- **Motor driver:** L298N dual H-bridge
- **Power supply:** 12V DC, 5A (60W)

## MATLAB requirements
- MATLAB with support for `serialport` (built-in)
- No additional toolboxes are required for the provided scripts

### MATLAB path setup
From the `matlab/` folder:
```matlab
addpath(genpath(pwd))
```
### Arduino requirements
- Install Arduino IDE 2.x
- In **Tools** → **Board**, select:
  - Arduino Uno R4 Minima or Arduino Uno R4 WiFi (depending on your board)
- Select the correct **Port**
- Upload: `firmware/pid_controller/pid_controller.ino`

### Serial communication
- Baud rate: **9600**
- The COM port is hardcoded in `matlab/gui/GUI.m` and must be updated to match your machine (e.g., `COM5`, `COM11`, etc.)

### File formats (CAD)
- Editable Fusion 360 exports: `.f3d` (single design) / `.f3z` (assembly)
- Print exports: `.3mf` and/or `.stl`

### Notes / common issues
- Close the Arduino IDE Serial Monitor when running the MATLAB GUI (it can lock the serial port).
- Ensure a **common ground** between the Arduino and motor driver.
---
## Setup (Quickstart)
This is the fastest path from cloning the repo to running a demo.
### 1) Hardware + wiring
**1.** Assemble the robot (see the report for mechanical build context).

**2.** Wire the electronics:
  - See `electronics/README.md`
  - Use `electronics/wiring_diagram.png`
  - Use `electronics/arduino_uno_r4_pinout.png` as a pin reference.

**Power checklist**
- 12V supply powers the L298N (+12V and GND)
- Arduino and L298N share a common GND
- Motors connect to L298N outputs (OUT1/OUT2 and OUT3/OUT4)

### 2) Upload firmware (Arduino)
**1.** Open Arduino IDE.

**2.** Open: `firmware/pid_controller/pid_controller.ino`.

**3.** Select your board (Uno R4 Minima/WiFi) and Port.

**4.** Click **Upload**.

### 3) Run MATLAB GUI
**1.** Open MATLAB.

**2.** Set current folder to `matlab/`.

**3.** Add paths:
```matlab
addpath(genpath(pwd))
```
**4.** Open `matlab/gui/GUI.m` and update the COM port:
```matlab
s = serialport("COM11", 9600);
```
**5.** Run:
```matlab
GUI
```
### 4) Demo
- Enter a target point (x, y) in the GUI and press **Send**
- The GUI sends commands to the Arduino and plots desired vs actual motion
---
### Troubleshooting
- If MATLAB can’t connect to the serial port:
  - Close Arduino IDE Serial Monitor
  - Double-check COM port and baud rate (9600)
- If motors don’t move:
  - Confirm 12V supply is live at the L298N
  - Confirm common ground (Arduino GND ↔ L298N GND)
  - Confirm ENA/ENB and IN1–IN4 wiring matches the firmware pin mapping
