# MATLAB

MATLAB code for the 2-DOF drawing robot arm.

This folder contains:
1) A **GUI** that communicates with the Arduino over Serial and plots the motion
2) **Kinematics + analysis** scripts (FK/IK/Jacobian/manipulability) used for offline evaluation and visualisations

---

## Folder structure

Recommended layout:

- `gui/`
  - `GUI.m` — MATLAB GUI: enter a target point (x, y), send to Arduino, plot desired vs actual path

- `control/`
  - `robot_arm_control.m` — offline sandbox for experimenting with control/trajectories (no hardware required)

- `kinematics/`
  - `inverse_kinematics.m` — analytical IK used by the GUI (target x,y → joint angles)
  - `forward_kinematics_version_2.m` — FK used by the GUI (joint angles → x,y for plotting)
  - `forwardKinematics.m` — FK utility (often used by analysis scripts; returns transform / position)
  - `ik_jacobian.m` — Jacobian for 2-link planar model
  - `ik_differential.m` — differential IK update step
  - `Manipulability.m` — manipulability plots + ellipsoids (Jacobian Transpose / DLS)

---

## File entry points (what you actually run)

### Run this (hardware required)
- `gui/GUI.m`  
  Starts the GUI, connects to the Arduino over Serial, sends commands, and plots results.

### Run these (offline / no hardware required)
- `control/robot_arm_control.m`  
  Sandbox for testing trajectories/control ideas locally.
- `kinematics/Manipulability.m`  
  Generates manipulability metrics and ellipsoid plots for Jacobian Transpose and/or DLS (depending on your script settings).

### Helper functions (not run directly)
These are called by the GUI or by the analysis scripts:
- `kinematics/inverse_kinematics.m`
- `kinematics/forward_kinematics_version_2.m`
- `kinematics/forwardKinematics.m`
- `kinematics/ik_jacobian.m`
- `kinematics/ik_differential.m`

---

## Requirements
- MATLAB (R2020b+ recommended)
- Serial support (uses `serialport`, built-in)

---

## Quickstart (Hardware + GUI)

1) Upload the Arduino firmware first  
See: `firmware/README.md`

2) Open MATLAB and set the current folder to `matlab/`

3) Add this folder + subfolders to your MATLAB path:
```matlab
addpath(genpath(pwd))
```
4) Edit the COM port in gui/GUI.m (it’s hardcoded):
```
matlab
s = serialport("COM11", 9600);
```
Change "COM11" to the port your Arduino shows up as.

5) Run the GUI:
```
matlab
GUI
```
---
## What the GUI does (high level)
- Takes a desired end-effector target (x, y)
- Computes joint targets using inverse_kinematics.m
- Sends joint targets over Serial to the Arduino firmware
- Reads measured joint angles back from the Arduino
- Computes the measured end-effector position using forward_kinematics_version_2.m
- Plots desired vs actual
---
## Serial protocol (matches the firmware)
### MATLAB → Arduino (command)
The Arduino parses a command that starts with a character and ends with ;:
`C<theta1>,<theta2>;`
Example:
`C45.00,10.00;`
### Arduino → MATLAB (telemetry)
Arduino streams lines like:
`c<i>,<theta1_meas>,<theta2_meas>`
Example:
`c12,44.63,9.92`
Baud rate: **9600**
---
## Offline analysis scripts (no hardware required)
### Manipulability / ellipsoids
Run:
```matlab
Manipulability
```
This generates manipulability curves and ellipsoids for Jacobian Transpose and/or Damped Least Squares (DLS) depending on your script settings.

### Differential IK utilities
- `ik_jacobian.m` — compute Jacobian at a configuration
- `ik_differential.m` — compute a single joint update for a desired end-effector delta
- `forwardKinematics.m` / `forward_kinematics_version_2.m` — compute end-effector position from joint angles
---
## Units + conventions (important)
- Keep units consistent across GUI input, IK, and FK:
  - If you input mm, link lengths should be mm.
  - If you input m, link lengths should be m.
- Be consistent about degrees vs radians:
  - If firmware expects degrees, keep MATLAB outputs in degrees (or convert explicitly).
A “wrong looking” plot is almost always unit mismatch.
---
## Troubleshooting
- **GUI runs but nothing moves**
  - Check Arduino firmware is uploaded and running
  - Confirm COM port and baud rate (9600)
  - Close Arduino IDE Serial Monitor (it can lock the port)
- **MATLAB errors: “Undefined function …”**
  - Make sure you ran:
  ```matlab
  addpath(genpath(pwd))
  ```
  from inside the `matlab/` folder
**- Plot looks mirrored / inverted**
  - Swap sign conventions in IK/FK or adjust joint angle reference (0° direction)
  - Double-check coordinate frame assumptions (x-right/y-up vs x-right/y-down)
---
## See also
- Firmware upload + pin mapping: `firmware/README.md`
- Wiring + parts list: `electronics/`
- Full project write-up: `report/Final_Applied_Robotics_Report.pdf`
