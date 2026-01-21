# Small Arm, Big Impact — 2DOF Drawing Robotic Arm

A low-cost 2-degree-of-freedom drawing arm designed for precise motion in a ~156mm x 156mm workspace.
Built with micro servos, an Arduino Uno, and an L298N driver, with analytical IK + PID control and a MATLAB GUI.

## What’s in this repo
- `firmware/` — Arduino code to receive target points and actuate motors
- `matlab/` — GUI + kinematics / plotting scripts
- `cad/` — CAD + printable files (STL/3MF) and assembly notes
- `electronics/` — wiring diagram + parts list
- `report/` — full project report (methods + results)

## Quickstart (typical workflow)
1. Print parts from `cad/`
2. Assemble + wire using `electronics/`
3. Upload Arduino code from `firmware/arduino/`
4. Run the MATLAB GUI in `matlab/gui/` to send (X,Y) targets

## Key methods implemented
- Forward + analytical inverse kinematics
- Differential IK options (Jacobian Transpose / DLS)
- PID tuning per motor
- MATLAB GUI that sends coordinates and plots feedback

## Results
See `report/Final_Applied_Robotics_Report.pdf` for the full evaluation.
