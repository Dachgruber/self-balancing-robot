# self-balancing-robot
Arduino code and building materials needed for the self-balancing-robot project for ElektronikFuerLA SoSe23 at CAU Kiel

## Hardware
The currently used hardware (MK1):
  -Arduino Nano (ATMega68p)
  -GY-521 Internal Measuring Unit
  -2x A4988 Stepper Motor Driver Boards
  -2x NEMA-17 0.48Nm Stepper Motors with D-Shaft
  -2200 mAh 40C 11.1V LiPo
  - 2 push buttons and 1 switch
  condensators, LEDs, wires...

Frame is build out of 10mm MDF-Plywood and 3mm craft wood sandwich plating
## Software

The code provided includes some testing programmes used at the R&D stage of development.

The finished piece of software will include:
  - a sensoring unit for measuring the current position
  - a calibration programm for calibrating said sensoring unit
  - a PID controller for controlling the two stepper motors
  - some form of remote control feature
and some more quality of life features
