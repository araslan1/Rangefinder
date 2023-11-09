# Rangefinder

Checkpoint 1:

Splash screen with your name displayed at start.
"Acquire" button triggers a range measurement.
Rangefinder measures the range to an object with 0.1 cm precision.
Distance displayed on LCD.

Checkpoint 2:
Features from Checkpoint 1 plus:
"Adjust" button selects which distance threshold to adjust, indicated on the LCD.
Rotary encoder adjusts both thresholds within the range of 1 to 400 cm.
Threshold settings stored in EEPROM, retrieved on Arduino restart.
LED changes color based on local range and thresholds.

Checkpoint 3:
Features from previous checkpoints.
Implementation of a 74HCT125 tri-state buffer for serial communication.
Testing serial interface with oscilloscope.
Distance data transmitted and received between rangefinders.
Buzzer sounds if remote range is below the remote threshold.
