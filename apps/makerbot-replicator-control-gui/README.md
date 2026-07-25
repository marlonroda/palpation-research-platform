# MakerBot Replicator Control GUI

This repository contains a Python Flask-based web GUI for controlling a MakerBot Replicator 2X / Sailfish printer over a serial connection.

The tool was developed as an experimental motion-control interface for using the printer as a programmable positioning platform. It provides browser-based controls for serial connection, homing, motion commands, speed control, swing motion, and waypoint-based indentation routines.

## Project Overview

The GUI allows a user to connect to a MakerBot-compatible serial device and send movement commands through the makerbot_driver Python package. The interface runs locally in a web browser and communicates with the printer through a selected serial port.

Main features include:

- Serial port auto-detection
- Connect and disconnect controls
- Optional auto-homing / calibration
- X, Y, and Z movement commands
- Feed speed control
- Swing and single movement modes
- Bounce count control for repeated go-return motion
- Waypoint-based indentation routines
- Local browser interface using Flask

## Repository Structure

- src/control_gui.py: Main Python Flask application
- requirements.txt: Python dependency list
- docs/: Reserved for future documentation
- README.md: Project documentation
- .gitignore: Ignore rules for temporary files

## Hardware Context

The code was written for:

- MakerBot Replicator 2X
- Sailfish firmware
- Serial communication over USB
- Python makerbot_driver
- Python pyserial
- Local Flask web interface

The motion constants and safety limits in the script are tuned for a Replicator 2 / Replicator 2X-style machine and should be reviewed before use with any other system.

## Safety Warning

This software sends motion commands to physical hardware. Before running the GUI, make sure the printer is clear of obstacles, the motion limits are appropriate, and the emergency stop or power switch is accessible.

Use this code at your own risk. Always verify the configured motion limits, homing behavior, feed speed, and target coordinates before executing movements.

## How to Run

This project was written for Python 2.7.

Create and activate a Python 2 environment, then install the dependencies:

pip install -r requirements.txt

Run the GUI:

python src/control_gui.py

Then open the local web interface in your browser:

http://127.0.0.1:5000

## Notes

The script contains hardware-specific constants, including:

- Serial baud rate
- X/Y/Z steps per millimeter
- Motion feed speed
- Homing wait times
- Soft motion limits
- Dwell times for indentation routines

These values should be checked and adjusted for the specific printer and experimental setup.

## Possible Improvements

- Port the code from Python 2.7 to Python 3
- Add configuration through a separate YAML or JSON file
- Add clearer safety interlocks before motion execution
- Add logging of motion commands
- Add experiment presets for repeated indentation routines
- Add screenshots of the web interface
- Refactor the Flask template into separate HTML/CSS files

## Author

Doga Ozbek
