# Palpation Research Platform

This repository serves as a centralized suite of research tools for palpation robotics. It contains a collection of applications used for motion control and sensory data capture.

## Repository Structure

Following best practices for monorepo structure, individual projects are contained within the `apps/` directory:

- **[`apps/esp32-quadrant-imu-gui`](apps/esp32-quadrant-imu-gui)**: A Python 3 Tkinter desktop application for live data capture and visualization from an ESP32-based quadrant pressure sensor and IMU system.
- **[`apps/makerbot-replicator-control-gui`](apps/makerbot-replicator-control-gui)**: A Python 2.7 Flask-based web GUI for controlling a MakerBot Replicator 2X as a programmable positioning platform.

Please refer to the individual `README.md` files within each application's directory for specific setup, calibration, and usage instructions.

## Global Setup

As this repository contains both Python 2 and Python 3 projects, it is highly recommended to manage dependencies using separate virtual environments for each application as detailed in their respective documentation.

## Author

Doga Ozbek
