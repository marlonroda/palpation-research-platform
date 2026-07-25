# Palpation Research Platform

> **A unified software and firmware suite for robotic palpation research.**

This repository provides a complete environment for experimental data capture and visualization. It leverages a **MakerBot Replicator 2X** as a programmable positioning stage to perform precise mechanical indentations, while simultaneously capturing live pressure and kinematic data using an **ESP32-based quadrant sensor and IMU**.

---

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Repository Structure](#repository-structure)
- [System Architecture & Environments](#system-architecture--environments)
  - [1. MakerBot Motion Control (Python 2)](#1-makerbot-motion-control-python-2)
  - [2. ESP32 Sensor Data Capture (Python 3)](#2-esp32-sensor-data-capture-python-3)
- [Data Output Policy](#data-output-policy)


---

## Hardware Requirements

To utilize this platform, the following hardware is required:
- **MakerBot Replicator 2X** (Running Sailfish firmware)
- **ESP32 Microcontroller**
- **Quadrant Pressure Sensor Array**
- **Inertial Measurement Unit (IMU)**

> [!NOTE]
> Supporting CAD models and electronics board designs are documented in [`docs/hardware-designs.md`](docs/hardware-designs.md).

---

## Repository Structure

The codebase is unified into a standard software project structure, separating source code, firmware, and dependencies:

- **`src/`**: High-level applications.
  - `esp32_imu/`: Python desktop application for visualizing pressure and IMU data.
  - `makerbot_control/`: Flask-based web interface to control the MakerBot positioning platform.
- **`firmware/`**: C++ code (`.ino`) for the ESP32 microcontroller, responsible for reading analog sensors and streaming serial data.
- **`docs/`**: Hardware design references and additional project documentation.
- **`requirements/`**: Dependency files mapped to their respective Python environments.
- **`images/figures/`**: Directory reserved for architectural diagrams, UI screenshots, and data plots.

---

## System Architecture & Environments

Because this platform interfaces with legacy MakerBot systems alongside modern data science libraries, **it requires two separate Python environments** to run simultaneously.

### 1. MakerBot Motion Control (Python 2)

This subsystem acts as the motion controller, exposing a local web interface to send G-code and waypoints to the Replicator 2X.

#### Prerequisites
- **Python 2.7**
- USB connection to the MakerBot Replicator 2X

#### Setup & Installation
Create and activate a virtual environment, then install dependencies:

```bash
# Create and activate environment (Linux/macOS)
virtualenv -p python2.7 venv-makerbot
source venv-makerbot/bin/activate

# Install dependencies
pip install -r requirements/requirements-makerbot-py2.txt
```

#### Usage
> [!WARNING]
> This software sends live motion commands to physical hardware. Ensure the printer is clear of obstacles and motion limits are appropriate for your specific setup before executing movements.

```bash
# Launch the local web server
python src/makerbot_control/control_gui.py
```
Open a web browser and navigate to `http://127.0.0.1:5000` to access the motion control dashboard. Here you can home the device, set feed speeds, and configure waypoint-based indentation routines.

---

### 2. ESP32 Sensor Data Capture (Python 3)

This subsystem connects to the ESP32, captures the 10-column serial stream (4x quadrant pressure, 3x accelerometer, 3x gyroscope), and renders a live 2D/3D pressure heatmap and center-of-pressure visualization.

#### Prerequisites
- **Python 3.x**
- **Arduino IDE** (for flashing firmware)
- USB connection to the ESP32

#### Setup & Installation
Flash the ESP32 firmware located at `firmware/esp32_quadrant_imu_firmware/esp32_quadrant_imu_firmware.ino` using the Arduino IDE.

Then, set up the Python environment:

```bash
# Create and activate environment (Linux/macOS)
python3 -m venv venv-esp32
source venv-esp32/bin/activate

# Install dependencies
pip install -r requirements/requirements-esp32-py3.txt
```

#### Usage
Before launching, you must specify your serial port. Open `src/esp32_imu/gui_esp32_capture.py` and modify the `PORT` variable at the top of the file to match your hardware (e.g., `COM3` on Windows, or `/dev/tty.usbserial-0001` on macOS).

```bash
# Launch the data capture GUI
python src/esp32_imu/gui_esp32_capture.py
```
Use the GUI's calibration buttons (no-touch and maximum-pressure) to initialize the sensor array before running indentation routines via the MakerBot control interface.

---

## Data Output Policy

No captured experimental data is checked into this repository. 

The ESP32 GUI exports time-series sensor data and metadata as NumPy `.npz` files directly to the local execution directory. To prevent accidental commits of large datasets, the `.gitignore` is configured to exclude `*.npz`, `*.npy`, `*.csv`, `*.mat`, and `*.log` files.


