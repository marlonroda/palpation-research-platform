# Palpation Research Platform

This repository contains the software and firmware suite for a robotic palpation research platform. The system uses a MakerBot Replicator 2X as a programmable positioning stage to perform mechanical indentations, while simultaneously capturing live pressure and kinematic data using an ESP32-based quadrant sensor and IMU.

By combining precise motion control with high-resolution sensory feedback, this suite provides a complete environment for experimental data capture and visualization.

---

## Repository Structure

The codebase is unified into a standard software project structure:

- **`src/`**: Contains the source code for the high-level applications.
  - **`esp32_imu/`**: Python desktop application for visualizing the pressure and IMU data.
  - **`makerbot_control/`**: Flask-based web interface to control the MakerBot positioning platform.
- **`firmware/`**: Contains the C++ code for the ESP32 microcontroller, responsible for reading the analog quadrant sensors and the IMU, and streaming data over serial.
- **`docs/`**: Additional documentation, including CAD model references and electronics designs (`hardware-designs.md`).
- **`requirements/`**: Dependency files mapped to their respective Python environments.
- **`images/figures/`**: Directory reserved for architectural diagrams, UI screenshots, and data plots.

---

## Architecture & Environments

Because this platform interfaces with legacy MakerBot systems alongside modern data science libraries, **it requires two separate Python environments** to run simultaneously.

1. **MakerBot Control Node**: Runs on **Python 2.7** (due to the `makerbot_driver` dependencies for Sailfish firmware).
2. **Data Capture GUI**: Runs on **Python 3.x** (to support modern `numpy`, `matplotlib`, and `tkinter` versions).

### 1. MakerBot Motion Control (Python 2)

This subsystem acts as the motion controller, exposing a local web interface to send G-code and waypoints to the Replicator 2X.

#### Requirements (Python 2.7)
Ensure you have a Python 2.7 installation. Create and activate a virtual environment:

```bash
# Example for Linux/macOS
virtualenv -p python2.7 venv-makerbot
source venv-makerbot/bin/activate

# Install dependencies
pip install -r requirements/requirements-makerbot-py2.txt
```

#### Usage
Before running, ensure the MakerBot is connected via USB and powered on.
**Warning**: This software sends live motion commands. Ensure the motion limits are appropriate for your specific setup.

```bash
# Inside the Python 2.7 environment
python src/makerbot_control/control_gui.py
```
Open a browser and navigate to `http://127.0.0.1:5000` to access the motion control dashboard. Here you can home the device, set feed speeds, and configure waypoint-based indentation routines.

### 2. ESP32 Sensor Data Capture (Python 3)

This subsystem connects to the ESP32, captures the 10-column serial stream (4x quadrant pressure, 3x accelerometer, 3x gyroscope), and renders a live 2D/3D pressure heatmap and center-of-pressure visualization.

#### Requirements (Python 3.x)
Ensure you have Python 3 installed. Create and activate a separate virtual environment:

```bash
# Example for Linux/macOS
python3 -m venv venv-esp32
source venv-esp32/bin/activate

# Install dependencies
pip install -r requirements/requirements-esp32-py3.txt
```

#### Usage
First, ensure the ESP32 is flashed with the provided firmware (`firmware/esp32_quadrant_imu_firmware/esp32_quadrant_imu_firmware.ino`) using the Arduino IDE.

Next, identify the serial port for your ESP32. Open `src/esp32_imu/gui_esp32_capture.py` and modify the `PORT` variable at the top of the file to match your hardware (e.g., `COM3` on Windows, or `/dev/tty.usbserial-0001` on macOS).

Then run the GUI:

```bash
# Inside the Python 3 environment
python src/esp32_imu/gui_esp32_capture.py
```
The interface will open. Use the calibration buttons (no-touch and maximum-pressure) to initialize the sensor array before running your indentation routines via the MakerBot control interface.

---

## Data Output

No captured experimental data is checked into this repository. The ESP32 GUI exports the time-series sensor data and metadata as NumPy `.npz` files (excluded via `.gitignore`) directly to the local directory where the script is executed. 

## Author
Doga Ozbek
