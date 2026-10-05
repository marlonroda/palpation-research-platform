# Palpation Research Platform

> **A unified software and firmware suite for robotic palpation research.**

This repository provides a complete environment for experimental data capture and automated indentation testing. It leverages a **MakerBot Replicator 2X** as a programmable 3-axis positioning stage to perform mechanical indentations, while simultaneously capturing live pressure and kinematic data using an **ESP32-based quadrant sensor and IMU**.

---

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Repository Structure](#repository-structure)
- [System Architecture & Installation](#system-architecture--installation)
  - [Prerequisites & Environment Setup](#prerequisites--environment-setup)
  - [1. MakerBot Motion Control GUI](#1-makerbot-motion-control-gui)
  - [2. ESP32 Sensor Data Capture GUI](#2-esp32-sensor-data-capture-gui)
- [Configuration & Presets](#configuration--presets)
  - [Hardware & Motion Settings (`config.yaml`)](#hardware--motion-settings-configyaml)
  - [Indentation Experiment Presets (`experiments/`)](#indentation-experiment-presets-experiments)
- [ESP32 Firmware](#esp32-firmware)
- [Data Output Policy](#data-output-policy)

---

## Hardware Requirements

To utilize this platform, the following hardware is required:
- **MakerBot Replicator 2X** (running Sailfish firmware)
- **ESP32 Microcontroller**
- **Quadrant Pressure Sensor Array** (interfaced via MCP3008 ADC)
- **Inertial Measurement Unit (IMU)** (MPU6050 or BNO08x)

> [!NOTE]
> Supporting CAD models and electronics board designs are documented in [`docs/hardware-designs.md`](docs/hardware-designs.md).

---

## Repository Structure

The codebase is unified into a standard software project structure:

- **`config.yaml`**: Hardware constants (steps/mm, baud rates, safe travel limits, dwell times).
- **`experiments/`**: Preconfigured indentation experiment routines in YAML format.
- **`requirements.txt`**: Unified Python 3 dependency specification.
- **`requirements/`**: Modular dependency files for individual subsystems:
  - `requirements-makerbot-py3.txt`: Dependencies for the MakerBot motion control server.
  - `requirements-esp32-py3.txt`: Dependencies for the sensor visualization GUI.
- **`src/`**: High-level desktop and web applications:
  - `makerbot_control/`: Flask-based web interface to control the MakerBot positioning platform.
    - `control_gui.py`: Main Flask application and motion runner.
    - `s3g_client.py`: Native Python 3 implementation of the S3G protocol.
    - `config.py`: Configuration loader for `config.yaml`.
    - `experiment.py`: Parser and serializer for experiment parameter files.
  - `esp32_imu/`: Python Tkinter desktop application for visualizing pressure and IMU data:
    - `gui_esp32_capture.py`: Live 2D heatmap, center-of-pressure, 3D surface, and time-series plotting.
- **`firmware/`**: C++ code (`.ino`) for the ESP32 microcontroller, streaming synchronized analog pressure and IMU values over serial.
- **`docs/`**: Hardware design references and documentation.

---

## System Architecture & Installation

The entire platform runs on modern **Python 3 (3.8+)**. The motion control subsystem uses a native, dependency-free Python 3 S3G protocol client (`s3g_client.py`), removing legacy Python 2 drivers and allowing both subsystems to operate in a single Python 3 environment.

### Prerequisites & Environment Setup

Create and activate a virtual environment, then install dependencies:

```bash
# Create and activate environment (Linux/macOS)
python3 -m venv venv
source venv/bin/activate

# Install all platform dependencies
pip install -r requirements.txt
```

Alternatively, if running the subsystems on separate machines or environments, individual requirement files in `requirements/` can be installed.

---

### 1. MakerBot Motion Control GUI

This subsystem acts as the programmable motion stage, exposing a local web interface to command the Replicator 2X.

#### Key Capabilities
- **Serial Auto-Detection**: Scans connected USB/serial devices with a dropdown selector.
- **Homed-State Persistence**: Queries and caches the machine's step positions (`home_state.json`), eliminating redundant auto-homing cycles across application restarts.
- **Motion Modes**: Single movements, continuous swing motion, and automated waypoint indentation routines.
- **Experiment Runner**: Supports multi-cycle indentations, configurable dwell times, and repeated run sequences with inter-run pauses.
- **Emergency Stop**: Dedicated hardware-level stop that immediately halts stepper motors and clears the queued command buffer.

#### Usage
> [!WARNING]
> This software sends live motion commands to physical hardware. Ensure the printer build area is clear of obstacles and verify travel limits before initiating movements.

```bash
# Launch the motion control web server
python src/makerbot_control/control_gui.py
```

Open a web browser and navigate to `http://127.0.0.1:5000`. Select your serial port, connect, perform homing if necessary, and execute manual movements or automated experiment routines.

---

### 2. ESP32 Sensor Data Capture GUI

This subsystem connects to the ESP32, streams 10-column sensor data (4x quadrant pressure, 3x accelerometer, 3x gyroscope), and provides real-time visualization.

#### Key Capabilities
- **Serial Port Selection**: Automatic detection of connected serial ports with in-app refresh.
- **Live Visualizations**:
  - 4-channel quadrant force and 6-axis IMU line plots.
  - 2D interpolated pressure heatmap and real-time Center-of-Pressure (CoP) marker.
  - Interactive 3D pressure surface.
- **Two-Stage Calibration**: In-app no-touch baseline subtraction and maximum-pressure normalization.
- **Timestamped Dataset Export**: Automatically names and saves experimental runs with ISO-8601 start/end timestamps and sensor channel names into `data/`.

#### Usage

```bash
# Launch the sensor capture GUI
python src/esp32_imu/gui_esp32_capture.py
```

1. Select the ESP32 serial port from the dropdown menu (or click "Refresh").
2. Click **Start Capture** to begin receiving the serial data stream.
3. Switch between **Line Plots** and **Heatmap / CoP / 3D** views as needed.
4. Perform baseline calibration prior to recording indentation trials.
5. Click **Stop Capture** to end the run. The session is automatically saved to `data/` as a NumPy `.npz` archive.

---

## Configuration & Presets

### Hardware & Motion Settings (`config.yaml`)

Hardware parameters are centralized in `config.yaml` at the root of the repository:

- `serial`: Communication baud rate (default: `115200`).
- `steps_per_mm`: Kinematic resolution for XY (`88.8889`) and Z (`400.0`) axes.
- `motion`: Default feed rate and post-move dwell time.
- `indent`: Dwell times at indentation depth and intermediate waypoints.
- `soft_limits`: Maximum travel limits in millimeters (`max_x_mm: 220.0`, `max_y_mm: 140.0`, `max_z_mm: 150.0`) to avoid mechanical endstop collisions.
- `logging`: File destination for motion command logs.

### Indentation Experiment Presets (`experiments/`)

Repetitive indentation experiments can be saved and loaded directly through the web UI using YAML specification files:

- `experiments/exp1.yaml`: Multi-point linear palpation routine across waypoints.
- `experiments/exp2.yaml`: Fine-depth indentation test condition.
- `experiments/exp3.yaml`: Continuous bounce indentation profile with multi-run repeats.

Users can also export customized routines created in the web interface by entering a target file path in the "Save to File" field.

---

## ESP32 Firmware

The microcontroller firmware is located at:
`firmware/esp32_quadrant_imu_firmware/esp32_quadrant_imu_firmware.ino`

### Uploading Firmware
1. Open the `.ino` sketch in the Arduino IDE.
2. Install the necessary libraries:
   - `Adafruit_MCP3008`
   - `SimpleKalmanFilter`
   - `Adafruit_MPU6050` and `Adafruit_Sensor` (or `Adafruit_BNO08x`)
3. Select your ESP32 board target and COM/serial port.
4. Upload the sketch to the microcontroller.

The firmware outputs a tab-separated 10-value stream:
```
TL_est  TR_est  BL_est  BR_est  ax  ay  az  gx  gy  gz
```
at ~700 Hz force sampling and ~200 Hz IMU cadence.

---

## Data Output Policy

No experimental datasets are committed to this repository.

Captured sensor runs are exported as `.npz` files into the `data/` directory:
- Format: `capture_<YYYY-MM-DD>_start_<HH-MM-SS>_end_<HH-MM-SS>.npz`
- Fields: `t` (relative time vector), `data` (sample array), `colNames` (channel headers), `startTime` (ISO timestamp), `endTime` (ISO timestamp).

All dataset files (`*.npz`, `*.npy`, `*.csv`, `*.mat`), logs, and runtime state snapshots are ignored by version control.
