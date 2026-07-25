# ESP32 Quadrant Sensor and IMU GUI

This repository contains a Python desktop application for live data capture and visualization from an ESP32-based quadrant pressure sensor and IMU system.

The application reads a 10-column serial stream containing four quadrant sensor estimates, three-axis accelerometer data, and three-axis gyroscope data. It provides live line plots, a calibrated two-dimensional pressure heatmap, center-of-pressure visualization, and a three-dimensional pressure surface.

## Features

- Serial communication with an ESP32
- Start, stop, resume, and reset controls
- Live visualization of four quadrant sensor channels
- Live accelerometer and gyroscope plots
- Two-stage pressure calibration
- Two-dimensional interpolated pressure heatmap
- Center-of-pressure estimation
- Three-dimensional pressure surface visualization
- NumPy `.npz` data export
- Separate line-plot and heatmap/3D visualization modes

## Expected Serial Format

The ESP32 should stream one tab-separated line containing 10 numeric values:

`TL_est, TR_est, BL_est, BR_est, ax, ay, az, gx, gy, gz`

## Repository Structure

- `src/gui_esp32_capture.py`: Main Tkinter capture and visualization application
- `requirements.txt`: Python dependencies
- `docs/`: Reserved for future documentation
- `.gitignore`: Excludes captured data, generated outputs, and local environments

## Requirements

- Python 3
- NumPy
- pySerial
- Matplotlib
- Tkinter with Tk support

Install the Python dependencies with:

```bash
pip install -r requirements.txt
```

## Configuration

The main acquisition settings are defined near the top of the script:

```python
PORT = "/dev/tty.usbserial-0001"
BAUD = 921600
FS = 700.0
SAVE_FILE = "esp32_capture.npz"
PLOT_UPDATE_MS = 100
```

Update `PORT` to match the connected ESP32 serial device.

## Calibration

The pressure visualization uses a two-stage calibration:

1. No-touch calibration
2. Maximum-pressure calibration

After calibration, the four quadrant signals are normalized and used to calculate the pressure map, center of pressure, and three-dimensional pressure surface.

## Running the Application

Activate a Python environment containing the required packages, then run:

```bash
python src/gui_esp32_capture.py
```

Example using the existing virtual environment:

```bash
source ~/Desktop/esp32_capture_app/venv/bin/activate
python ~/Desktop/"for github"/esp32-quadrant-imu-gui/src/gui_esp32_capture.py
```

## Data Policy

No captured experimental data is included in this repository.

Generated `.npz`, `.npy`, `.csv`, `.mat`, `.xlsx`, and `.log` files are excluded through `.gitignore`.

## Notes

This is an experimental research utility. Verify the serial port, baud rate, sensor mapping, and sampling behavior before use.

The saved time vector is generated from the assumed sampling frequency rather than measured host timestamps.

## Possible Improvements

- Add automatic serial-port detection
- Move hardware settings into a configuration file
- Add command-line arguments
- Record actual host-side timestamps
- Add automatic sampling-rate estimation
- Add CSV export
- Add calibration progress indicators
- Add graceful handling of disconnected devices
- Add example ESP32 firmware

## Author

Doga Ozbek

## Hardware and Electronics Designs

Supporting CAD models and electronics-board designs are available through Autodesk Fusion:

- [Hardware Design 1](https://a360.co/4vqEozp)
- [Hardware Design 2](https://a360.co/4b3lKFo)
- [Hardware Design 3](https://a360.co/4enpDHA)
- [Hardware Design 4](https://a360.co/44qpbSW)

Further information is available in [`docs/hardware-designs.md`](docs/hardware-designs.md).

## ESP32 Firmware

The ESP32 firmware used by this project is included in:

`firmware/esp32_quadrant_imu_firmware/esp32_quadrant_imu_firmware.ino`

Open the sketch in the Arduino IDE, select the appropriate ESP32 board and serial port, install the required Arduino libraries, and upload it to the board before starting the Python GUI.

The firmware streams the quadrant sensor and IMU values expected by the desktop application.
