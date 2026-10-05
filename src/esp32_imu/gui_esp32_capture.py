#!/usr/bin/env python3
"""
ESP32 Quadrant Pad + IMU GUI
- Line plots (quadrants + IMU) stacked vertically (full width)
- 2D pressure heatmap + CoP (left) + 3D pressure surface (right)
- View switch: "Line plots" vs "Heatmap + 3D"

Stream format (10 cols):
TL_est, TR_est, BL_est, BR_est, ax, ay, az, gx, gy, gz
"""

import datetime
import os
import threading
import time
import sys
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import serial
import serial.tools.list_ports
from pathlib import Path
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3D)
from serial.tools import list_ports

# Quadrant index mapping inside each sample (vals / data row)
# 0-based indices in the 10-column stream:
#   3rd index -> Q1 (TL)
#   4th index -> Q2 (BR)
#   1st index -> Q3 (BL)
#   2nd index -> Q4 (TR)
IDX_Q1 = 2  # TL
IDX_Q2 = 3  # BR
IDX_Q3 = 0  # BL
IDX_Q4 = 1  # TR

# ================== USER CONFIG ==================
# Find initial port: prefer ttyUSB devices for ESP32, fallback to any USB port, then any port
ports_info = list(list_ports.comports())
ports = [p.device for p in ports_info]
usb_ports = [p.device for p in ports_info if p.vid is not None]
esp_ports = [p.device for p in ports_info if "ttyUSB" in p.device]

if esp_ports:
    selected_port = esp_ports[0]
elif usb_ports:
    selected_port = usb_ports[0]
elif ports:
    selected_port = ports[0]
else:
    selected_port = os.environ.get("ESP32_GUI_PORT", "/dev/ttyUSB0")

PORT = selected_port
BAUD       = 921600
FS         = 700.0                      # Hz (assumed uniform)
_repo_dir   = Path(__file__).resolve().parents[2] if len(Path(__file__).resolve().parents) >= 3 else Path(__file__).resolve().parent
save_folder = _repo_dir / "data"
PLOT_UPDATE_MS = 100                    # ms

# Heatmap / CoP / 3D calibration settings
CALIB_MIN_SEC = 2.0     # no-touch (rest) duration
CALIB_MAX_SEC = 3.0     # max-press duration after that
GRID_N        = 40      # heatmap/surface resolution
# ================================================

def list_available_ports():
    """Return a sorted list of connected serial port device paths."""
    return sorted(p.device for p in serial.tools.list_ports.comports())


COL_NAMES = np.array([
    "TL_est", "TR_est", "BL_est", "BR_est",
    "ax", "ay", "az",
    "gx", "gy", "gz"
], dtype=object)


class ESP32CaptureApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("ESP32 Quadrant + IMU Capture")
        self.geometry("1400x900")

        # ----------- capture state -----------
        self.ser = None
        self.capture_thread = None
        self.stop_event = threading.Event()
        self.data = []      # list of [10] arrays
        self.n_samples = 0
        self.capture_start_time = None
        self.capture_end_time = None

        # ----------- calibration state for heatmap/CoP/3D -----------
        self.calib_start_time = None
        self.rest_sum = np.zeros(4, dtype=float)
        self.rest_count = 0
        self.press_sum = np.zeros(4, dtype=float)
        self.press_count = 0
        self.restQ = None
        self.pressQ = None
        self.calib_denom = None
        self.calib_done = False

        # 2x2 coordinates for CoP computation (Qp layout: [Q1 Q4; Q3 Q2])
        self.xc = np.array([[1.0, 2.0],
                            [1.0, 2.0]])
        self.yc = np.array([[1.0, 1.0],
                            [2.0, 2.0]])

        # Interpolation grid for 2x2 -> GRID_N x GRID_N (heatmap)
        xs = np.linspace(1.0, 2.0, GRID_N)
        ys = np.linspace(1.0, 2.0, GRID_N)
        tx = xs - 1.0   # 0..1
        ty = ys - 1.0   # 0..1
        self.TX, self.TY = np.meshgrid(tx, ty)

        # 3D surface grid (1..2 × 1..2) to match heatmap coords
        xn = np.linspace(1.0, 2.0, GRID_N)
        yn = np.linspace(1.0, 2.0, GRID_N)
        self.XX, self.YY = np.meshgrid(xn, yn)

        # View mode: "line" or "heatmap"
        self.view_mode = tk.StringVar(value="line")

        # Serial port selection
        self.port_var = tk.StringVar(value=PORT)

        # Handles
        self.surface_plot = None
        self.heatmap_cbar = None

        # ---------- UI ----------
        self._build_controls()
        self._build_plot()

        # Start periodic plot update
        self.after(PLOT_UPDATE_MS, self.update_plot)

        # Handle window close
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- UI BUILDERS ----------

    def _build_controls(self):
        frame = ttk.Frame(self)
        frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        # Left: capture buttons
        self.start_btn = ttk.Button(frame, text="Start Capture", command=self.start_capture)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(frame, text="Stop Capture", command=self.stop_capture, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.reset_btn = ttk.Button(frame, text="Reset Data", command=self.reset_data)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        # Port selection
        port_frame = ttk.LabelFrame(frame, text="Serial Port")
        port_frame.pack(side=tk.LEFT, padx=20)

        self.port_combo = ttk.Combobox(
            port_frame, textvariable=self.port_var, width=22, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=5, pady=5)

        self.refresh_ports_btn = ttk.Button(port_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_ports_btn.pack(side=tk.LEFT, padx=5, pady=5)

        self.refresh_ports()

        # Middle: view mode radio buttons
        view_frame = ttk.LabelFrame(frame, text="View")
        view_frame.pack(side=tk.LEFT, padx=20)

        rb_line = ttk.Radiobutton(
            view_frame,
            text="Line plots",
            value="line",
            variable=self.view_mode,
            command=self.update_view_mode
        )
        rb_line.pack(side=tk.LEFT, padx=5)

        rb_heatmap = ttk.Radiobutton(
            view_frame,
            text="Heatmap + 3D",
            value="heatmap",
            variable=self.view_mode,
            command=self.update_view_mode
        )
        rb_heatmap.pack(side=tk.LEFT, padx=5)

        # Right: status
        self.status_label = ttk.Label(frame, text="Status: Idle")
        self.status_label.pack(side=tk.LEFT, padx=20)

    def _build_plot(self):
        self.fig = Figure(figsize=(11, 7), dpi=100)
        gs = self.fig.add_gridspec(2, 2, hspace=0.35, wspace=0.25)

        # ---------- Line view axes (2 rows, full width) ----------
        self.ax_line1 = self.fig.add_subplot(gs[0, :])
        self.ax_line2 = self.fig.add_subplot(gs[1, :])

        self.ax_line1.set_title("Quadrant estimates (TL, TR, BL, BR)")
        self.ax_line1.set_xlabel("")  # avoid collision
        self.ax_line1.set_ylabel("ADC (est.)")
        self.ax_line1.grid(True)
        self.lines_quadrants = self.ax_line1.plot([], [], [], [], [], [], [], [])
        self.ax_line1.legend(["TL", "TR", "BL", "BR"], loc="upper right")

        self.ax_line2.set_title("IMU (ax, ay, az / gx, gy, gz)")
        self.ax_line2.set_xlabel("Time (s)")
        self.ax_line2.set_ylabel("SI units")
        self.ax_line2.grid(True)
        self.lines_imu = self.ax_line2.plot([], [], [], [], [], [], [], [], [], [], [], [])
        self.ax_line2.legend(["ax", "ay", "az", "gx", "gy", "gz"], loc="upper right")

        # ---------- Heatmap axis (left, both rows) ----------
        self.ax_heatmap = self.fig.add_subplot(gs[:, 0])
        self.ax_heatmap.set_title("Pressure Heatmap (calibration pending)")
        self.ax_heatmap.set_xlabel("X")
        self.ax_heatmap.set_ylabel("Y")
        self.ax_heatmap.set_xlim(1.0, 2.0)
        self.ax_heatmap.set_ylim(1.0, 2.0)
        self.ax_heatmap.set_aspect("equal", adjustable="box")
        self.ax_heatmap.grid(False)

        Z0 = np.zeros((GRID_N, GRID_N))
        self.heatmap_img = self.ax_heatmap.imshow(
            Z0,
            extent=(1.0, 2.0, 1.0, 2.0),
            origin="lower",
            vmin=0.0,
            vmax=1.0
        )
        self.cop_point, = self.ax_heatmap.plot([], [], 'wo', markerfacecolor='k', markersize=6, label="CoP")
        self.ax_heatmap.legend(loc="upper right")

        # ---------- 3D surface axis (right, both rows) ----------
        self.ax_3d = self.fig.add_subplot(gs[:, 1], projection='3d')
        self.ax_3d.set_title("3D Pressure Surface")
        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Pressure")
        self.ax_3d.set_zlim(0.0, 1.0)
        self.ax_3d.view_init(elev=35, azim=-35)
        self.ax_3d.grid(True)

        Z3D0 = np.zeros_like(self.XX)
        self.surface_plot = self.ax_3d.plot_surface(
            self.XX, self.YY, Z3D0,
            rstride=1, cstride=1,
            linewidth=0, antialiased=True,
            cmap="hot"
        )
        self.surface_plot.set_clim(0.0, 1.0)

        # ---------- Shared colorbar for both heatmap & 3D ----------
        self.heatmap_cbar = self.fig.colorbar(
            self.heatmap_img,
            ax=[self.ax_heatmap, self.ax_3d],
            label="Normalized Pressure",
            fraction=0.046,
            pad=0.04
        )

        # ---------- Embed canvas ----------
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Start in line mode
        self.update_view_mode()

    # ---------- VIEW MODE TOGGLE ----------

    def update_view_mode(self):
        mode = self.view_mode.get()
        if mode == "line":
            # Show line plots, hide heatmap + 3D + colorbar
            self.ax_line1.set_visible(True)
            self.ax_line2.set_visible(True)

            self.ax_heatmap.set_visible(False)
            self.ax_3d.set_visible(False)
            if self.heatmap_cbar is not None:
                self.heatmap_cbar.ax.set_visible(False)
        else:
            # Show heatmap + 3D, hide line plots
            self.ax_line1.set_visible(False)
            self.ax_line2.set_visible(False)

            self.ax_heatmap.set_visible(True)
            self.ax_3d.set_visible(True)
            if self.heatmap_cbar is not None:
                self.heatmap_cbar.ax.set_visible(True)

        self.canvas.draw_idle()

    # ---------- PORT SELECTION ----------

    def refresh_ports(self):
        """Re-scan available serial ports and refresh the dropdown."""
        ports = list_available_ports()
        current = self.port_var.get()

        self.port_combo["values"] = ports

        if current in ports:
            self.port_var.set(current)
        elif ports:
            esp_ports = [p for p in ports if "ttyUSB" in p]
            if esp_ports:
                self.port_var.set(esp_ports[0])
            else:
                self.port_var.set(ports[0])
        # else: keep whatever was there (e.g. the hardcoded default) so the
        # user can still see/edit what was configured even if nothing is detected

    # ---------- CAPTURE LOGIC ----------

    def start_capture(self):
        """Start or resume capturing data."""
        if self.capture_thread and self.capture_thread.is_alive():
            messagebox.showinfo("Info", "Capture is already running.")
            return

        port = self.port_var.get()
        if not port:
            messagebox.showerror("Serial Error", "No serial port selected.")
            return

        self.stop_event.clear()
        self.capture_start_time = datetime.datetime.now()
        self.capture_end_time = None

        # Open serial
        try:
            self.ser = serial.Serial(port, baudrate=BAUD, timeout=1.0)
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Could not open {port} @ {BAUD} baud:\n{e}")
            self.ser = None
            return

        self.status_label.config(text=f"Status: Capturing on {port} @ {BAUD}")
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.port_combo.config(state=tk.DISABLED)
        self.refresh_ports_btn.config(state=tk.DISABLED)

        # Start background thread
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

    def stop_capture(self):
        """Request capture to stop and schedule clean-up."""
        if not self.capture_thread:
            return

        self.stop_event.set()
        self.capture_end_time = datetime.datetime.now()
        self.status_label.config(text="Status: Stopping...")
        self.after(100, self._finish_stop)

    def _finish_stop(self):
        """Finish stopping: close serial, save data, re-enable controls."""
        if self.capture_thread and self.capture_thread.is_alive():
            self.after(100, self._finish_stop)
            return

        if self.capture_end_time is None:
            self.capture_end_time = datetime.datetime.now()

        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.port_combo.config(state="readonly")
        self.refresh_ports_btn.config(state=tk.NORMAL)

        self.save_data()
        self.status_label.config(text="Status: Idle")

    def _capture_loop(self):
        """Background thread: read lines from serial and append to self.data."""
        print(f"[THREAD] Capture started on {self.ser.port} @ {BAUD}")
        while not self.stop_event.is_set():
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode("ascii", errors="ignore").strip()
                if not line or line.startswith("#"):
                    continue

                parts = line.split("\t")
                if len(parts) != 10:
                    continue

                try:
                    vals = [float(p) for p in parts]
                except ValueError:
                    continue

                self.data.append(vals)
                self.n_samples += 1

                # ---- Calibration handling (uses quadrants) ----
                if self.calib_start_time is None:
                    self.calib_start_time = time.time()

                if not self.calib_done:
                    tNow = time.time() - self.calib_start_time

                    Q1 = vals[IDX_Q1]  # TL
                    Q2 = vals[IDX_Q2]  # BR
                    Q3 = vals[IDX_Q3]  # BL
                    Q4 = vals[IDX_Q4]  # TR
                    curQ = np.array([Q1, Q2, Q3, Q4], dtype=float)

                    if tNow < CALIB_MIN_SEC:
                        self.rest_sum += curQ
                        self.rest_count += 1
                    elif tNow < CALIB_MIN_SEC + CALIB_MAX_SEC:
                        self.press_sum += curQ
                        self.press_count += 1
                    else:
                        if self.rest_count > 0 and self.press_count > 0:
                            self.restQ = self.rest_sum / float(self.rest_count)
                            self.pressQ = self.press_sum / float(self.press_count)
                            denom = self.restQ - self.pressQ
                            denom[denom <= 1e-6] = 1.0
                            self.calib_denom = denom
                            self.calib_done = True
                            print("[CALIB] Done.")
                            print("[CALIB] restQ:", self.restQ)
                            print("[CALIB] pressQ:", self.pressQ)

            except serial.SerialException as e:
                print(f"[THREAD] Serial error: {e}", file=sys.stderr)
                break
            except Exception as e:
                print(f"[THREAD] Error: {e}", file=sys.stderr)
                continue

        print("[THREAD] Capture loop exiting.")

    # ---------- RESET & SAVE ----------

    def reset_data(self):
        """Clear captured data and reset plots + calibration."""
        if self.capture_thread and self.capture_thread.is_alive():
            messagebox.showinfo("Info", "Stop capture before resetting data.")
            return

        self.data = []
        self.n_samples = 0
        self.capture_start_time = None
        self.capture_end_time = None

        # Calibration reset
        self.calib_start_time = None
        self.rest_sum[:] = 0.0
        self.rest_count = 0
        self.press_sum[:] = 0.0
        self.press_count = 0
        self.restQ = None
        self.pressQ = None
        self.calib_denom = None
        self.calib_done = False

        # Clear line plots
        for line in self.lines_quadrants:
            line.set_data([], [])
        for line in self.lines_imu:
            line.set_data([], [])

        self.ax_line1.set_xlim(0, 1)
        self.ax_line2.set_xlim(0, 1)
        self.ax_line1.relim()
        self.ax_line1.autoscale_view()
        self.ax_line2.relim()
        self.ax_line2.autoscale_view()

        # Clear heatmap + CoP
        self.heatmap_img.set_data(np.zeros((GRID_N, GRID_N)))
        self.cop_point.set_data([], [])
        self.ax_heatmap.set_title("Pressure Heatmap (calibration pending)")

        # Clear 3D surface
        if self.surface_plot is not None:
            try:
                self.surface_plot.remove()
            except Exception:
                pass
            self.surface_plot = None

        Z3D0 = np.zeros_like(self.XX)
        self.surface_plot = self.ax_3d.plot_surface(
            self.XX, self.YY, Z3D0,
            rstride=1, cstride=1,
            linewidth=0, antialiased=True,
            cmap="hot"
        )
        self.surface_plot.set_clim(0.0, 1.0)
        self.ax_3d.set_zlim(0.0, 1.0)
        self.ax_3d.set_title("3D Pressure Surface")

        self.canvas.draw_idle()
        self.status_label.config(text="Status: Data reset")

    def save_data(self):
        """Save captured data to npz file."""
        if self.n_samples == 0:
            messagebox.showinfo("Info", "No data captured, nothing to save.")
            return

        try:
            d = np.array(self.data, dtype=float)
            t = np.arange(d.shape[0]) / FS

            start_dt = self.capture_start_time or datetime.datetime.now()
            end_dt = self.capture_end_time or datetime.datetime.now()

            date_str = start_dt.strftime("%Y-%m-%d")
            start_str = start_dt.strftime("%H-%M-%S")
            end_str = end_dt.strftime("%H-%M-%S")

            save_folder.mkdir(parents=True, exist_ok=True)
            save_file = save_folder / f"capture_{date_str}_start_{start_str}_end_{end_str}.npz"

            np.savez(
                save_file,
                t=t,
                data=d,
                colNames=COL_NAMES,
                startTime=start_dt.isoformat(),
                endTime=end_dt.isoformat(),
            )
            messagebox.showinfo("Saved", f"Captured {d.shape[0]} samples.\nSaved to {save_file.name}")
            print(f"Saved {d.shape[0]} samples to {save_file}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Could not save data:\n{e}")

    # ---------- PLOTTING ----------

    def update_plot(self):
        """Periodic plot update from self.data."""
        if self.n_samples > 2:
            try:
                d = np.array(self.data, dtype=float)
                t = np.arange(d.shape[0]) / FS

                # ---- Line plots (updated even if hidden) ----
                for i in range(4):  # quadrants
                    self.lines_quadrants[i].set_data(t, d[:, i])

                for i in range(3):  # accel
                    self.lines_imu[i].set_data(t, d[:, 4 + i])
                for i in range(3):  # gyro
                    self.lines_imu[3 + i].set_data(t, d[:, 7 + i])

                if d.shape[0] > 5:
                    t_max = t[-1]
                    self.ax_line1.set_xlim(0, max(1.0, t_max))
                    self.ax_line2.set_xlim(0, max(1.0, t_max))

                self.ax_line1.relim()
                self.ax_line1.autoscale_view()
                self.ax_line2.relim()
                self.ax_line2.autoscale_view()

                # ---- Heatmap + 3D ----
                if not self.calib_done and self.calib_start_time is not None:
                    tNow = time.time() - self.calib_start_time
                    if tNow < CALIB_MIN_SEC:
                        self.ax_heatmap.set_title(f"Calibration step 1: NO TOUCH (t = {tNow:.1f} s)")
                    elif tNow < CALIB_MIN_SEC + CALIB_MAX_SEC:
                        self.ax_heatmap.set_title(f"Calibration step 2: MAX PRESS (t = {tNow:.1f} s)")
                    else:
                        self.ax_heatmap.set_title("Waiting to finalize calibration...")
                elif self.calib_done:
                    self.ax_heatmap.set_title("Pressure map (0..1, calibrated)")

                    last = d[-1, :]  # last sample

                    Q1 = last[IDX_Q1]
                    Q2 = last[IDX_Q2]
                    Q3 = last[IDX_Q3]
                    Q4 = last[IDX_Q4]

                    curQ = np.array([Q1, Q2, Q3, Q4], dtype=float)
                    P = (self.restQ - curQ) / self.calib_denom
                    P = np.clip(P, 0.0, 1.0)

                    # Layout: [Q1 Q4; Q3 Q2]
                    Qp = np.array([[P[0], P[3]],
                                   [P[2], P[1]]], dtype=float)

                    # Bilinear interpolation 2x2 -> GRID_N×GRID_N
                    Q11 = Qp[0, 0]
                    Q21 = Qp[0, 1]
                    Q12 = Qp[1, 0]
                    Q22 = Qp[1, 1]

                    TX = self.TX
                    TY = self.TY
                    Zq = (Q11 * (1 - TX) * (1 - TY) +
                          Q21 * TX       * (1 - TY) +
                          Q12 * (1 - TX) * TY       +
                          Q22 * TX       * TY)
                    Zq = np.clip(Zq, 0.0, 1.0)

                    # 2D heatmap update
                    self.heatmap_img.set_data(Zq)

                    # Center of Pressure (CoP)
                    totalP = Qp.sum()
                    if totalP > 1e-6:
                        copX = np.sum(Qp * self.xc) / totalP
                        copY = np.sum(Qp * self.yc) / totalP
                        self.cop_point.set_data([copX], [copY])
                    else:
                        self.cop_point.set_data([], [])

                    # 3D surface update: remove old surface and draw new one
                    if self.surface_plot is not None:
                        try:
                            self.surface_plot.remove()
                        except Exception:
                            pass
                        self.surface_plot = None

                    Zplot = Zq
                    self.surface_plot = self.ax_3d.plot_surface(
                        self.XX, self.YY, Zplot,
                        rstride=1, cstride=1,
                        linewidth=0, antialiased=True,
                        cmap="hot"
                    )
                    self.surface_plot.set_clim(0.0, 1.0)
                    self.ax_3d.set_zlim(0.0, 1.0)

                self.canvas.draw_idle()

            except Exception as e:
                print(f"[PLOT] Error updating plot: {e}", file=sys.stderr)

        self.after(PLOT_UPDATE_MS, self.update_plot)

    # ---------- CLEANUP ----------

    def on_close(self):
        """Handle window close."""
        if self.capture_thread and self.capture_thread.is_alive():
            if not messagebox.askyesno("Quit", "Capture is running. Stop and quit?"):
                return
            self.stop_event.set()
            self.after(300, self.destroy)
        else:
            self.destroy()


if __name__ == "__main__":
    app = ESP32CaptureApp()
    app.mainloop()
