#!/usr/bin/env python3
"""
Web GUI control for MakerBot Replicator 2X (Sailfish) from Python 3.

Features:
- Auto-detect serial ports and let the user select one.
- Connect / disconnect buttons.
- Optional auto-homing (calibration) via a button.
- Speed (FEED_DDA) control.
- Swing / Single mode selection.
- Bounce count (how many go-return cycles in swing mode).
- X/Y/Z target fields + "Move" button.
- Experiment Runner:
    * Bounce between a start point and a single waypoint, indenting at both.
    * Repeat bounce cycles, and wait predefined minutes before starting new runs.

Motion/hardware constants are loaded from config.yaml at the repo root (see
config.py) instead of being hardcoded here.

Open http://127.0.0.1:5000 in your browser after starting this script.
"""

import atexit
import json
import os
import signal
import sys
import time
import threading

from serial.tools import list_ports
from flask import Flask, request, redirect, url_for, render_template_string, jsonify

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import config as app_config
from experiment import load_experiment, save_experiment
from s3g_client import S3GClient, TransmissionError, open_connection

CFG = app_config.load_config()

BAUD = CFG.baud
XY_STEPS_PER_MM = CFG.xy_steps_per_mm
Z_STEPS_PER_MM = CFG.z_steps_per_mm
DEFAULT_FEED_DDA = CFG.default_feed_dda
DWELL_SEC = CFG.dwell_sec
INDENT_DWELL_SEC = CFG.indent_dwell_sec
WAYPOINT_DWELL_SEC = CFG.waypoint_dwell_sec
HOME_WAIT_X = CFG.home_wait_x
HOME_WAIT_Y = CFG.home_wait_y
HOME_WAIT_Z = CFG.home_wait_z
MAX_X_MM = CFG.max_x_mm
MAX_Y_MM = CFG.max_y_mm
MAX_Z_MM = CFG.max_z_mm


# ---------------------------------------------------------------------
# HELPER FUNCTIONS (same as CLI logic)
# ---------------------------------------------------------------------
def clamp_mm(x, y, z):
    """Clamp requested mm positions to a safe box."""
    if x < 0:
        x = 0.0
    if y < 0:
        y = 0.0
    if z < 0:
        z = 0.0

    if x > MAX_X_MM:
        x = MAX_X_MM
    if y > MAX_Y_MM:
        y = MAX_Y_MM
    if z > MAX_Z_MM:
        z = MAX_Z_MM
    return x, y, z


def mm_to_steps(x_mm, y_mm, z_mm):
    """
    Convert user mm to step coordinates.

    After homing:
    - X and Y are homed to their MAX endstops (back/right).
      Firmware coordinates increase *towards* the switches.
      We want user-positive X/Y to move INTO the bed, i.e. NEGATIVE in firmware.
      => Flip sign for X and Y.
    - Z is homed to MIN (bottom), so positive Z goes up as usual.
    """
    x_steps = int(round(-x_mm * XY_STEPS_PER_MM))  # NOTE: minus sign
    y_steps = int(round(-y_mm * XY_STEPS_PER_MM))  # NOTE: minus sign
    z_steps = int(round(z_mm * Z_STEPS_PER_MM))
    return [x_steps, y_steps, z_steps, 0, 0]


def home_axis_max(r, axis, wait_s, log_func, lock):
    """Home given axis towards maximum, then sleep."""
    log_func("Homing %s towards MAX..." % axis.upper())
    try:
        with lock:
            r.find_axes_maximums([axis], 500, 60)
    except TransmissionError as e:
        log_func("[Warning] TransmissionError during %s homing: %r" % (axis, e))
        log_func("If the carriage hit the %s endstop, this is probably OK." % axis.upper())
    time.sleep(wait_s)
    log_func("%s homing wait done." % axis.upper())


def home_axis_min(r, axis, wait_s, log_func, lock):
    """Home given axis towards minimum, then sleep."""
    log_func("Homing %s towards MIN..." % axis.upper())
    try:
        with lock:
            r.find_axes_minimums([axis], 500, 60)
    except TransmissionError as e:
        log_func("[Warning] TransmissionError during %s homing: %r" % (axis, e))
        log_func("If the axis hit the %s endstop, this is probably OK." % axis.upper())
    time.sleep(wait_s)
    log_func("%s homing wait done." % axis.upper())


class _MotionStopped(Exception):
    """Internal control-flow signal used to unwind out of the nested
    waypoint_indent loops as soon as a STOP is requested."""


# ---------------------------------------------------------------------
# CONTROLLER CLASS
# ---------------------------------------------------------------------
class Rep2XController(object):
    def __init__(self):
        self.ser = None
        self.r = None
        self.connected = False
        self.homed = False

        self.feed_dda = DEFAULT_FEED_DDA
        self.bounce_count = 1
        self.swing_mode = True
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0

        self.experiment = {}
        self.experiment_path = ""

        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.progress = {"active": False, "label": "", "current": 0, "total": 0}
        self.progress_lock = threading.Lock()

        self.log_lines = []
        self.lock = threading.Lock()

        log_path = CFG.log_file
        log_dir = os.path.dirname(log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(log_path, "a", buffering=1)

        self.home_state_file = os.path.join(log_dir or ".", "home_state.json")

    def log(self, msg):
        with self.lock:
            line = time.strftime("[%H:%M:%S] ") + msg
            print(line)
            self.log_lines.append(line)
            if len(self.log_lines) > 200:
                self.log_lines = self.log_lines[-200:]
            self.log_file.write(time.strftime("%Y-%m-%d %H:%M:%S ") + msg + "\n")

    def get_log_text(self):
        with self.lock:
            return "\n".join(self.log_lines)

    def _set_progress(self, active, label="", current=0, total=0):
        with self.progress_lock:
            self.progress = {"active": active, "label": label, "current": current, "total": total}

    def get_progress(self):
        with self.progress_lock:
            p = dict(self.progress)
        p["percent"] = round(100 * p["current"] / p["total"]) if p["total"] else 0
        return p

    def _interruptible_sleep(self, seconds):
        deadline = time.time() + seconds
        while True:
            remaining = deadline - time.time()
            if remaining <= 0 or self.stop_event.is_set():
                return
            time.sleep(min(0.05, remaining))

    def emergency_stop(self):
        self.stop_event.set()

        if not self.connected or self.r is None:
            self.log("STOP pressed, but not connected - nothing to halt on the machine.")
            return

        try:
            with self.serial_lock:
                self.r.extended_stop(halt_steppers=True, clear_queue=True)
            self.log("=== STOP: steppers halted, command queue cleared ===")
        except Exception as e:
            self.log("[Error] STOP: failed to send extended_stop: %r" % (e,))

        self.homed = False
        self.log("[Warning] Position tracking may no longer match the physical carriage - re-home before continuing.")

    def connect(self, port):
        if self.connected:
            self.log("Already connected.")
            return

        try:
            self.ser = open_connection(port, BAUD, timeout=1)
        except Exception as e:
            self.log("Failed to open port %s: %r" % (port, e))
            raise

        self.r = S3GClient(self.ser)

        self.connected = True
        self.curr_x = self.curr_y = self.curr_z = 0.0
        self.log("Connected to Replicator 2X on %s @ %d baud." % (port, BAUD))
        self.homed = self._check_already_homed()

    def _save_home_state(self):
        try:
            with self.serial_lock:
                pos = self.r.get_extended_position()
        except Exception as e:
            self.log("[Warning] Could not read position to save home state: %r" % (e,))
            return
        try:
            with open(self.home_state_file, "w") as f:
                json.dump({"steps": list(pos[:5])}, f)
        except OSError as e:
            self.log("[Warning] Could not write home state file: %r" % (e,))

    def _check_already_homed(self):
        try:
            with open(self.home_state_file) as f:
                saved_steps = tuple(json.load(f)["steps"])
        except (OSError, ValueError, KeyError):
            return False

        try:
            with self.serial_lock:
                pos = self.r.get_extended_position()
        except Exception as e:
            self.log("[Info] Could not query firmware position: %r" % (e,))
            return False

        if tuple(pos[:5]) == saved_steps:
            self.log("Firmware position matches last known homed state - skipping re-home.")
            return True

        self.log("Firmware position %s doesn't match last homed state %s - please Auto-home."
                 % (pos[:5], saved_steps))
        return False

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.r = None
        self.connected = False
        self.homed = False
        self.log("Disconnected.")

    def auto_home(self):
        if not self.connected or self.r is None:
            self.log("Cannot home – not connected.")
            return

        def do_home():
            self.log("=== AUTO-HOMING SEQUENCE START ===")
            self.log("Make sure the bed is clear.\n")

            home_axis_max(self.r, 'x', HOME_WAIT_X, self.log, self.serial_lock)
            home_axis_max(self.r, 'y', HOME_WAIT_Y, self.log, self.serial_lock)
            home_axis_min(self.r, 'z', HOME_WAIT_Z, self.log, self.serial_lock)

            self.log("Recalling home positions from firmware...")
            try:
                with self.serial_lock:
                    self.r.recall_home_positions(['x', 'y', 'z', 'a', 'b'])
                self.log("recall_home_positions sent.")
            except TransmissionError as e:
                self.log("[Warning] TransmissionError during recall_home_positions: %r" % (e,))
                self.log("(On some Sailfish builds this still works despite the warning.)")

            time.sleep(2.0)

            self.homed = True
            self.curr_x = self.curr_y = self.curr_z = 0.0
            self._save_home_state()
            self.log("=== AUTO-HOMING COMPLETE ===")
            self.log("X/Y should be at back/right, Z at home.\n")

        t = threading.Thread(target=do_home)
        t.daemon = True
        t.start()

    def set_speed(self, v):
        self.feed_dda = v
        self.log("Updated FEED_DDA speed to %d" % self.feed_dda)

    def set_mode(self, mode):
        self.swing_mode = (mode == "swing")
        self.log("Mode set to %s." % ("SWING (go & return)" if self.swing_mode else "SINGLE (go & stay)"))

    def set_bounce(self, n):
        self.bounce_count = n
        self.log("Updated bounce_count to %d" % self.bounce_count)

    def _send_point(self, steps, label):
        try:
            with self.serial_lock:
                self.r.queue_extended_point(steps, self.feed_dda)
        except TransmissionError as e:
            self.log("    [Warning] TransmissionError during %s: %r" % (label, e))
        except Exception as e:
            self.log("    [Error] Other exception during %s: %r" % (label, e))

    def move(self, x_mm, y_mm, z_mm):
        if not self.connected or self.r is None:
            self.log("Cannot move – not connected.")
            return

        self.stop_event.clear()

        # Clamp to safe region
        x_mm, y_mm, z_mm = clamp_mm(x_mm, y_mm, z_mm)

        start_x, start_y, start_z = self.curr_x, self.curr_y, self.curr_z
        target_x, target_y, target_z = x_mm, y_mm, z_mm

        if not self.swing_mode:
            # SINGLE mode: go once and stay at target
            self.log("[SINGLE] Moving to X=%.2f  Y=%.2f  Z=%.2f  at FEED_DDA=%d"
                     % (target_x, target_y, target_z, self.feed_dda))
            steps_target = mm_to_steps(target_x, target_y, target_z)

            self._set_progress(True, "Move (single)", 0, 1)
            self._send_point(steps_target, "move to target")
            self._interruptible_sleep(DWELL_SEC)

            if self.stop_event.is_set():
                self.log("[Stop] Move interrupted; position tracking may be stale.")
            else:
                self.curr_x, self.curr_y, self.curr_z = target_x, target_y, target_z
            self._set_progress(False)

        else:
            # SWING mode: do bounce_count go-return cycles, end at start
            self.log("[SWING] Target X=%.2f  Y=%.2f  Z=%.2f  at FEED_DDA=%d, %d bounce(s)"
                     % (target_x, target_y, target_z, self.feed_dda, self.bounce_count))
            steps_target = mm_to_steps(target_x, target_y, target_z)
            steps_start = mm_to_steps(start_x, start_y, start_z)

            stopped = False
            self._set_progress(True, "Move (swing)", 0, self.bounce_count)
            for i in range(self.bounce_count):
                if self.stop_event.is_set():
                    stopped = True
                    break

                self.log("   Bounce %d/%d: start -> target" % (i + 1, self.bounce_count))
                self._send_point(steps_target, "move to target")
                self._interruptible_sleep(DWELL_SEC)
                if self.stop_event.is_set():
                    stopped = True
                    break

                self.log("   Bounce %d/%d: target -> start" % (i + 1, self.bounce_count))
                self._send_point(steps_start, "return move")
                self._interruptible_sleep(DWELL_SEC)
                self._set_progress(True, "Move (swing)", i + 1, self.bounce_count)

            if stopped:
                self.log("[Stop] Move interrupted; position tracking may be stale.")
            else:
                self.curr_x, self.curr_y, self.curr_z = start_x, start_y, start_z
                self.log("   Returned to start X=%.2f  Y=%.2f  Z=%.2f"
                         % (start_x, start_y, start_z))
            self._set_progress(False)

    def load_experiment_file(self, path):
        exp = load_experiment(path)
        self.experiment = exp
        self.experiment_path = path
        self.log("Loaded experiment condition from '%s': %s" % (path, exp))

    def save_experiment_file(self, path, values):
        save_experiment(path, values)
        self.experiment = values
        self.experiment_path = path
        self.log("Saved experiment condition to '%s': %s" % (path, values))

    # ----------------------------
    # Experiment / Indent Logic
    # ----------------------------
    def _goto_abs(self, x_mm, y_mm, z_mm, label=""):
        """Absolute move to (x,y,z) in mm (clamped), blocking with dwell."""
        if not self.connected or self.r is None:
            self.log("Cannot _goto_abs – not connected.")
            return

        if self.stop_event.is_set():
            self.log("[Stop] Skipping move; stop requested.")
            return

        x_mm, y_mm, z_mm = clamp_mm(x_mm, y_mm, z_mm)
        steps = mm_to_steps(x_mm, y_mm, z_mm)

        if label:
            self.log(label)
        self.log("   -> ABS X=%.2f Y=%.2f Z=%.2f  [steps=%s]  FEED_DDA=%d"
                 % (x_mm, y_mm, z_mm, steps, self.feed_dda))
        self._send_point(steps, "_goto_abs")

        self._interruptible_sleep(DWELL_SEC)
        if self.stop_event.is_set():
            self.log("[Stop] _goto_abs interrupted; position tracking may be stale.")
            return
        self.curr_x, self.curr_y, self.curr_z = x_mm, y_mm, z_mm

    def _indent_cycle(self, indent_mm, wait_sec):
        """
        One indent cycle at current XY:
          go down by indent_mm (towards 0), then back up.
        Assumes current Z is the 'top' position for the cycle.
        """
        if indent_mm <= 0:
            self.log("[Indent] indent_mm <= 0, skipping.")
            return

        z_top = self.curr_z
        z_down = z_top - indent_mm
        if z_down < 0.0:
            z_down = 0.0

        self._goto_abs(self.curr_x, self.curr_y, z_down, label="      [Indent] down")
        self._interruptible_sleep(wait_sec)

        if self.stop_event.is_set():
            return

        self._goto_abs(self.curr_x, self.curr_y, z_top, label="      [Indent] up")
        self._interruptible_sleep(wait_sec)

    def waypoint_indent(self, axis, start_x, start_y, start_z, step_mm, indent_mm, cycles,
                         repeats=1, wait_between_runs_min=0.0, wait_sec=None):
        """
        New behavior:
        For each run (`repeats` times):
           Loop `cycles` times:
             1. Go to start
             2. Indent (down and up)
             3. Go to waypoint (start + step_mm)
             4. Indent (down and up)
             5. Return to start point
           If repeats > 1: Wait `wait_between_runs_min`
        """
        if not self.connected or self.r is None:
            self.log("Cannot run experiment – not connected.")
            return

        axis = (axis or "").strip().lower()
        if axis not in ('x', 'y', 'z'):
            self.log("waypoint_indent: invalid axis '%s' (use x/y/z)" % axis)
            return

        try:
            cycles_i = int(cycles)
        except Exception:
            self.log("waypoint_indent: invalid cycles")
            return

        if cycles_i < 1 or cycles_i > 500:
            self.log("waypoint_indent: cycles out of range (1..500)")
            return

        try:
            repeats_i = int(repeats)
        except Exception:
            self.log("waypoint_indent: invalid repeats")
            return

        if repeats_i < 1 or repeats_i > 1000:
            self.log("waypoint_indent: repeats out of range (1..1000)")
            return

        try:
            wait_between_f = float(wait_between_runs_min)
        except Exception:
            self.log("waypoint_indent: invalid wait_between_runs_min")
            return

        if wait_between_f < 0.0:
            self.log("waypoint_indent: wait_between_runs_min cannot be negative")
            return

        try:
            step_f = float(step_mm)     # can be negative
            indent_f = float(indent_mm)
        except Exception:
            self.log("waypoint_indent: invalid step or indent")
            return

        if wait_sec is None:
            wait_f = INDENT_DWELL_SEC
        else:
            try:
                wait_f = float(wait_sec)
            except Exception:
                self.log("waypoint_indent: invalid wait_sec")
                return
            if wait_f < 0.0 or wait_f > 60.0:
                self.log("waypoint_indent: wait_sec out of range (0..60)")
                return

        self.stop_event.clear()

        def _check_stop():
            if self.stop_event.is_set():
                raise _MotionStopped()

        # Each cycle has 2 indentations (one at start, one at waypoint)
        total_actions = repeats_i * cycles_i * 2

        def worker():
            self.log("=== EXPERIMENT START (%d run(s)) ===" % repeats_i)
            self.log("Axis=%s start=(%.2f,%.2f,%.2f) step=%.2fmm indent=%.2fmm cycles/run=%d wait=%.2fs wait_runs=%.2fmin"
                     % (axis, start_x, start_y, start_z, step_f, indent_f, cycles_i, wait_f, wait_between_f))

            done = 0
            self._set_progress(True, "Experiment Running", 0, total_actions)
            
            try:
                # Calculate waypoint absolute coords
                wx, wy, wz = start_x, start_y, start_z
                if axis == 'x': wx += step_f
                elif axis == 'y': wy += step_f
                elif axis == 'z': wz += step_f

                for rep in range(repeats_i):
                    _check_stop()
                    
                    if rep > 0 and wait_between_f > 0:
                        self.log("--- Waiting %.2f minutes before next run ---" % wait_between_f)
                        self._set_progress(True, "Waiting before Run %d/%d" % (rep + 1, repeats_i), done, total_actions)
                        self._interruptible_sleep(wait_between_f * 60.0)
                        _check_stop()

                    self.log("--- Run %d/%d ---" % (rep + 1, repeats_i))

                    for c in range(cycles_i):
                        _check_stop()
                        self.log("   [Run %d, Cycle %d/%d] Go to start point" % (rep+1, c+1, cycles_i))
                        
                        # 1. Go to Start Point
                        self._goto_abs(start_x, start_y, start_z, label="")
                        self._interruptible_sleep(WAYPOINT_DWELL_SEC)
                        _check_stop()

                        # 2. Indent Free Node Zone
                        self.log("   [Run %d, Cycle %d/%d] Indenting free node zone" % (rep+1, c+1, cycles_i))
                        self._indent_cycle(indent_f, wait_f)
                        done += 1
                        self._set_progress(True, "Run %d/%d, Cycle %d/%d (Start)", done, total_actions)
                        _check_stop()

                        # 3. Go to Waypoint
                        self.log("   [Run %d, Cycle %d/%d] Go to waypoint" % (rep+1, c+1, cycles_i))
                        self._goto_abs(wx, wy, wz, label="")
                        self._interruptible_sleep(WAYPOINT_DWELL_SEC)
                        _check_stop()

                        # 4. Indent Waypoint
                        self.log("   [Run %d, Cycle %d/%d] Indenting waypoint" % (rep+1, c+1, cycles_i))
                        self._indent_cycle(indent_f, wait_f)
                        done += 1
                        self._set_progress(True, "Run %d/%d, Cycle %d/%d (Waypoint)", done, total_actions)
                        _check_stop()
                        
                        # 5. Return to Start Point
                        self.log("   [Run %d, Cycle %d/%d] Returning to start point" % (rep+1, c+1, cycles_i))
                        self._goto_abs(start_x, start_y, start_z, label="")
                        self._interruptible_sleep(WAYPOINT_DWELL_SEC)
                        _check_stop()
                        
            except _MotionStopped:
                self.log("=== EXPERIMENT STOPPED by user ===")
                self.log("[Warning] Position tracking may no longer match the physical carriage - re-home before continuing.")
                self._set_progress(False)
                return

            self._set_progress(False)
            self.log("=== EXPERIMENT COMPLETE ===")

        t = threading.Thread(target=worker)
        t.daemon = True
        t.start()


# Global controller instance
controller = Rep2XController()

# ---------------------------------------------------------------------
# FLASK APP
# ---------------------------------------------------------------------
app = Flask(__name__)


TEMPLATE = """
<!doctype html>
<html>
<head>
    <title>Replicator 2X Probe Control</title>
    <style>
        :root {
            --bg: #f4f6f8;
            --card-bg: #ffffff;
            --border: #dde2e8;
            --text: #1f2937;
            --muted: #6b7280;
            --accent: #2563eb;
            --accent-dark: #1d4ed8;
            --ok: #059669;
            --bad: #dc2626;
            --radius: 8px;
        }
        * { box-sizing: border-box; }
        body {
            font-family: -apple-system, "Segoe UI", Roboto, Arial, sans-serif;
            margin: 0;
            background: var(--bg);
            color: var(--text);
        }
        header {
            background: var(--card-bg);
            border-bottom: 1px solid var(--border);
            padding: 10px 24px;
            display: flex;
            align-items: center;
            justify-content: space-between;
            flex-wrap: wrap;
            gap: 10px;
            position: sticky;
            top: 0;
            z-index: 10;
        }
        header h1 {
            font-size: 18px;
            margin: 0;
            font-weight: 600;
        }
        .header-right { display: flex; align-items: center; gap: 14px; flex-wrap: wrap; }
        .pills { display: flex; gap: 8px; flex-wrap: wrap; }
        .pill {
            display: inline-flex;
            align-items: center;
            gap: 6px;
            padding: 4px 10px;
            border-radius: 999px;
            font-size: 12px;
            font-weight: 600;
            background: #f1f5f9;
            color: var(--muted);
        }
        .pill.on { background: #ecfdf5; color: var(--ok); }
        .pill.off { background: #fef2f2; color: var(--bad); }
        .pill .dot { width: 7px; height: 7px; border-radius: 50%; background: currentColor; }

        input[type="submit"].stop-btn {
            background: var(--bad);
            color: #fff;
            border: none;
            padding: 10px 22px;
            border-radius: 6px;
            font-size: 14px;
            font-weight: 800;
            letter-spacing: 0.04em;
            cursor: pointer;
        }
        input[type="submit"].stop-btn:hover { background: #b91c1c; }

        main {
            padding: 16px 20px;
            max-width: 1400px;
            margin: 0 auto;
        }
        .grid {
            columns: 320px 3;
            column-gap: 12px;
        }
        .card {
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: var(--radius);
            padding: 12px 14px;
            break-inside: avoid;
            display: inline-block;
            width: 100%;
            margin: 0 0 12px;
        }
        .card h2 {
            font-size: 12px;
            text-transform: uppercase;
            letter-spacing: 0.04em;
            color: var(--muted);
            margin: 0 0 8px 0;
            font-weight: 700;
        }
        .field {
            display: grid;
            grid-template-columns: 128px 1fr;
            align-items: center;
            column-gap: 8px;
            row-gap: 2px;
            margin-bottom: 6px;
        }
        label { font-size: 13px; color: var(--muted); }
        input[type="text"], select {
            padding: 5px 7px;
            border: 1px solid var(--border);
            border-radius: 6px;
            font-size: 13px;
            width: 100%;
            font-family: inherit;
        }
        input[type="text"]:focus, select:focus {
            outline: none;
            border-color: var(--accent);
            box-shadow: 0 0 0 3px rgba(37,99,235,0.12);
        }
        .radio-row { display: flex; gap: 16px; font-size: 13px; align-items: center; }
        .radio-row label { display: inline-flex; align-items: center; gap: 5px; color: var(--text); }
        .fh { grid-column: 2; font-size: 11px; color: var(--muted); line-height: 1.35; }

        button, input[type="submit"] {
            background: var(--accent);
            color: #fff;
            border: none;
            padding: 6px 13px;
            border-radius: 6px;
            font-size: 13px;
            font-weight: 600;
            cursor: pointer;
        }
        button:hover, input[type="submit"]:hover { background: var(--accent-dark); }
        button.secondary, input[type="submit"].secondary {
            background: #fff;
            color: var(--text);
            border: 1px solid var(--border);
        }
        button.secondary:hover, input[type="submit"].secondary:hover { background: #f1f5f9; }
        .btn-row { display: flex; gap: 8px; margin-top: 8px; flex-wrap: wrap; }

        .hint { color: var(--muted); font-size: 11px; margin-top: 6px; line-height: 1.45; }

        .progress-card.hidden { display: none; }
        .progress-meta { display: flex; justify-content: space-between; font-size: 12px; color: var(--muted); margin-bottom: 6px; }
        .progress-track { background: #e5e7eb; border-radius: 999px; height: 10px; overflow: hidden; }
        .progress-fill { background: var(--accent); height: 100%; width: 0%; transition: width 0.25s ease; }

        .log-card { margin-top: 0; }
        textarea {
            width: 100%;
            height: 220px;
            font-family: "SFMono-Regular", Consolas, Menlo, monospace;
            font-size: 12px;
            background: #0f172a;
            color: #d1d5db;
            border: none;
            border-radius: 6px;
            padding: 10px 12px;
            resize: vertical;
        }
    </style>
</head>
<body>
    <header>
        <h1>Replicator 2X Probe Control</h1>
        <div class="header-right">
            <div class="pills">
                <span class="pill {{ 'on' if connected else 'off' }}" id="pill-connected"><span class="dot"></span>{{ 'Connected' if connected else 'Disconnected' }}</span>
                <span class="pill {{ 'on' if homed else 'off' }}" id="pill-homed"><span class="dot"></span>{{ 'Homed' if homed else 'Not homed' }}</span>
                <span class="pill" id="pill-position">X={{ curr_x }} Y={{ curr_y }} Z={{ curr_z }}</span>
            </div>
            <form method="post" action="{{ url_for('stop') }}">
                <input type="submit" value="STOP" class="stop-btn">
            </form>
        </div>
    </header>

    <main>
    <div class="card progress-card{{ '' if progress.active else ' hidden' }}" id="progress-card">
        <h2>Task Progress</h2>
        <div class="progress-meta">
            <span id="progress-label">{{ progress.label }}</span>
            <span id="progress-pct">{{ progress.percent }}%</span>
        </div>
        <div class="progress-track">
            <div class="progress-fill" id="progress-fill" style="width: {{ progress.percent }}%;"></div>
        </div>
    </div>

    <div class="grid">

        <div class="card">
            <h2>Connection</h2>
            <form method="post" action="{{ url_for('connect') }}">
                <div class="field">
                    <label>Serial port</label>
                    <select name="port">
                    {% for p in ports %}
                        <option value="{{ p }}" {% if p == selected_port %}selected{% endif %}>{{ p }}</option>
                    {% endfor %}
                    </select>
                    <span class="fh">USB-serial device the printer is connected to</span>
                </div>
                <div class="btn-row">
                    <input type="submit" value="Connect">
                    <a href="{{ url_for('index') }}"><button type="button" class="secondary">Refresh Ports</button></a>
                </div>
            </form>
            <form method="post" action="{{ url_for('disconnect') }}" class="btn-row">
                <input type="submit" value="Disconnect" class="secondary">
            </form>
        </div>

        <div class="card">
            <h2>Calibration / Homing</h2>
            <p class="hint">Optional: you can also home from the front panel. Re-home after every reconnect.</p>
            <form method="post" action="{{ url_for('home') }}">
                <input type="submit" value="Auto-home (X max, Y max, Z min)">
            </form>
        </div>

        <div class="card">
            <h2>Motion Settings</h2>
            <form method="post" action="{{ url_for('settings') }}">
                <div class="field">
                    <label>Speed (FEED_DDA)</label>
                    <input type="text" name="speed" value="{{ feed_dda }}">
                    <span class="fh">Feed rate for all moves, DDA units (50-5000)</span>
                </div>
                <div class="field">
                    <label>Mode</label>
                    <div class="radio-row">
                        <label><input type="radio" name="mode" value="swing" {% if swing_mode %}checked{% endif %}> Swing</label>
                        <label><input type="radio" name="mode" value="single" {% if not swing_mode %}checked{% endif %}> Single</label>
                    </div>
                    <span class="fh">Swing = go &amp; return to start; Single = go &amp; stay</span>
                </div>
                <div class="field">
                    <label>Bounces (swing)</label>
                    <input type="text" name="bounces" value="{{ bounce_count }}">
                    <span class="fh">Go-return cycles per Move in Swing mode (1-100)</span>
                </div>
                <div class="btn-row"><input type="submit" value="Apply Settings"></div>
            </form>
        </div>

        <div class="card">
            <h2>Move Command</h2>
            <form method="post" action="{{ url_for('move') }}">
                <div class="field">
                    <label>X [mm]</label>
                    <input type="text" name="x" value="{{ curr_x }}">
                </div>
                <div class="field">
                    <label>Y [mm]</label>
                    <input type="text" name="y" value="{{ curr_y }}">
                </div>
                <div class="field">
                    <label>Z [mm]</label>
                    <input type="text" name="z" value="{{ curr_z }}">
                    <span class="fh">Absolute target position to move to</span>
                </div>
                <div class="btn-row"><input type="submit" value="Move"></div>
            </form>
        </div>

        <div class="card">
            <h2>Load Experiment Condition</h2>
            <form method="post" action="{{ url_for('load_experiment_route') }}">
                <div class="field">
                    <label>File path</label>
                    <input type="text" name="exp_path" value="{{ exp_path }}" placeholder="experiments/exp1.yaml">
                    <span class="fh">YAML file to read into the Experiment Runner below</span>
                </div>
                <div class="btn-row"><input type="submit" value="Load into form"></div>
            </form>
            <div class="hint">
                Only fills in the form — press "Run Experiment" there to execute it.
            </div>
        </div>

        <div class="card">
            <h2>Experiment Runner</h2>
            <form method="post" action="{{ url_for('waypoint_indent') }}">
                <div class="field">
                    <label>Axis</label>
                    <select name="axis">
                        <option value="x" {% if wp_axis == 'x' %}selected{% endif %}>X</option>
                        <option value="y" {% if wp_axis == 'y' %}selected{% endif %}>Y</option>
                        <option value="z" {% if wp_axis == 'z' %}selected{% endif %}>Z</option>
                    </select>
                    <span class="fh">Axis to step along to reach the waypoint</span>
                </div>
                <div class="field">
                    <label>Start X [mm]</label>
                    <input type="text" name="sx" value="{{ wp_sx }}">
                </div>
                <div class="field">
                    <label>Start Y [mm]</label>
                    <input type="text" name="sy" value="{{ wp_sy }}">
                </div>
                <div class="field">
                    <label>Start Z [mm]</label>
                    <input type="text" name="sz" value="{{ wp_sz }}">
                    <span class="fh">Position of the free node zone (Z = indent's "top")</span>
                </div>
                <div class="field">
                    <label>Step [mm]</label>
                    <input type="text" name="step" value="{{ wp_step }}">
                    <span class="fh">Distance from start to waypoint (negative = backwards)</span>
                </div>
                <div class="field">
                    <label>Cycles (Bounces)</label>
                    <input type="text" name="cycles" value="{{ wp_cycles }}">
                    <span class="fh">Number of times to bounce between start &amp; waypoint per run</span>
                </div>
                <div class="field">
                    <label>Indent Z [mm]</label>
                    <input type="text" name="indent" value="{{ wp_indent }}">
                    <span class="fh">How far to dip down in Z at the start and waypoint</span>
                </div>
                <div class="field">
                    <label>Total Runs</label>
                    <input type="text" name="repeats" value="{{ wp_repeats }}">
                    <span class="fh">Number of times to repeat the whole bouncing run (1-1000)</span>
                </div>
                <div class="field">
                    <label>Wait between [min]</label>
                    <input type="text" name="wait_between_runs_min" value="{{ wp_wait_between }}">
                    <span class="fh">Time to wait after a run finishes before starting the next</span>
                </div>
                <div class="field">
                    <label>Indent wait [s]</label>
                    <input type="text" name="wait_sec" value="{{ wp_wait_sec }}">
                    <span class="fh">Dwell at the bottom/top of each indent (0-60s)</span>
                </div>
                <div class="field">
                    <label>Save to path</label>
                    <input type="text" name="save_path" value="{{ exp_path }}" placeholder="experiments/exp1.yaml">
                    <span class="fh">Where "Save to File" below writes these values (overwrites)</span>
                </div>
                <div class="btn-row">
                    <input type="submit" value="Run Experiment">
                    <input type="submit" value="Save to File" class="secondary" formaction="{{ url_for('save_experiment_route') }}">
                </div>
            </form>
        </div>

    </div>

    <div class="card log-card">
        <h2>Log</h2>
        <textarea readonly>{{ log_text }}</textarea>
    </div>
    </main>

    <script>
        document.querySelectorAll('input[type="text"]').forEach(function (input) {
            input.addEventListener('keydown', function (e) {
                if (e.key === 'Enter') {
                    e.preventDefault();
                }
            });
        });

        function setPill(id, isOn, onText, offText) {
            var pill = document.getElementById(id);
            pill.classList.toggle('on', isOn);
            pill.classList.toggle('off', !isOn);
            pill.lastChild.textContent = isOn ? onText : offText;
        }

        function pollStatus() {
            fetch('{{ url_for("status") }}').then(function (r) { return r.json(); }).then(function (data) {
                var card = document.getElementById('progress-card');
                var p = data.progress;
                card.classList.toggle('hidden', !p.active);
                document.getElementById('progress-label').textContent = p.label;
                document.getElementById('progress-pct').textContent = p.percent + '%';
                document.getElementById('progress-fill').style.width = p.percent + '%';

                setPill('pill-connected', data.connected, 'Connected', 'Disconnected');
                setPill('pill-homed', data.homed, 'Homed', 'Not homed');
                document.getElementById('pill-position').textContent =
                    'X=' + data.curr_x + ' Y=' + data.curr_y + ' Z=' + data.curr_z;
            }).catch(function () {});
        }
        setInterval(pollStatus, 500);
        pollStatus();
    </script>
</body>
</html>
"""


@app.route("/status", methods=["GET"])
def status():
    return jsonify(
        connected=controller.connected,
        homed=controller.homed,
        curr_x=controller.curr_x,
        curr_y=controller.curr_y,
        curr_z=controller.curr_z,
        progress=controller.get_progress(),
    )


@app.route("/", methods=["GET"])
def index():
    ports_info = list(list_ports.comports())
    ports = [p.device for p in ports_info]

    usb_ports = [p.device for p in ports_info if p.vid is not None]
    acm_ports = [p.device for p in ports_info if "ttyACM" in p.device]

    if acm_ports:
        selected_port = acm_ports[0]
    elif usb_ports:
        selected_port = usb_ports[0]
    elif ports:
        selected_port = ports[0]
    else:
        selected_port = ""

    exp = controller.experiment
    return render_template_string(
        TEMPLATE,
        ports=ports,
        selected_port=selected_port,
        connected=controller.connected,
        homed=controller.homed,
        curr_x=controller.curr_x,
        curr_y=controller.curr_y,
        curr_z=controller.curr_z,
        feed_dda=controller.feed_dda,
        swing_mode=controller.swing_mode,
        bounce_count=controller.bounce_count,
        log_text=controller.get_log_text(),
        progress=controller.get_progress(),
        exp_path=controller.experiment_path,
        wp_axis=exp.get("axis", "x"),
        wp_sx=exp.get("start_x", controller.curr_x),
        wp_sy=exp.get("start_y", controller.curr_y),
        wp_sz=exp.get("start_z", controller.curr_z),
        wp_step=exp.get("step", 5.0),
        wp_indent=exp.get("indent", 2.0),
        wp_cycles=exp.get("cycles", 10),
        wp_repeats=exp.get("repeats", 1),
        wp_wait_between=exp.get("wait_between_runs_min", 0.0),
        wp_wait_sec=exp.get("wait_sec", INDENT_DWELL_SEC),
    )


@app.route("/connect", methods=["POST"])
def connect():
    port = request.form.get("port", "").strip()
    if not port:
        controller.log("No port selected for connect.")
        return redirect(url_for("index"))
    try:
        controller.connect(port)
    except Exception as e:
        controller.log("Connect failed: %r" % (e,))
    return redirect(url_for("index"))


@app.route("/disconnect", methods=["POST"])
def disconnect():
    controller.disconnect()
    return redirect(url_for("index"))


@app.route("/home", methods=["POST"])
def home():
    controller.auto_home()
    return redirect(url_for("index"))


@app.route("/stop", methods=["POST"])
def stop():
    controller.emergency_stop()
    return redirect(url_for("index"))


@app.route("/settings", methods=["POST"])
def settings():
    speed = request.form.get("speed", "").strip()
    mode = request.form.get("mode", "swing").strip()
    bounces = request.form.get("bounces", "").strip()

    try:
        v = int(speed)
        if 50 <= v <= 5000:
            controller.set_speed(v)
        else:
            controller.log("Speed %d out of range (50-5000), ignored." % v)
    except ValueError:
        controller.log("Could not parse speed '%s', ignored." % speed)

    if mode in ("swing", "single"):
        controller.set_mode(mode)
    else:
        controller.log("Unknown mode '%s', ignored." % mode)

    try:
        n = int(bounces)
        if 1 <= n <= 100:
            controller.set_bounce(n)
        else:
            controller.log("Bounce %d out of range (1-100), ignored." % n)
    except ValueError:
        controller.log("Could not parse bounce '%s', ignored." % bounces)

    return redirect(url_for("index"))


@app.route("/move", methods=["POST"])
def move():
    x = request.form.get("x", "0").strip()
    y = request.form.get("y", "0").strip()
    z = request.form.get("z", "0").strip()

    try:
        x_mm = float(x)
        y_mm = float(y)
        z_mm = float(z)
        controller.move(x_mm, y_mm, z_mm)
    except ValueError:
        controller.log("Move ignored: could not parse X/Y/Z '%s','%s','%s'." % (x, y, z))

    return redirect(url_for("index"))


@app.route("/load_experiment", methods=["POST"])
def load_experiment_route():
    path = request.form.get("exp_path", "").strip()
    if not path:
        controller.log("Load experiment: no file path given.")
        return redirect(url_for("index"))

    try:
        controller.load_experiment_file(path)
    except ValueError as e:
        controller.log("Load experiment failed: %s" % (e,))

    return redirect(url_for("index"))


def _parse_wp_form(form):
    return {
        "axis": form.get("axis", "x").strip().lower(),
        "start_x": float(form.get("sx", "0").strip()),
        "start_y": float(form.get("sy", "0").strip()),
        "start_z": float(form.get("sz", "0").strip()),
        "step": float(form.get("step", "1").strip()),
        "indent": float(form.get("indent", "1").strip()),
        "cycles": int(form.get("cycles", "10").strip()),
        "repeats": int(form.get("repeats", "1").strip()),
        "wait_between_runs_min": float(form.get("wait_between_runs_min", "0.0").strip()),
        "wait_sec": float(form.get("wait_sec", str(INDENT_DWELL_SEC)).strip()),
    }


@app.route("/waypoint_indent", methods=["POST"])
def waypoint_indent():
    try:
        v = _parse_wp_form(request.form)
        controller.waypoint_indent(v["axis"], v["start_x"], v["start_y"], v["start_z"], 
                                   v["step"], v["indent"], v["cycles"], v["repeats"], 
                                   v["wait_between_runs_min"], v["wait_sec"])
    except ValueError:
        controller.log("Experiment ignored: parse error in inputs.")

    return redirect(url_for("index"))


@app.route("/save_experiment", methods=["POST"])
def save_experiment_route():
    path = request.form.get("save_path", "").strip()
    if not path:
        controller.log("Save experiment: no file path given.")
        return redirect(url_for("index"))

    try:
        v = _parse_wp_form(request.form)
        controller.save_experiment_file(path, v)
    except ValueError as e:
        controller.log("Save experiment failed: %s" % (e,))

    return redirect(url_for("index"))


def _shutdown(*_args):
    sys.exit(0)


def main():
    atexit.register(controller.disconnect)
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    app.run(host="127.0.0.1", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()