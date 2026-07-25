#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Web GUI control for MakerBot Replicator 2X (Sailfish) from Python 2.7.

Features:
- Auto-detect serial ports and let the user select one.
- Connect / disconnect buttons.
- Optional auto-homing (calibration) via a button.
- Speed (FEED_DDA) control.
- Swing / Single mode selection.
- Bounce count (how many go-return cycles in swing mode).
- X/Y/Z target fields + "Move" button.
- Waypoints + Indent:
    * choose axis (X/Y/Z), start position, step size (can be negative), waypoint count,
      indent amount, and number of indentation cycles per waypoint.

Open http://127.0.0.1:5000 in your browser after starting this script.
"""

import time
import threading

import serial
from serial.tools import list_ports

import makerbot_driver
from makerbot_driver import errors as mb_errors

from flask import Flask, request, redirect, url_for, render_template_string

# ---------------------------------------------------------------------
# CONFIGURATION / CONSTANTS
# ---------------------------------------------------------------------
BAUD = 115200

# Replicator 2/2X nominal mechanics
XY_STEPS_PER_MM = 88.8889   # documented Rep2/2X value
Z_STEPS_PER_MM  = 400.0     # typical Z lead screw

DEFAULT_FEED_DDA = 600      # default motion speed (DDA units)
DWELL_SEC        = 0.5      # pause after each move (s)

# Waypoint+Indent tuning (UI controls the repeat count now)
INDENT_DWELL_SEC   = 0.15   # pause between down/up moves (tune)
WAYPOINT_DWELL_SEC = 0.30   # pause after reaching each waypoint (tune)

# How long to wait after each homing command (seconds)
HOME_WAIT_X = 6.0
HOME_WAIT_Y = 6.0
HOME_WAIT_Z = 6.0

# Safety soft-limits in mm (keep inside the real build volume)
MAX_X_MM = 220.0    # real is ~246, we stay a bit inside
MAX_Y_MM = 140.0    # real is ~152, we stay inside
MAX_Z_MM = 150.0    # arbitrary safe cap; adjust if needed


# ---------------------------------------------------------------------
# Condition wrapper so StreamWriter can use "with self._condition:" on 2.7
# ---------------------------------------------------------------------
import threading as _threading

class ConditionWrapper(object):
    def __init__(self):
        self._cond = _threading.Condition()

    def __enter__(self):
        self._cond.acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._cond.release()

    def wait(self, timeout=None):
        self._cond.wait(timeout)

    def notify(self):
        self._cond.notify()

    def notifyAll(self):
        self._cond.notifyAll()

    def acquire(self):
        self._cond.acquire()

    def release(self):
        self._cond.release()


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


def home_axis_max(r, axis, wait_s, log_func):
    """Home given axis towards maximum, then sleep."""
    log_func("Homing %s towards MAX..." % axis.upper())
    try:
        r.find_axes_maximums([axis], 500, 60)
    except mb_errors.TransmissionError as e:
        log_func("[Warning] TransmissionError during %s homing: %r" % (axis, e))
        log_func("If the carriage hit the %s endstop, this is probably OK." % axis.upper())
    time.sleep(wait_s)
    log_func("%s homing wait done." % axis.upper())


def home_axis_min(r, axis, wait_s, log_func):
    """Home given axis towards minimum, then sleep."""
    log_func("Homing %s towards MIN..." % axis.upper())
    try:
        r.find_axes_minimums([axis], 500, 60)
    except mb_errors.TransmissionError as e:
        log_func("[Warning] TransmissionError during %s homing: %r" % (axis, e))
        log_func("If the axis hit the %s endstop, this is probably OK." % axis.upper())
    time.sleep(wait_s)
    log_func("%s homing wait done." % axis.upper())


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
        self.swing_mode = True  # True = swing, False = single
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0

        self.log_lines = []
        self.lock = threading.Lock()

    def log(self, msg):
        with self.lock:
            line = time.strftime("[%H:%M:%S] ") + msg
            # Strip non-ASCII characters so Flask/Jinja don't crash on Py2
            clean = ''.join((c if ord(c) < 128 else '?') for c in line)
            print(clean)
            self.log_lines.append(clean)
            # keep last 200 lines
            if len(self.log_lines) > 200:
                self.log_lines = self.log_lines[-200:]

    def get_log_text(self):
        with self.lock:
            return "\n".join(self.log_lines)

    def connect(self, port):
        if self.connected:
            self.log("Already connected.")
            return

        try:
            self.ser = serial.Serial(port, BAUD, timeout=1)
        except Exception as e:
            self.log("Failed to open port %s: %r" % (port, e))
            raise

        try:
            self.r = makerbot_driver.s3g()
            self.r.writer = makerbot_driver.Writer.StreamWriter(self.ser, True)
            self.r.writer._condition = ConditionWrapper()
        except Exception as e:
            self.log("Failed to init makerbot_driver: %r" % (e,))
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            raise

        self.connected = True
        self.homed = False
        self.curr_x = self.curr_y = self.curr_z = 0.0
        self.log("Connected to Replicator 2X on %s @ %d baud." % (port, BAUD))

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

            home_axis_max(self.r, 'x', HOME_WAIT_X, self.log)
            home_axis_max(self.r, 'y', HOME_WAIT_Y, self.log)
            home_axis_min(self.r, 'z', HOME_WAIT_Z, self.log)

            self.log("Recalling home positions from firmware...")
            try:
                self.r.recall_home_positions(['x', 'y', 'z', 'a', 'b'])
                self.log("recall_home_positions sent.")
            except mb_errors.TransmissionError as e:
                self.log("[Warning] TransmissionError during recall_home_positions: %r" % (e,))
                self.log("(On some Sailfish builds this still works despite the warning.)")

            time.sleep(2.0)

            self.homed = True
            self.curr_x = self.curr_y = self.curr_z = 0.0
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

    def move(self, x_mm, y_mm, z_mm):
        if not self.connected or self.r is None:
            self.log("Cannot move – not connected.")
            return

        # Clamp to safe region
        x_mm, y_mm, z_mm = clamp_mm(x_mm, y_mm, z_mm)

        start_x, start_y, start_z = self.curr_x, self.curr_y, self.curr_z
        target_x, target_y, target_z = x_mm, y_mm, z_mm

        if not self.swing_mode:
            # SINGLE mode: go once and stay at target
            self.log("[SINGLE] Moving to X=%.2f  Y=%.2f  Z=%.2f  at FEED_DDA=%d"
                     % (target_x, target_y, target_z, self.feed_dda))
            steps_target = mm_to_steps(target_x, target_y, target_z)
            self.log("   [debug] target steps: %s" % (steps_target,))

            try:
                self.r.queue_extended_point(steps_target, self.feed_dda, 0, 0)
            except mb_errors.TransmissionError as e:
                self.log("[Warning] TransmissionError during move to target: %r" % (e,))
            except Exception as e:
                self.log("[Error] Other exception during move to target: %r" % (e,))

            time.sleep(DWELL_SEC)
            self.curr_x, self.curr_y, self.curr_z = target_x, target_y, target_z

        else:
            # SWING mode: do bounce_count go-return cycles, end at start
            self.log("[SWING] Target X=%.2f  Y=%.2f  Z=%.2f  at FEED_DDA=%d, %d bounce(s)"
                     % (target_x, target_y, target_z, self.feed_dda, self.bounce_count))
            steps_target = mm_to_steps(target_x, target_y, target_z)
            steps_start  = mm_to_steps(start_x,  start_y,  start_z)
            self.log("   [debug] start steps:  %s" % (steps_start,))
            self.log("   [debug] target steps: %s" % (steps_target,))

            for i in range(self.bounce_count):
                self.log("   Bounce %d/%d: start -> target" % (i + 1, self.bounce_count))
                try:
                    self.r.queue_extended_point(steps_target, self.feed_dda, 0, 0)
                except mb_errors.TransmissionError as e:
                    self.log("    [Warning] TransmissionError during move to target: %r" % (e,))
                except Exception as e:
                    self.log("    [Error] Other exception during move to target: %r" % (e,))
                time.sleep(DWELL_SEC)

                self.log("   Bounce %d/%d: target -> start" % (i + 1, self.bounce_count))
                try:
                    self.r.queue_extended_point(steps_start, self.feed_dda, 0, 0)
                except mb_errors.TransmissionError as e:
                    self.log("    [Warning] TransmissionError during return move: %r" % (e,))
                except Exception as e:
                    self.log("    [Error] Other exception during return move: %r" % (e,))
                time.sleep(DWELL_SEC)

            self.curr_x, self.curr_y, self.curr_z = start_x, start_y, start_z
            self.log("   Returned to start X=%.2f  Y=%.2f  Z=%.2f"
                     % (start_x, start_y, start_z))

    # ----------------------------
    # Waypoints + Indent (updated)
    # ----------------------------
    def _goto_abs(self, x_mm, y_mm, z_mm, label=""):
        """Absolute move to (x,y,z) in mm (clamped), blocking with dwell."""
        if not self.connected or self.r is None:
            self.log("Cannot _goto_abs – not connected.")
            return

        x_mm, y_mm, z_mm = clamp_mm(x_mm, y_mm, z_mm)
        steps = mm_to_steps(x_mm, y_mm, z_mm)

        if label:
            self.log(label)
        self.log("   -> ABS X=%.2f Y=%.2f Z=%.2f  [steps=%s]  FEED_DDA=%d"
                 % (x_mm, y_mm, z_mm, steps, self.feed_dda))
        try:
            self.r.queue_extended_point(steps, self.feed_dda, 0, 0)
        except mb_errors.TransmissionError as e:
            self.log("[Warning] TransmissionError during _goto_abs: %r" % (e,))
        except Exception as e:
            self.log("[Error] Other exception during _goto_abs: %r" % (e,))

        time.sleep(DWELL_SEC)
        self.curr_x, self.curr_y, self.curr_z = x_mm, y_mm, z_mm

    def _indent_cycle(self, indent_mm):
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

        self._goto_abs(self.curr_x, self.curr_y, z_down, label="   [Indent] down")
        time.sleep(INDENT_DWELL_SEC)

        self._goto_abs(self.curr_x, self.curr_y, z_top, label="   [Indent] up")
        time.sleep(INDENT_DWELL_SEC)

    def waypoint_indent(self, axis, start_x, start_y, start_z, step_mm, count, indent_mm, cycles):
        """
        Move along 'axis' starting at (start_x,start_y,start_z),
        stepping by step_mm for 'count' waypoints.
        At each waypoint: perform 'cycles' indent cycles (down/up in Z).

        IMPORTANT: step_mm can be NEGATIVE to scan backwards.
        axis: 'x' or 'y' or 'z'
        """
        if not self.connected or self.r is None:
            self.log("Cannot run waypoint_indent – not connected.")
            return

        axis = (axis or "").strip().lower()
        if axis not in ('x', 'y', 'z'):
            self.log("waypoint_indent: invalid axis '%s' (use x/y/z)" % axis)
            return

        try:
            count_i = int(count)
        except Exception:
            self.log("waypoint_indent: invalid count")
            return

        try:
            cycles_i = int(cycles)
        except Exception:
            self.log("waypoint_indent: invalid cycles")
            return

        if count_i < 1 or count_i > 500:
            self.log("waypoint_indent: count out of range (1..500)")
            return

        if cycles_i < 1 or cycles_i > 500:
            self.log("waypoint_indent: cycles out of range (1..500)")
            return

        try:
            step_f = float(step_mm)     # can be negative
            indent_f = float(indent_mm)
        except Exception:
            self.log("waypoint_indent: invalid step or indent")
            return

        def worker():
            self.log("=== WAYPOINT+INDENT START ===")
            self.log("Axis=%s  start=(%.2f,%.2f,%.2f)  step=%.2fmm  count=%d  indent=%.2fmm  cycles=%d"
                     % (axis, start_x, start_y, start_z, step_f, count_i, indent_f, cycles_i))

            self._goto_abs(start_x, start_y, start_z, label="[Waypoints] go to start")
            time.sleep(WAYPOINT_DWELL_SEC)

            for i in range(count_i):
                wx, wy, wz = start_x, start_y, start_z
                delta = i * step_f  # negative works too

                if axis == 'x':
                    wx = start_x + delta
                elif axis == 'y':
                    wy = start_y + delta
                elif axis == 'z':
                    wz = start_z + delta

                self.log("[Waypoints] %d/%d -> (%.2f, %.2f, %.2f)" % (i + 1, count_i, wx, wy, wz))
                self._goto_abs(wx, wy, wz, label="   [Waypoints] move to waypoint")
                time.sleep(WAYPOINT_DWELL_SEC)

                for k in range(cycles_i):
                    self.log("   [Indent] cycle %d/%d" % (k + 1, cycles_i))
                    self._indent_cycle(indent_f)

            self.log("=== WAYPOINT+INDENT COMPLETE ===")

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
        body { font-family: sans-serif; margin: 20px; }
        fieldset { margin-bottom: 15px; }
        legend { font-weight: bold; }
        label { display:inline-block; width: 170px; }
        textarea { width: 100%; height: 250px; font-family: monospace; font-size: 11px; }
        input[type="text"], select { padding: 2px; }
        .hint { color: #555; font-size: 12px; margin-top: 6px; }
    </style>
</head>
<body>
    <h1>Replicator 2X Probe Control</h1>

    <form method="post" action="{{ url_for('connect') }}">
        <fieldset>
            <legend>Connection</legend>
            <label>Serial port:</label>
            <select name="port">
            {% for p in ports %}
                <option value="{{ p }}" {% if p == selected_port %}selected{% endif %}>{{ p }}</option>
            {% endfor %}
            </select>
            <input type="submit" value="Connect">
            <a href="{{ url_for('index') }}"><button type="button">Refresh Ports</button></a>
        </fieldset>
    </form>

    <form method="post" action="{{ url_for('disconnect') }}">
        <fieldset>
            <legend>Status</legend>
            <p>Connected: <strong>{{ 'YES' if connected else 'NO' }}</strong></p>
            <p>Homed: <strong>{{ 'YES' if homed else 'NO' }}</strong></p>
            <p>Current position (mm): X={{ curr_x }}, Y={{ curr_y }}, Z={{ curr_z }}</p>
            <input type="submit" value="Disconnect">
        </fieldset>
    </form>

    <form method="post" action="{{ url_for('home') }}">
        <fieldset>
            <legend>Calibration / Homing</legend>
            <p>Optional: you can also home from the front panel.</p>
            <input type="submit" value="Auto-home (X max, Y max, Z min)">
        </fieldset>
    </form>

    <form method="post" action="{{ url_for('settings') }}">
        <fieldset>
            <legend>Motion Settings</legend>
            <p>
                <label>Speed (FEED_DDA):</label>
                <input type="text" name="speed" value="{{ feed_dda }}" size="6">
            </p>
            <p>
                <label>Mode:</label>
                <input type="radio" name="mode" value="swing" {% if swing_mode %}checked{% endif %}> Swing (go & return)
                <input type="radio" name="mode" value="single" {% if not swing_mode %}checked{% endif %}> Single (go & stay)
            </p>
            <p>
                <label>Bounces (swing only):</label>
                <input type="text" name="bounces" value="{{ bounce_count }}" size="4">
            </p>
            <input type="submit" value="Apply Settings">
        </fieldset>
    </form>

    <form method="post" action="{{ url_for('move') }}">
        <fieldset>
            <legend>Move Command</legend>
            <p>
                <label>X [mm]:</label>
                <input type="text" name="x" value="{{ curr_x }}" size="6">
            </p>
            <p>
                <label>Y [mm]:</label>
                <input type="text" name="y" value="{{ curr_y }}" size="6">
            </p>
            <p>
                <label>Z [mm]:</label>
                <input type="text" name="z" value="{{ curr_z }}" size="6">
            </p>
            <input type="submit" value="Move">
        </fieldset>
    </form>

    <form method="post" action="{{ url_for('waypoint_indent') }}">
        <fieldset>
            <legend>Waypoints + Indent</legend>

            <p>
                <label>Axis:</label>
                <select name="axis">
                    <option value="x">X</option>
                    <option value="y">Y</option>
                    <option value="z">Z</option>
                </select>
            </p>

            <p>
                <label>Start X [mm]:</label>
                <input type="text" name="sx" value="{{ curr_x }}" size="6">
            </p>
            <p>
                <label>Start Y [mm]:</label>
                <input type="text" name="sy" value="{{ curr_y }}" size="6">
            </p>
            <p>
                <label>Start Z [mm]:</label>
                <input type="text" name="sz" value="{{ curr_z }}" size="6">
            </p>

            <p>
                <label>Step [mm] (can be -):</label>
                <input type="text" name="step" value="5.0" size="6">
            </p>

            <p>
                <label>Waypoints (count):</label>
                <input type="text" name="count" value="5" size="6">
            </p>

            <p>
                <label>Indent Z [mm]:</label>
                <input type="text" name="indent" value="2.0" size="6">
            </p>

            <p>
                <label>Indent cycles per point:</label>
                <input type="text" name="cycles" value="10" size="6">
            </p>

            <div class="hint">
                Tip: Use a negative step to move in -X / -Y direction (e.g., step = -5.0).
            </div>

            <input type="submit" value="Run Waypoints + Indent">
        </fieldset>
    </form>

    <fieldset>
        <legend>Log</legend>
        <textarea readonly>{{ log_text }}</textarea>
    </fieldset>

</body>
</html>
"""


@app.route("/", methods=["GET"])
def index():
    ports_info = list(list_ports.comports())
    ports = []
    for p in ports_info:
        dev = getattr(p, "device", None) or p[0]
        ports.append(dev)

    selected_port = ports[0] if ports else ""

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
            controller.log("Speed %d out of range (50–5000), ignored." % v)
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
            controller.log("Bounce %d out of range (1–100), ignored." % n)
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


@app.route("/waypoint_indent", methods=["POST"])
def waypoint_indent():
    axis = request.form.get("axis", "x").strip().lower()

    sx = request.form.get("sx", "0").strip()
    sy = request.form.get("sy", "0").strip()
    sz = request.form.get("sz", "0").strip()

    step = request.form.get("step", "1").strip()
    count = request.form.get("count", "1").strip()
    indent = request.form.get("indent", "1").strip()
    cycles = request.form.get("cycles", "10").strip()

    try:
        start_x = float(sx)
        start_y = float(sy)
        start_z = float(sz)
        step_mm = float(step)       # can be negative
        indent_mm = float(indent)
        cycles_i = int(cycles)

        controller.waypoint_indent(axis, start_x, start_y, start_z, step_mm, count, indent_mm, cycles_i)
    except ValueError:
        controller.log("Waypoint+Indent ignored: parse error in inputs.")

    return redirect(url_for("index"))


def main():
    app.run(host="127.0.0.1", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()
