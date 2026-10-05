"""Loads and saves Waypoints+Indent experiment conditions as YAML files, so
a run can be prepared/reviewed ahead of time and re-loaded later.

Expected file layout (all keys optional, fall back to the field's usual
GUI default when omitted):

    axis: x
    start_x: 10.0
    start_y: 20.0
    start_z: 5.0
    step: 5.0
    cycles: 10
    indent: 2.0
    repeats: 1
    wait_between_runs_min: 0.0
    wait_sec: 0.15
"""

import json

import yaml

# name -> (unit, short description). Order here is also the order fields
# are written out by save_experiment().
FIELD_INFO = {
    "axis": ("", "Axis to step along to reach the waypoint: x, y, or z"),
    "start_x": ("mm", "Starting X position"),
    "start_y": ("mm", "Starting Y position"),
    "start_z": ("mm", "Starting Z position (top of indent)"),
    "step": ("mm", "Distance from start to the waypoint (negative moves backwards)"),
    "indent": ("mm", "How far to dip down in Z at the start and waypoint"),
    "cycles": ("cycles", "Number of start-to-waypoint bounces per run"),
    "repeats": ("repeats", "Number of times to repeat the entire run sequence"),
    "wait_between_runs_min": ("min", "Time to wait between repeated runs (in minutes)"),
    "wait_sec": ("s", "Dwell time at the bottom and top of each indent cycle"),
}

FIELDS = tuple(FIELD_INFO)


def load_experiment(path):
    """Read and validate a waypoint+indent experiment YAML file.

    Returns a dict with only the keys present in FIELDS that were set in
    the file. Raises ValueError on missing file, bad YAML, or unknown keys.
    """
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except OSError as e:
        raise ValueError("could not read '%s': %s" % (path, e))
    except yaml.YAMLError as e:
        raise ValueError("invalid YAML in '%s': %s" % (path, e))

    if not isinstance(data, dict):
        raise ValueError("'%s' must contain a YAML mapping of experiment fields" % path)

    unknown = set(data) - set(FIELDS)
    if "count" in unknown:
        unknown.remove("count")
    if unknown:
        raise ValueError("unknown experiment field(s) in '%s': %s" % (path, ", ".join(sorted(unknown))))

    return {k: data[k] for k in FIELDS if k in data}


def save_experiment(path, values):
    """Write experiment fields out as a commented YAML file.

    'values' is a dict covering (a subset of) FIELDS; each written line is
    preceded by a '# <unit> - <description>' comment so the file is
    self-documenting. Raises ValueError if the path can't be written.
    """
    lines = []
    for name in FIELDS:
        if name not in values:
            continue
        unit, desc = FIELD_INFO[name]
        comment = ("# [%s] %s" % (unit, desc)) if unit else ("# %s" % desc)
        lines.append(comment)
        # json.dumps gives a bare scalar (no "..." document-end marker that
        # yaml.safe_dump adds); JSON scalars are valid YAML.
        lines.append("%s: %s" % (name, json.dumps(values[name])))
        lines.append("")

    try:
        with open(path, "w") as f:
            f.write("\n".join(lines).rstrip() + "\n")
    except OSError as e:
        raise ValueError("could not write '%s': %s" % (path, e))