"""Loads motion/hardware constants from config.yaml (falling back to
built-in defaults for any missing keys, so a partial or absent config file
still works).
"""

import os

import yaml

DEFAULTS = {
    'serial': {'baud': 115200},
    'steps_per_mm': {'xy': 88.8889, 'z': 400.0},
    'motion': {'default_feed_dda': 600, 'dwell_sec': 0.5},
    'indent': {'dwell_sec': 0.15, 'waypoint_dwell_sec': 0.30},
    'homing': {'wait_x': 6.0, 'wait_y': 6.0, 'wait_z': 6.0},
    'soft_limits': {'max_x_mm': 220.0, 'max_y_mm': 140.0, 'max_z_mm': 150.0},
    'logging': {'file': 'logs/motion_commands.log'},
}

_this_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(os.path.dirname(_this_dir))


def get_default_config_path():
    candidates = [
        os.path.join(os.getcwd(), 'config.yaml'),
        os.path.join(_repo_root, 'config.yaml'),
        os.path.join(_this_dir, 'config.yaml'),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return os.path.join(_repo_root, 'config.yaml')


DEFAULT_CONFIG_PATH = get_default_config_path()


def _merge(defaults, overrides):
    merged = dict(defaults)
    for key, value in (overrides or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge(merged[key], value)
        else:
            merged[key] = value
    return merged


class Config(object):
    def __init__(self, data):
        self.baud = data['serial']['baud']
        self.xy_steps_per_mm = data['steps_per_mm']['xy']
        self.z_steps_per_mm = data['steps_per_mm']['z']
        self.default_feed_dda = data['motion']['default_feed_dda']
        self.dwell_sec = data['motion']['dwell_sec']
        self.indent_dwell_sec = data['indent']['dwell_sec']
        self.waypoint_dwell_sec = data['indent']['waypoint_dwell_sec']
        self.home_wait_x = data['homing']['wait_x']
        self.home_wait_y = data['homing']['wait_y']
        self.home_wait_z = data['homing']['wait_z']
        self.max_x_mm = data['soft_limits']['max_x_mm']
        self.max_y_mm = data['soft_limits']['max_y_mm']
        self.max_z_mm = data['soft_limits']['max_z_mm']
        self.log_file = data['logging']['file']


def load_config(path=None):
    path = path or get_default_config_path()
    file_data = {}
    if os.path.exists(path):
        with open(path) as f:
            file_data = yaml.safe_load(f) or {}
    return Config(_merge(DEFAULTS, file_data))
