"""
Reads/writes place_position_<color> in urxp_pick_place's config yaml — this
is what makes a UI edit survive a restart, on top of ros_bridge.py pushing
the same values live via the parameter service.

Edits the SOURCE file (not the installed copy) with a targeted regex
substitution rather than a full YAML re-dump, so the file's comments and
formatting survive untouched.
"""

import os
import re
import yaml

DEFAULT_CONFIG_PATH = os.path.expanduser(
    '~/URXP_ws/src/urxp_pick_place/config/pick_place_params.yaml')
CONFIG_PATH = os.environ.get('URXP_PICK_PLACE_CONFIG', DEFAULT_CONFIG_PATH)

COLORS = ('red', 'green', 'blue')


def read_place_positions() -> dict:
    """Returns {'red': [x,y,z], ...} straight from the yaml file (used as a
    fallback when the live node isn't up to ask via its parameter service)."""
    try:
        with open(CONFIG_PATH) as f:
            data = yaml.safe_load(f)
        params = data['urxp_pick_place_server']['ros__parameters']
        return {c: list(params[f'place_position_{c}']) for c in COLORS if f'place_position_{c}' in params}
    except Exception:
        return {}


def write_place_positions(positions: dict) -> tuple[bool, str]:
    """positions: {'red': [x,y,z], ...} (subset ok). Rewrites only the
    matching lines in-place, preserving everything else in the file."""
    try:
        with open(CONFIG_PATH) as f:
            text = f.read()
    except Exception as e:
        return False, f'Could not read {CONFIG_PATH}: {e}'

    for color, xyz in positions.items():
        if color not in COLORS or len(xyz) != 3:
            continue
        formatted = '[{:.4f}, {:.4f}, {:.4f}]'.format(*xyz)
        pattern = re.compile(
            r'^(\s*place_position_{}:\s*)\[[^\]]*\]'.format(re.escape(color)),
            re.MULTILINE)
        if not pattern.search(text):
            return False, f'place_position_{color} not found in {CONFIG_PATH}'
        text = pattern.sub(lambda m: m.group(1) + formatted, text)

    try:
        with open(CONFIG_PATH, 'w') as f:
            f.write(text)
    except Exception as e:
        return False, f'Could not write {CONFIG_PATH}: {e}'
    return True, 'Saved to config file'
