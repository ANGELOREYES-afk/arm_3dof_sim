"""Locate and import the IM6014 motor code from github.com/sidrk-dev/im6014Development.

The bridge reuses that repo's packet codec, serial transaction, health gate and packet-level
simulator unchanged, so protocol fixes there apply here too.
"""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
REPO = 'https://github.com/sidrk-dev/im6014Development'


def load(path=None):
    candidates = [path, os.environ.get('IM6014_PATH'), ROOT / 'im6014Development', ROOT.parent / 'im6014Development']
    for c in candidates:
        if c and (Path(c) / 'spin_5rpm.py').is_file() and (Path(c) / 'motor_session.py').is_file():
            sys.path.insert(0, str(Path(c).resolve()))
            break
    else:
        raise SystemExit(f'IM6014 code not found. Run ./setup.sh, or: git clone {REPO} {ROOT / "im6014Development"}\n'
                         'or pass --im6014 PATH / set IM6014_PATH.')
    import spin_5rpm
    import motor_session
    return spin_5rpm, motor_session
