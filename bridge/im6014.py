"""Locate and import the IM6014 motor code.

Prefers sidrk-dev/im6014-workbench (private; the successor) and falls back to the public
sidrk-dev/im6014Development. Both provide the same packet codec, `exchange`, `healthy`,
`DemoSerial` and `fault_text`, so the bridge runs on either. With the workbench it also gets:

  im6014_serial   an exchange that resyncs past corrupt or foreign-ID frames on a shared bus and
                  enforces its deadline, typed timeout errors with rejected-frame counts, and a
                  packet-capture ring for diagnostics
  joint_mapping   the joint <-> actuator-output algebra (JointMap)
  im6014_port     the per-device bus lease (Linux)

The code is imported unchanged, so protocol fixes there apply here too.
"""
from dataclasses import dataclass
import importlib
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
REPOS = ('im6014-workbench', 'im6014Development')


@dataclass
class Im6014:
    codec: object             # spin_5rpm: command, exchange, healthy, crc32, RATIO
    sim: object               # motor_session: DemoSerial, fault_text
    serial: object = None     # im6014_serial (workbench only)
    mapping: object = None    # joint_mapping (workbench only)
    port: object = None       # im6014_port (workbench only)
    path: Path = None

    @property
    def name(self):
        return 'im6014-workbench' if self.serial else 'im6014Development'


def _candidates(path):
    yield path
    yield os.environ.get('IM6014_PATH')
    for parent in (ROOT, ROOT.parent):
        for repo in REPOS:
            yield parent / repo


def load(path=None):
    for c in _candidates(path):
        if c and (Path(c) / 'spin_5rpm.py').is_file() and (Path(c) / 'motor_session.py').is_file():
            path = Path(c).resolve()
            break
    else:
        raise SystemExit('IM6014 code not found. Run ./setup.sh, or clone sidrk-dev/im6014-workbench (or the public '
                         f'sidrk-dev/im6014Development) into {ROOT}, or pass --im6014 PATH / set IM6014_PATH.')
    if sys.version_info < (3, 10) and (path / 'im6014_serial.py').is_file():
        raise SystemExit(f'{path.name} needs Python 3.10+ (this is {sys.version.split()[0]}).')
    sys.path.insert(0, str(path))
    optional = {}
    for attr, module in (('serial', 'im6014_serial'), ('mapping', 'joint_mapping'), ('port', 'im6014_port')):
        if (path / f'{module}.py').is_file():
            optional[attr] = importlib.import_module(module)
    return Im6014(codec=importlib.import_module('spin_5rpm'), sim=importlib.import_module('motor_session'),
                  path=path, **optional)
