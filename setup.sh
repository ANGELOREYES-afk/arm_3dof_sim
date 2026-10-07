#!/usr/bin/env bash
# One-time setup: clone the IM6014 motor code next to the bridge and make a venv with pyserial.
set -eu
cd "$(dirname "$0")"
[ -d im6014Development ] || git clone https://github.com/sidrk-dev/im6014Development.git im6014Development
python3 -m venv .venv
.venv/bin/python -m pip install -q -r requirements.txt
echo "Ready. Start the simulator + motor bridge with:"
echo "  .venv/bin/python bridge/arm_bridge.py      # then open http://127.0.0.1:8765"
