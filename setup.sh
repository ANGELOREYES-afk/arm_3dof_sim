#!/usr/bin/env bash
# One-time setup: fetch the IM6014 motor code next to the bridge and make a venv with pyserial.
# Prefers the private sidrk-dev/im6014-workbench (needs access); falls back to the public
# sidrk-dev/im6014Development. The bridge runs on either (see bridge/im6014.py).
set -eu
cd "$(dirname "$0")"
PY=${PYTHON:-python3}
if ! "$PY" -c 'import sys; sys.exit(sys.version_info < (3, 10))'; then
  echo "Needs Python 3.10 or newer (the workbench uses 3.10 syntax). Try: PYTHON=python3.12 ./setup.sh"
  exit 1
fi
if [ ! -d im6014-workbench ] && [ ! -d im6014Development ]; then
  if ! GIT_TERMINAL_PROMPT=0 git clone -q https://github.com/sidrk-dev/im6014-workbench.git im6014-workbench; then
    echo "im6014-workbench is private and this account cannot read it; using the public im6014Development."
    git clone -q https://github.com/sidrk-dev/im6014Development.git im6014Development
  fi
fi
"$PY" -m venv .venv
.venv/bin/python -m pip install -q -r requirements.txt
echo "Ready. Start the simulator + motor bridge with:"
echo "  .venv/bin/python bridge/arm_bridge.py      # then open http://127.0.0.1:8765"
