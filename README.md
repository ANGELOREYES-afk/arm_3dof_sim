# 3-DOF arm simulator · IK · Xbox control · IM6014 bridge · auto-calibration

A browser simulation of the yaw–shoulder–elbow arm from its Onshape URDF export, with closed-form
inverse kinematics and Xbox controller input. A Python bridge streams joint targets to three Unitree
IM6014 actuators on one RS-485 bus. It calibrates the joints' absolute output encoders against
contacts predicted from the CAD. The motor transport, packet codec and joint mapping come from
**sidrk-dev/im6014-workbench**, with the public
[sidrk-dev/im6014Development](https://github.com/sidrk-dev/im6014Development) as a fallback.

![Driving the demo motors after calibration](docs/driving.jpg)

```
Xbox pad ─► browser sim (three.js, URDF meshes, IK, collisions) ─► WebSocket ─► bridge/arm_bridge.py
                                                    trajectory, effort guard, calibration │
                                          im6014-workbench: exchange · JointMap · lease  ▼
                                                               RS-485 ─► IM6014 IDs 0, 1, 2  (or demo motors)
```

## Run it

```bash
./setup.sh                              # fetches the IM6014 code, makes .venv with pyserial (Python 3.10+)
.venv/bin/python bridge/arm_bridge.py   # serves the sim + bridge on http://127.0.0.1:8765
```

Open <http://127.0.0.1:8765>. The motors stay disconnected until you connect them in the page.
For the simulation alone, any static server works (`python3 -m http.server`, then open `/web/`).

`setup.sh` clones **im6014-workbench**, which is private, so it needs a GitHub account with access.
If that fails it falls back to the public im6014Development. The bridge prints which one it loaded,
and the Motors panel shows it as *Transport*. Workbench code is only fetched at setup and is never
copied into this repo.

**Xbox controller:** pair it over Bluetooth, then press any button while the page is open, because browsers
only expose a gamepad after a press. Keep the sim tab visible. A hidden tab throttles timers, and the bridge treats
that as a lost heartbeat and stops the motors.

| Input | IK mode (default) | Joint mode |
|---|---|---|
| Left stick | Move the target across the floor, relative to the camera | J1 ↔, J2 ↕ |
| Right stick | ↕ raise/lower the target, ↔ orbit the camera | J3 ↕ |
| LT / RT | Precision (¼ speed) / full speed | same |
| A · X · Y · LB | IK ↔ joint · ready pose · flip elbow · figure-8 demo | |
| **RB (hold)** | **Deadman: armed motors follow the sim, and calibration/verify move, only while held** | |
| **B** | **STOP: all motors to Stop, disarm (torque off)** | |
| Menu ☰ | Check · verify · arm, or confirm a calibration/verify step | |
| View ⧉ / D-pad ↕ | Reset camera / speed ± | |

Keyboard equivalents: WASD, R/F, Q/E, Shift, Space (deadman), M, H, G, P, C, and Esc (STOP).

## The model (`finalassembly.zip`)

`tools/import_urdf.py` reads the Onshape URDF and glTF meshes, then writes the per-body meshes and
`web/meshes/manifest.json`. Re-run it after any CAD change:

```bash
pip install -r tools/requirements.txt
python3 tools/import_urdf.py ~/Downloads/finalassembly.zip
```

- **Joints:** J1 yaw = `revolute_2`, J2 shoulder = `revolute_1`, J3 elbow = `revolute_3`. Every
  angle in the sim, config, bridge and calibration file is the **URDF joint angle**, so it matches
  RViz.
- **Up direction:** CAD −Z is up. The world origin is on the J1 axis at the lowest point of the base,
  which is the table.
- **Renumbering:** Onshape renumbers links and `Part_1_N` mesh files on every export. The importer
  therefore recognises parts by **shape**, finds the tool point (the forearm's end clamps) by
  **geometry**, and drops the duplicated J1 motor body by **position**.
- **`revolute_4` is welded at its CAD angle.** It is a free revolute between the J3 hub and the
  forearm clamp, with no actuator. Make it a fastened mate in Onshape.
- **Closed-form IK:** h = 137.2 mm, L2 = 350.5 mm, L3 = 268.9 mm (to the end-clamp centre), tool
  offset −58.1 mm sideways. It is checked against the URDF chain to 0.01 mm.
- **Rendering and collisions:** rendering uses the URDF chain itself. The IK rejects poses past joint
  limits, into the table, or with the arm into the turret or base, using CAD-derived sphere sets.
- **Inertials:** the URDF's are placeholders (about 0.1 g per part), so the gravity model uses
  `geometry.mass_kg` in the config.

## Joint limits

| Joint | Limits (URDF °) | Why |
|---|---|---|
| J1 yaw | −90 … +90 | **180° total for wire management.** There is no mechanical stop, so this is a soft limit. It depends on the J1 calibration (aligned by eye). |
| J2 shoulder | −84.9 … +47.5, replaced by the measured contacts − 4° | Back: a tube clamp meets a nut on the turret at −88.94°. Front: the elbow motor cage touches the table at +51.54°. With the screw interference removed it now has **140° of travel**, up from 61°. |
| J3 elbow | 60 … 390 (360 is the CAD pose) | It has no stop, so the window is placed so the 30° it can't use is "forearm folded back along the upper arm". |

## Calibration (absolute output encoders)

Each joint stores one reference, and each encoder reading maps to the URDF angle with

```
q = centre + wrap±180°(q_ref + direction · (enc − enc_ref) − centre)      centre = middle of the joint's limits
```

This is unambiguous for any travel under 360°, whether the IM6014 reports a single-turn or multi-turn
absolute angle at power-up. The result is saved once in `config/calibration.json`.

> **Power-up semantics are not measured yet.** The workbench manual says the multi-turn count is not
> retained through power loss. Its one hardware datum shows a multi-turn accumulator, but nothing has
> been recorded across a power cycle. So a calibration **loaded from file must be verified once per
> connection** before the arm can be armed (`safety.verify_after_connect`):
> 1. **Verify** makes J2 touch its mechanical back stop.
> 2. The measured angle must match the CAD within 3°.
> 3. You confirm that the green motor-feedback ghost lies on the real arm (J1, J3).
>
> A mismatch stops the motors and asks for recalibration. To settle the question on the bench:
> mark one output, power-cycle it, and back-drive it while unpowered. If every reading still matches
> the mark (mod 360°), the encoder is absolute at power-up.

### How the reference angle is found

The importer sweeps each joint through the CAD and records the first angle at which anything touches:
the arm's own parts, the turret, the base, or the table plane. At a physical contact the joint angle
equals that predicted angle. Calibration seeks the contact on each side:

- **enc_ref** = midpoint of the two measured encoder angles.
- **q_ref** = midpoint of the two predicted angles.

The measured span must match the CAD span (default ±4°). A mismatch stops the motors, disarms them,
and saves nothing.

**Contact detection:** the target ramps at 10°/s. A contact is declared when the measured speed stays
under 30% of the commanded speed for 0.2 s while the target leads by at least 2°. That detection is
gravity-proof, and the joint then backs off 4°.

| Step | Joint | Contacts (CAD) | Method |
|---|---|---|---|
| 1 | J1 yaw | none | Align the turret to 0° by eye, then confirm (☰) |
| 2 | J2 shoulder | −88.94° tube clamp on a turret nut · +51.54° elbow cage on the table | Seek both; reference −18.7° |
| 3 | J3 elbow | With J2 parked at 43.5°, the end clamps touch the table at 26.64° and 231.53° | Seek both; reference 129.09° |

**Start pose** (*Calibration start pose* in the sim): J2 ≈ −19°, J3 ≈ 148°, forearm folded up. From
there J2's sweep is stopped only by its own parts. **Preview in sim** animates the whole plan.

**To calibrate:** Connect (Acknowledge a startup timeout), pose the arm, tick *supported*, then
**Calibrate**. Align J1 and press ☰, then **hold RB** while J2 and J3 seek. When it finishes it saves
and holds; release RB and hold it again to drive. **Fallback:** put the arm at the CAD pose and click
**Set reference = URDF zero**.

## What the IM6014 workbench adds

The workbench's own README describes the custom six-joint IM6014 arm it was built for. From it the bridge uses:

- **Transaction (`im6014_serial.exchange`):**
  - Skips corrupt frames and replies from the wrong motor ID, and keeps reading for the requested
    motor until the deadline. im6014Development faulted on the first bad frame, which matters with
    three motors sharing a bus.
  - Bounds every blocking call to that deadline.
  - Raises typed errors carrying rejected and CRC-rejected frame counts. These feed the panel's
    *CRC / rejected / timeouts* counters, and faults name the joint and ID.
- **Packet capture:** the last 256 transactions (request, reply, outcome) feed
  **Export diagnostics** (`/api/diagnostics`), together with state and events.
- **`JointMap`:** the joint ↔ actuator-output conversion, with overflow and non-finite checks.
- **Port lease (Linux):** `open_managed_port` holds a per-device lease and checks the opened device.
  Set `bus.expected_serial` / `expected_channel` to reject the wrong adapter. On macOS the bridge opens
  pyserial directly, because the lease refuses to run when `/tmp` is a symlink.

Its guarded controller core (`JointCore`) is still changing upstream, so instead of importing it, the
bridge mirrors its policies:

- **Explicit fault acknowledgment:** Check alone can't clear a fault.
- **Acceleration-limited trajectory:** a discrete trapezoid per joint (`max_speed_dps`,
  `max_accel_dps2`). The velocity feed-forward no longer jumps when RB engages.
- **Predicted-effort guard:** the bridge won't step further into a commanded torque (feed-forward +
  Kp/Kd terms) above `safety.effort_limit_nm` (3.5 Nm, below `healthy()`'s 4 Nm trip). Steps that
  reduce the effort are always allowed, so you can back out.

**Timing:** the demo runs at about 85 Hz. On a real FTDI adapter, expect roughly 15 ms per exchange
at the default 16 ms latency timer, so about 20 Hz for three joints. The panel shows each joint's
round-trip time. On Linux, set `/sys/bus/usb-serial/devices/ttyUSBn/latency_timer` to 1.

## Safety

- **Start-up:** the bridge starts disconnected and disarmed, and sends only Stop/status until armed.
  Every FOC packet sets the firmware watchdog.
- **Trips:** these stop, disarm and latch everything, and recovery needs **Acknowledge**:
  - browser heartbeat older than 0.5 s, or the controlling tab closing
  - motor feedback older than 0.1 s
  - any `healthy()` fault (measured torque over 4 Nm, voltage, temperature, fault bits)
  - a joint more than 5° past its limit
  - a tracking error over 30°
- **Arming:** after any arm, RB must be released and pressed again before the motors follow the sim.
- **Access:** only one tab controls the arm; any tab can STOP. The bridge binds to 127.0.0.1 and
  rejects foreign Host/Origin headers.
- **Demo motors:** all three run on **one shared simulated bus**, like the real wiring, each with an
  absolute encoder (a fixed hidden offset) and the CAD contacts as stops. Demo results go to
  `config/calibration.demo.json`.

> **Stop releases torque — a gravity-loaded arm falls.** Support it whenever you stop. This is not a
> safety-rated controller. Nothing here has been run on physical IM6014 motors, and neither the
> workbench's position mode, multi-motor bus nor new transport has been validated on hardware yet.

Before real hardware:
- **IDs and port:** set the motor IDs (0/1/2) and the port (`--list-ports`; on macOS
  `/dev/cu.usbserial-*`). The workbench is validated on Linux only.
- **Directions:** jog each joint to confirm its `direction`.
- **Table height:** the J2 front contact and both J3 contacts are table touches, so they assume the
  table is flush with the bottom of the base. Otherwise rerun the importer with `--table-offset-mm`.
- **J2 torque:** holding the arm straight out needs about 3.96 Nm (estimated masses). The effort
  guard stops short of the 4 Nm trip. Set real masses before enabling `safety.gravity_ff`.

## Files

```
config/arm.json            joints (URDF names, IDs, limits, gains, accel), masses, calibration + safety settings
config/calibration.json    written by Calibrate (absolute-encoder references); not created until you run it
web/                       three.js sim: kinematics.js (closed-form FK/IK, sphere collisions), main.js, …
web/meshes/                per-body meshes + manifest.json (joints, IK params, CAD stops, calibration plan)
bridge/im6014.py           loads im6014-workbench (or im6014Development)
bridge/arm_bridge.py       HTTP + WebSocket server (stdlib), origin checks, one controlling page, diagnostics
bridge/arm_session.py      serial-owner thread: arming, trajectory, effort guard, calibration + verify
bridge/calibration.py      encoder mapping, contact-seek detector, two-contact solver (pure logic)
bridge/demo_motors.py      shared demo bus: DemoSerial + absolute encoders + CAD stops
tools/import_urdf.py       URDF → meshes, kinematics, CAD contact sweeps, calibration plan
```

```bash
node tests/test_kinematics.mjs     # closed form vs URDF chain, IK round trips, limits, table sliding
python3 tests/test_bridge.py       # arming, deadman, trips, fault acknowledgment, WebSocket rules
python3 tests/test_calibration.py  # mapping math; full demo calibration; reconnect + verify; span mismatch
python3 tests/test_transport.py    # shared bus: foreign/corrupt/missing replies; trajectory; effort guard; verify catches a bad calibration
python3 tests/test_ui.py           # optional: headless Chromium + stubbed Xbox pad: calibrate, drive, stop, reconnect, verify
```

![Calibration running against the demo motors](docs/calibrating.jpg)
