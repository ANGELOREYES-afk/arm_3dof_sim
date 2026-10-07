"""Split the 3-DOF CAD export (one binary STL) into per-link meshes for the simulator.

    python3 tools/split_assembly.py ~/Downloads/mainAssem.stl

Needs numpy + scipy. Writes web/meshes/<link>.<material>.bin (float32 xyz + uint32 triangle indices) and manifest.json.

The STL is a single soup with no part names, so parts are recovered as edge-connected
components and assigned to links by where they sit:

  CAD -Z is up. The brown end plate is the mounting face on the table.
  J1 yaw      = motor with its axis along CAD Z (blue). Its cage is the static base.
  J2 shoulder = motor with its axis along CAD Y (red), carried by the L-bracket that is
                bolted to the J1 output. Its output face (+Y) carries the upper arm.
  J3 elbow    = the third motor, not yet mated in CAD. It is re-posed at the elbow with
                its axis parallel to J2, stator on the upper arm, output facing the forearm.

The upper arm and forearm do not exist in CAD yet; the browser draws them parametrically.

Output frames (mm, Z up): base and turret are in the world frame W, origin on the yaw
axis at floor level, W = diag(1, -1, -1) @ (CAD - origin). The elbow motor is in the J2
frame shifted to the elbow axis, so the browser only places it at z = upper-arm length.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

PLATE_MM = 6.0   # assumed thickness of the parametric upper-arm and forearm plates
OUT = Path(__file__).resolve().parent.parent / 'web' / 'meshes'
R = np.diag([1., -1., -1.])  # CAD -> world rotation (180 deg about X): -Z becomes up


def read_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        tri = np.fromfile(f, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n)
    if len(tri) != n:
        raise SystemExit('Truncated or ASCII STL; export as binary STL')
    return tri['v'].astype(np.float64)


def edge_components(T):
    """Label triangles that share an edge. Parts touching at a point stay separate."""
    q = np.round(T.reshape(-1, 3) * 1000).astype(np.int64)  # weld at 1 micron
    _, inv = np.unique(q, axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    nv = inv.max() + 1
    e = np.sort(np.stack([inv[:, [0, 1]], inv[:, [1, 2]], inv[:, [2, 0]]], 1).reshape(-1, 2), 1)
    key = e[:, 0] * nv + e[:, 1]
    tid = np.repeat(np.arange(len(T)), 3)
    o = np.argsort(key, kind='stable')
    same = key[o][1:] == key[o][:-1]
    a, b = tid[o][:-1][same], tid[o][1:][same]
    g = coo_matrix((np.ones(len(a)), (a, b)), shape=(len(T), len(T)))
    return connected_components(g, directed=False)[1]


def motor_axis(lo, hi):
    """Motor bodies are ~65 mm round; the axis is the extent that differs from the other two."""
    ext = hi - lo
    return int(np.argmax(np.abs(ext - np.median(ext))))


def output_end_is_max(motor, comps):
    """Each motor has three small near-axis parts ~9.4 mm inside its output face; its
    back-cover screws sit at r = 30 mm on the other end."""
    a = motor['axis']
    radial = [i for i in range(3) if i != a]
    near = [c for c in comps if len(c['tris']) < 500
            and np.all(c['c'] > motor['lo']) and np.all(c['c'] < motor['hi'])
            and np.linalg.norm((c['c'] - motor['c'])[radial]) < 15]
    if not near:
        raise SystemExit('Could not find the near-axis parts used to tell the output end')
    conn = np.mean([c['c'][a] for c in near])
    return abs(conn - motor['hi'][a]) < abs(conn - motor['lo'][a])


def write_bin(name, T):
    q = np.round(T.reshape(-1, 3) * 1000).astype(np.int64)
    _, first, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    pos = T.reshape(-1, 3)[first].astype('<f4')
    idx = inv.reshape(-1).astype('<u4')
    (OUT / f'{name}.bin').write_bytes(pos.tobytes() + idx.tobytes())
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    return dict(file=f'{name}.bin', vertices=len(pos), triangles=len(idx) // 3,
                bbox_min=np.round(lo, 2).tolist(), bbox_max=np.round(hi, 2).tolist())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('stl')
    args = ap.parse_args()
    T = read_stl(args.stl)
    lab = edge_components(T)
    comps = []
    for c in range(lab.max() + 1):
        m = np.where(lab == c)[0]
        p = T[m].reshape(-1, 3)
        comps.append(dict(tris=m, lo=p.min(0), hi=p.max(0), c=(p.min(0) + p.max(0)) / 2))

    motors = [c for c in comps if len(c['tris']) > 50000]
    if len(motors) != 3:
        raise SystemExit(f'Expected 3 motor bodies (>50k triangles), found {len(motors)}')
    for mo in motors:
        mo['axis'] = motor_axis(mo['lo'], mo['hi'])
    yaw = [m for m in motors if m['axis'] == 2]
    pitch = [m for m in motors if m['axis'] != 2]
    if len(yaw) != 1:
        raise SystemExit('Expected exactly one motor with its axis along CAD Z (the base yaw)')
    yaw = yaw[0]
    # The shoulder is the pitch motor nearest the base; the other one is the unmated elbow.
    pitch.sort(key=lambda m: np.linalg.norm(m['c'] - yaw['c']))
    shoulder, elbow = pitch
    if shoulder['axis'] != 1:
        raise SystemExit('Expected the shoulder motor axis along CAD Y')

    origin = np.array([yaw['c'][0], yaw['c'][1], 0.])
    base_cut = yaw['lo'][2] - 1   # J1 output face (most -Z point of the yaw motor), with 1 mm slack
    elbow_box = (elbow['lo'] - 5, elbow['hi'] + 5)
    in_elbow = [c for c in comps if np.all(c['c'] > elbow_box[0]) and np.all(c['c'] < elbow_box[1])]
    placed = [c for c in comps if len(c['tris']) > 3000 and not any(c is e for e in in_elbow)]
    envelope_x = (min(c['lo'][0] for c in placed) - 1, max(c['hi'][0] for c in placed) + 1)

    # The shoulder output flange: coaxial with the J2 motor, on its output end. It turns with J2.
    a = shoulder['axis']
    sh_out_hi = output_end_is_max(shoulder, comps)
    sh_end = shoulder['hi'][a] if sh_out_hi else shoulder['lo'][a]
    flange = [c for c in comps if c is not shoulder and 1000 < len(c['tris']) < 50000
              and np.all(np.abs(np.delete(c['c'] - shoulder['c'], a)) < 2)
              and c['lo'][a] - 1 < sh_end < c['hi'][a] + 1]
    if len(flange) != 1 or not sh_out_hi:
        raise SystemExit('Expected one output flange on the +Y (output) end of the shoulder motor')
    flange = flange[0]

    groups = dict(base=[], turret=[], shoulder_flange=[flange], elbow_motor=[])
    dropped = []
    for c in comps:
        if c is flange:
            continue
        if any(c is e for e in in_elbow):
            groups['elbow_motor'].append(c)
        elif not envelope_x[0] <= c['c'][0] <= envelope_x[1]:
            dropped.append(c)  # loose hardware floating outside every bracket
        elif c['c'][2] > base_cut:
            groups['base'].append(c)
        else:
            groups['turret'].append(c)

    ground_z = max(c['hi'][2] for c in groups['base'])  # lowest point once -Z is up
    origin[2] = ground_z

    def world(P):
        return (P - origin) @ R.T

    shoulder_height = ground_z - shoulder['c'][2]
    shoulder_face_y = -(flange['hi'][1] - origin[1])  # J2 flange face, world y
    motor_len = float(elbow['hi'][elbow['axis']] - elbow['lo'][elbow['axis']])
    motor_dia = float(np.median(elbow['hi'] - elbow['lo']))
    upper_y = (shoulder_face_y - PLATE_MM, shoulder_face_y)
    elbow_back_y = upper_y[0]
    elbow_out_y = elbow_back_y - motor_len
    fore_y = (elbow_out_y - PLATE_MM, elbow_out_y)

    # Elbow motor: map its axis onto -y so the output faces the forearm.
    a = elbow['axis']
    out_at_min = not output_end_is_max(elbow, comps)
    others = [i for i in range(3) if i != a]
    M = np.zeros((3, 3))
    M[1, a] = 1. if out_at_min else -1.          # axis -> y, output end toward -y
    M[0, others[0]] = 1.
    M[2, others[1]] = 1.
    if np.linalg.det(M) < 0:  # keep it a proper rotation
        M[2, others[1]] = -1.
    out_face = elbow['lo'][a] if out_at_min else elbow['hi'][a]

    def elbow_local(P):
        p = (P - elbow['c']) @ M.T
        p[:, 1] += (elbow['c'][a] - out_face) * M[1, a] + elbow_out_y
        return p

    # IK keep-out in the arm plane (J1 frame), as [x0, x1, z0, z1] boxes per slab. The turret
    # yaws with the arm, so its geometry inside each slab becomes a skyline of max heights.
    # The base does not yaw; it is treated as round (its max radius) up to its top.
    turret_P = world(T[np.concatenate([c['tris'] for c in groups['turret']])].reshape(-1, 3))
    base_P = world(T[np.concatenate([c['tris'] for c in groups['base']])].reshape(-1, 3))
    base_r = float(np.hypot(base_P[:, 0], base_P[:, 1]).max())
    base_box = [-base_r, base_r, 0., float(base_P[:, 2].max())]

    def skyline(y0, y1, bins=16):
        S = turret_P[(turret_P[:, 1] >= y0 - 1) & (turret_P[:, 1] <= y1 + 1)]
        edges = np.linspace(S[:, 0].min(), S[:, 0].max(), bins + 1)
        boxes = [base_box]
        for a, b in zip(edges[:-1], edges[1:]):
            z = S[(S[:, 0] >= a - .5) & (S[:, 0] <= b + .5), 2]
            if len(z):
                boxes.append([float(a), float(b), 0., float(z.max())])
        return [[round(v, 1) for v in box] for box in boxes]

    def material(c):
        if any(np.all(c['lo'] >= m['lo'] - 1) and np.all(c['hi'] <= m['hi'] + 1) for m in motors):
            return 'motor'      # motor bodies and the parts inside their envelope
        return 'hardware' if len(c['tris']) < 2500 else 'frame'   # rods/screws vs brackets/plates

    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob('*.bin'):
        old.unlink()
    parts = {}
    for name, cs in groups.items():
        parts[name] = {}
        for mat in ('motor', 'frame', 'hardware'):
            sel = [c for c in cs if material(c) == mat]
            if not sel:
                continue
            P = T[np.concatenate([c['tris'] for c in sel])].reshape(-1, 3)
            if name == 'elbow_motor':
                P = elbow_local(P)
            elif name == 'shoulder_flange':
                P = world(P) - [0, 0, shoulder_height]  # J2 frame: origin on the shoulder axis
            else:
                P = world(P)
            parts[name][mat] = write_bin(f'{name}.{mat}', P.reshape(-1, 3, 3))
            parts[name][mat]['components'] = len(sel)

    manifest = dict(
        source=dict(file=Path(args.stl).name, triangles=len(T),
                    sha256=hashlib.sha256(Path(args.stl).read_bytes()).hexdigest()),
        units='mm', up='+z', frames=dict(base='world', turret='J1 (world at q1=0)',
                                         shoulder_flange='J2, origin on the shoulder axis',
                                         elbow_motor='J2, origin on the elbow axis'),
        parts=parts,
        dropped=[dict(triangles=len(c['tris']), cad_center=np.round(c['c'], 1).tolist()) for c in dropped],
        geometry=dict(
            shoulder_height_mm=round(float(shoulder_height), 2),
            shoulder_face_y_mm=round(float(shoulder_face_y), 2),
            plate_mm=PLATE_MM,
            upper_arm_y_mm=[round(float(v), 2) for v in upper_y],
            elbow_motor_back_y_mm=round(float(elbow_back_y), 2),
            elbow_output_y_mm=round(float(elbow_out_y), 2),
            forearm_y_mm=[round(float(v), 2) for v in fore_y],
            tool_offset_y_mm=round(float(sum(fore_y) / 2), 2),
            motor_length_mm=round(motor_len, 2),
            motor_diameter_mm=round(motor_dia, 2),
            # The forearm slab clears the turret and base, so it only meets the floor.
            keepout_upper_arm_mm=skyline(*upper_y),
            keepout_elbow_motor_mm=skyline(elbow_out_y, elbow_back_y),
        ))
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    print(f"{len(T)} triangles -> {len(comps)} parts")
    for name, mats in parts.items():
        for mat, p in mats.items():
            print(f"  {name + '.' + mat:22s} {p['triangles']:7d} tris  {p['components']:3d} parts  "
                  f"bbox {p['bbox_min']} .. {p['bbox_max']}")
    print(f"  dropped      {len(dropped)} loose part(s): {[d['cad_center'] for d in manifest['dropped']]}")
    print(json.dumps(manifest['geometry'], indent=2))


if __name__ == '__main__':
    main()
