"""Import the Onshape URDF export into the simulator, and plan the joint calibration from the CAD.

    pip install -r tools/requirements.txt
    python3 tools/import_urdf.py ~/Downloads/mainAssem.zip

Writes web/meshes/<body>.<material>.bin (float32 xyz + uint32 triangle indices, mm, world frame at
the URDF zero pose) and web/meshes/manifest.json with:

  joints        world origin + axis of J1..J3 at the zero pose (product-of-exponentials model)
  analytic      the closed-form IK parameters, checked against the URDF chain
  collision     sphere sets per body for the browser's IK collision checks
  hard_stops    CAD-predicted contact angles, swept per joint against the rest of the arm and table
  calibration   the per-joint reference plan the motor bridge executes

World frame: CAD -Z is up, so world = diag(1, -1, -1) @ (CAD - origin); origin on the J1 axis at the
lowest point of the base (the table plane, z = 0). Joint angles everywhere are the URDF angles.

Export quirks handled for this assembly:
  --weld revolute_4      Onshape left a free revolute between the J3 hub and the forearm clamp; a
                         3-actuator arm cannot drive it, so it is held at its CAD angle.
  duplicate motor        the J1 motor is exported twice in the same place; the copy on the moving
                         side is dropped automatically (matched by position, not link name).
Onshape renumbers links and Part_1_N mesh files on every export, so parts are recognised by shape
and the tool point by geometry; use --drop / --tool-links only to override.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from scipy.spatial import cKDTree

try:
    import fast_simplification
except ImportError:
    fast_simplification = None

OUT = Path(__file__).resolve().parent.parent / 'web' / 'meshes'
R_UP = np.diag([1., -1., -1.])  # CAD -Z up -> world +Z up
CONTACT_MM = 0.6                # surfaces closer than this count as touching
SAMPLE_MM = 0.8                 # surface sampling pitch for contact sweeps


def classify(file, P, I):
    """(material, description) from the part's shape. Onshape renumbers Part_1_N files between
    exports, so names cannot be trusted; sizes (mm, sorted extents) and triangle counts can."""
    a, b, c = sorted(np.ptp(P, 0) * 1000)
    low = file.lower()
    if abs(c - 65) < 1.5 and abs(b - 65) < 1.5 and abs(a - 62.3) < 2:  # IM6014: 65 mm dia x 62.3
        return 'motor', 'IM6014'
    if 'screw' in low:
        return 'hardware', 'screw'
    if 'nut' in low:
        return 'hardware', 'nut'
    if c > 150 and b < 25:
        return 'tube', 'link tube'
    if a < 11 and abs(b - 54.6) < 2 and abs(c - 54.6) < 2:
        return 'hub', 'output flange'
    if c < 40:
        return 'hardware', 'standoff'
    if abs(c - 71) < 2 and abs(b - 71) < 2:
        return 'link', 'tube clamp'
    if c > 120:
        return 'frame', 'turret L-bracket'
    if abs(c - 90) < 2 and a < 16:
        return 'link', 'elbow plate'
    if abs(c - 93.7) < 1 and a < 11:
        return 'frame', 'motor plate'
    if abs(c - 93.7) < 1:
        return 'frame', 'motor bracket'
    return 'frame', file.replace('.gltf', '')


# ---------------------------------------------------------------- URDF + glTF
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot(axis, t):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def pose(xyz, r):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = r, xyz
    return M


def nums(s):
    return [float(x) for x in s.split()]


class Robot:
    def __init__(self, urdf, mesh_dir):
        root = ET.parse(urdf).getroot()
        self.mesh_dir = mesh_dir
        self.links = {l.get('name'): l for l in root.findall('link')}
        self.joints = {j.get('name'): j for j in root.findall('joint')}
        self.parent_joint = {j.find('child').get('link'): j for j in self.joints.values()}
        self.kids = {}
        for j in self.joints.values():
            self.kids.setdefault(j.find('parent').get('link'), []).append(j)
        self.root = next(n for n in self.links if n not in self.parent_joint)
        self._gltf = {}

    def origin(self, j):
        o = j.find('origin')
        return pose(nums(o.get('xyz', '0 0 0')), rpy(*nums(o.get('rpy', '0 0 0'))))

    def axis(self, j):
        a = j.find('axis')
        return np.array(nums(a.get('xyz')) if a is not None else [1., 0, 0])

    def fk(self):
        """Link frames at the zero pose (URDF units, CAD frame)."""
        out = {self.root: np.eye(4)}
        stack = [self.root]
        while stack:
            n = stack.pop()
            for j in self.kids.get(n, []):
                c = j.find('child').get('link')
                out[c] = out[n] @ self.origin(j)
                stack.append(c)
        return out

    def body(self, link, actuated):
        """The actuated joint that moves this link (None = base)."""
        while link in self.parent_joint:
            j = self.parent_joint[link]
            if j.get('name') in actuated:
                return j.get('name')
            link = j.find('parent').get('link')
        return None

    def visuals(self, link):
        for v in self.links[link].findall('visual'):
            m = v.find('geometry/mesh')
            if m is None:
                continue
            o = v.find('origin')
            M = pose(nums(o.get('xyz', '0 0 0')), rpy(*nums(o.get('rpy', '0 0 0')))) if o is not None else np.eye(4)
            yield m.get('filename').split('/')[-1], M

    def mesh(self, name, reduce_over=40000, keep=0.12):
        """[(points, triangles)] in mesh units; very dense meshes are decimated."""
        if name in self._gltf:
            return self._gltf[name]
        g = json.loads((self.mesh_dir / name).read_text())
        bufs = [base64.b64decode(b['uri'].split(',', 1)[1]) for b in g['buffers']]

        def acc(i):
            a = g['accessors'][i]
            bv = g['bufferViews'][a['bufferView']]
            n = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
            arr = np.frombuffer(bufs[bv['buffer']], {5126: '<f4', 5125: '<u4', 5123: '<u2', 5121: 'u1'}[a['componentType']],
                                count=a['count'] * n, offset=bv.get('byteOffset', 0) + a.get('byteOffset', 0))
            return arr.reshape(-1, n) if n > 1 else arr

        parts = []

        def walk(i, M):
            nd = g['nodes'][i]
            L = np.eye(4)
            if 'matrix' in nd:
                L = np.array(nd['matrix']).reshape(4, 4).T
            else:
                if 'rotation' in nd:
                    x, y, z, w = nd['rotation']
                    L[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
                if 'scale' in nd:
                    L[:3, :3] = L[:3, :3] @ np.diag(nd['scale'])
                if 'translation' in nd:
                    L[:3, 3] = nd['translation']
            M = M @ L
            for p in g['meshes'][nd['mesh']]['primitives'] if 'mesh' in nd else []:
                P = acc(p['attributes']['POSITION']).astype(float) @ M[:3, :3].T + M[:3, 3]
                I = acc(p['indices']).reshape(-1, 3).astype(np.int64) if 'indices' in p else np.arange(len(P)).reshape(-1, 3)
                parts.append((P, I))
            for c in nd.get('children', []):
                walk(c, M)

        for i in g['scenes'][g.get('scene', 0)]['nodes']:
            walk(i, np.eye(4))
        P = np.concatenate([p for p, _ in parts])
        I = np.concatenate([i + off for (_, i), off in zip(parts, np.cumsum([0] + [len(p) for p, _ in parts[:-1]]))])
        if len(I) > reduce_over and fast_simplification:
            P, I = fast_simplification.simplify(P.astype(np.float32), I.astype(np.int32), target_reduction=1 - keep)
            P, I = np.asarray(P, float), np.asarray(I, np.int64)
        self._gltf[name] = [(P, I)]
        return self._gltf[name]


# ---------------------------------------------------------------- geometry helpers
def sample_surface(tris, pitch, rng):
    A = np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1) / 2
    ti = np.repeat(np.arange(len(tris)), rng.poisson(A / pitch ** 2))
    r = rng.random((len(ti), 2))
    m = r.sum(1) > 1
    r[m] = 1 - r[m]
    pts = tris[ti, 0] + r[:, :1] * (tris[ti, 1] - tris[ti, 0]) + r[:, 1:] * (tris[ti, 2] - tris[ti, 0])
    pts = np.concatenate([pts, tris.reshape(-1, 3)])
    _, keep = np.unique(np.floor(pts / (pitch * 1.2)).astype(np.int64), axis=0, return_index=True)
    return pts[keep]


def spheres(points, cell=22.):
    """Cover a point cloud with spheres: one per occupied voxel, radius to its farthest point."""
    key = np.floor(points / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    out = []
    for k in range(inv.max() + 1):
        p = points[inv.ravel() == k]
        c = p.mean(0)
        out.append([*np.round(c, 1), round(float(np.linalg.norm(p - c, axis=1).max()) + 1.0, 1)])
    return out


class Chain:
    """Product of exponentials on world-frame joint axes at the zero pose."""

    def __init__(self, joints):
        self.o = [np.array(j['origin']) for j in joints]
        self.a = [np.array(j['axis']) for j in joints]

    def body_transform(self, k, q):
        """Rotation R and translation t moving body k (1-based) from the zero pose to q."""
        R, t = np.eye(3), np.zeros(3)
        for i in range(k):
            Ri = rot(self.a[i], q[i])
            ti = self.o[i] - Ri @ self.o[i]
            R, t = R @ Ri, R @ ti + t
        return R, t

    def point(self, k, q, p):
        R, t = self.body_transform(k, q)
        return R @ p + t


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', help='Onshape URDF export: the .zip or the package folder')
    ap.add_argument('--joints', default='revolute_2,revolute_1,revolute_3', help='J1,J2,J3 URDF joint names')
    ap.add_argument('--names', default='J1 yaw,J2 shoulder,J3 elbow')
    ap.add_argument('--weld', default='revolute_4')
    ap.add_argument('--drop', default='', help='links whose visuals are not part of the arm '
                    '(a duplicate motor body stacked on another is dropped automatically)')
    ap.add_argument('--tool-links', default='', help='tool point = centre of these links '
                    '(default: the forearm tube clamps farthest from the J3 axis)')
    ap.add_argument('--table-offset-mm', type=float, default=0., help='table plane above the lowest base point')
    args = ap.parse_args()
    t_start = time.time()

    src = Path(args.source).expanduser()
    tmp = None
    if src.suffix == '.zip':
        tmp = tempfile.TemporaryDirectory()
        zipfile.ZipFile(src).extractall(tmp.name)
        pkg = Path(tmp.name)
    else:
        pkg = src
    urdf = next(pkg.rglob('*.urdf'))
    robot = Robot(urdf, urdf.parent.parent / 'meshes')
    actuated = args.joints.split(',')
    names = args.names.split(',')
    drop = set(args.drop.split(',')) if args.drop else set()
    welded = [w for w in args.weld.split(',') if w]
    for w in welded:
        if robot.joints[w].get('type') == 'fixed':
            raise SystemExit(f'{w} is already fixed')
    free = [n for n, j in robot.joints.items() if j.get('type') != 'fixed' and n not in actuated + welded]
    if free:
        raise SystemExit(f'Unhandled movable joints {free}: list them in --joints or --weld')

    F = robot.fk()
    bodies = ['base'] + actuated
    body_key = {None: 'base', **{j: f'j{i + 1}' for i, j in enumerate(actuated)}}

    # ---- triangles per link, CAD mm
    rng = np.random.default_rng(0)
    link_tris, link_mat, link_desc = {}, {}, {}
    for n in robot.links:
        if n in drop:
            continue
        for f, V in robot.visuals(n):
            for P, I in robot.mesh(f):
                M = F[n] @ V
                link_tris.setdefault(n, []).append(((P @ M[:3, :3].T + M[:3, 3]) * 1000)[I])
                link_mat[n], link_desc[n] = classify(f, P, I)
    link_tris = {n: np.concatenate(t) for n, t in link_tris.items()}
    link_body = {n: body_key[robot.body(n, actuated)] for n in link_tris}
    # Onshape sometimes exports a motor twice (stator and rotor sides) in the same place. Keep the
    # copy on the proximal side of the joint; link names change between exports, so match by position.
    motors = [n for n in link_tris if link_mat[n] == 'motor']
    rank = {b: i for i, b in enumerate(body_key.values())}
    mid = {n: (link_tris[n].reshape(-1, 3).min(0) + link_tris[n].reshape(-1, 3).max(0)) / 2 for n in motors}
    for i, a in enumerate(motors):
        for b in motors[i + 1:]:
            if a in link_tris and b in link_tris and np.linalg.norm(mid[a] - mid[b]) < 3:
                extra = max((a, b), key=lambda n: rank[link_body[n]])
                print(f'Dropping duplicate motor body {extra} (stacked on {a if extra == b else b})', flush=True)
                drop.add(extra)
                del link_tris[extra], link_body[extra]

    # ---- world frame
    j_cad = []
    for jn in actuated:
        j = robot.joints[jn]
        M = F[j.find('parent').get('link')] @ robot.origin(j)
        a = M[:3, :3] @ robot.axis(j)
        j_cad.append((M[:3, 3] * 1000, a / np.linalg.norm(a)))
    base_pts = np.concatenate([t.reshape(-1, 3) for n, t in link_tris.items() if link_body[n] == 'base'])
    up_cad = R_UP.T @ np.array([0, 0, 1.])
    origin = j_cad[0][0] - (j_cad[0][0] @ up_cad - (base_pts @ up_cad).min()) * up_cad

    def world(P):
        return (np.asarray(P) - origin) @ R_UP.T

    joints = [dict(name=nm, urdf=jn, origin=np.round(world(o), 4).tolist(), axis=np.round(a @ R_UP.T, 6).tolist())
              for nm, jn, (o, a) in zip(names, actuated, j_cad)]
    chain = Chain(joints)
    a1 = np.array(joints[0]['axis'])
    if abs(abs(a1[2]) - 1) > 1e-4 or np.hypot(*joints[0]['origin'][:2]) > 1e-3:
        raise SystemExit('J1 must be the vertical axis through the base origin')

    world_tris = {n: world(t.reshape(-1, 3)).reshape(-1, 3, 3) for n, t in link_tris.items()}
    def centre(n):
        P = world_tris[n].reshape(-1, 3)
        return (P.min(0) + P.max(0)) / 2

    if args.tool_links:
        tool_links = args.tool_links.split(',')
    else:  # link names are renumbered between exports, so find the forearm's end clamps by geometry
        o3, a3w = np.array(joints[2]['origin']), np.array(joints[2]['axis'])
        clamps = [n for n in world_tris if link_body[n] == 'j3' and link_desc[n] == 'tube clamp']
        reach = {n: np.linalg.norm(np.cross(centre(n) - o3, a3w)) for n in clamps}
        far = max(reach.values())
        tool_links = sorted(n for n, r in reach.items() if r > far - 5)
    tool = np.mean([centre(n) for n in tool_links], 0)
    print(f'Tool point: centre of {tool_links}', flush=True)

    # ---- closed-form IK parameters (see web/js/kinematics.js) and a check against the chain
    z = np.array([0, 0, 1.])
    s1 = 1. if a1[2] > 0 else -1.
    a2 = np.array(joints[1]['axis'])
    a3 = np.array(joints[2]['axis'])
    if abs(a2 @ z) > 1e-4 or abs(abs(a2 @ a3) - 1) > 1e-4:
        raise SystemExit('Expected J2 horizontal and J3 parallel to J2')
    o2, o3 = np.array(joints[1]['origin']), np.array(joints[2]['origin'])
    S = np.array([0, 0, o2[2]])  # where the J2 axis meets the J1 axis (checked below)
    if np.linalg.norm(np.cross(o2 - S, a2)) > 0.5:
        raise SystemExit('J2 axis does not intersect J1; the closed-form IK needs it to')
    x_arm = np.cross(a2, z)
    upper = (o3 - S) - ((o3 - S) @ a2) * a2
    fore = (tool - o3) - ((tool - o3) @ a2) * a2
    th2 = np.arctan2(upper @ x_arm, upper @ z)
    phi = np.arctan2(fore @ x_arm, fore @ z)
    s3 = 1. if a3 @ a2 > 0 else -1.
    analytic = dict(h=float(S[2]), L2=float(np.linalg.norm(upper)), L3=float(np.linalg.norm(fore)),
                    d=float((tool - S) @ a2), sign=[s1, 1., s3],
                    offset=[float(np.arctan2(x_arm[1], x_arm[0])), float(th2), float(phi - th2)])

    def fk_closed(q):
        qi = [analytic['sign'][i] * q[i] + analytic['offset'][i] for i in range(3)]
        r = analytic['L2'] * np.sin(qi[1]) + analytic['L3'] * np.sin(qi[1] + qi[2])
        zz = analytic['h'] + analytic['L2'] * np.cos(qi[1]) + analytic['L3'] * np.cos(qi[1] + qi[2])
        c, s = np.cos(qi[0]), np.sin(qi[0])
        return np.array([r * c - analytic['d'] * s, r * s + analytic['d'] * c, zz])

    err = max(np.linalg.norm(fk_closed(q) - chain.point(3, q, tool)) for q in rng.uniform(-np.pi, np.pi, (500, 3)))
    if err > 0.5:  # export rounding leaves J2 ~0.05 mm off the J1 axis; anything larger is a model error
        raise SystemExit(f'Closed-form FK disagrees with the URDF chain by {err} mm')
    analytic['max_fk_error_mm'] = float(err)

    # ---- render meshes
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob('*.bin'):
        old.unlink()
    parts = {}
    for b in body_key.values():
        parts[b] = {}
        for mat in sorted({link_mat[n] for n in world_tris if link_body[n] == b}):
            T = np.concatenate([world_tris[n] for n in world_tris if link_body[n] == b and link_mat[n] == mat])
            q = np.round(T.reshape(-1, 3) * 1000).astype(np.int64)
            _, first, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
            pos = T.reshape(-1, 3)[first].astype('<f4')
            (OUT / f'{b}.{mat}.bin').write_bytes(pos.tobytes() + inv.reshape(-1).astype('<u4').tobytes())
            parts[b][mat] = dict(file=f'{b}.{mat}.bin', vertices=len(pos), triangles=len(T))

    # ---- contact model: dense surface points per link, world frame at the zero pose
    pts = {n: sample_surface(t, SAMPLE_MM, rng) for n, t in world_tris.items()}
    trees = {n: cKDTree(p) for n, p in pts.items()}
    order = {b: i for i, b in enumerate(body_key.values())}
    table_z = float(args.table_offset_mm)

    def moved(n, q):
        k = order[link_body[n]]
        if k == 0:
            return pts[n]
        R, t = chain.body_transform(k, q)
        return pts[n] @ R.T + t

    # Links touching at the zero pose across bodies are joint interfaces (flange on motor face, etc.).
    interface = set()
    box = {n: (p.min(0) - 1, p.max(0) + 1) for n, p in pts.items()}
    for m in pts:
        for s in pts:
            if order[link_body[m]] > order[link_body[s]] and np.all(box[m][0] <= box[s][1]) and np.all(box[s][0] <= box[m][1]):
                if trees[s].query(pts[m], distance_upper_bound=1.0)[0].min() < 1.0:
                    interface.add((m, s))

    cache = {}

    def static_side(k, q):
        """One tree over every link proximal to joint k, posed at q (fixed during a sweep of k)."""
        key = (k,) + tuple(np.round(np.delete(np.asarray(q, float), k - 1), 9))
        if key not in cache:
            st = [n for n in pts if order[link_body[n]] < k]
            P = np.concatenate([moved(s, q) for s in st])
            cache.clear()
            cache[key] = (cKDTree(P), np.repeat(np.arange(len(st)), [len(pts[s]) for s in st]), st,
                          P.min(0) - CONTACT_MM, P.max(0) + CONTACT_MM)
        return cache[key]

    def contacts(k, q, intrinsic=False):
        """What body k (and, unless intrinsic, its distal bodies and the table) touches at q."""
        tree, lab, st, lo, hi = static_side(k, q)
        hits = []
        for m in (n for n in pts if (order[link_body[n]] == k if intrinsic else order[link_body[n]] >= k)):
            P = moved(m, q)
            if not intrinsic and P[:, 2].min() < table_z + CONTACT_MM:
                hits.append((m, 'table'))
            P = P[np.all((P >= lo) & (P <= hi), axis=1)]
            if not len(P):
                continue
            d, i = tree.query(P, k=6, distance_upper_bound=CONTACT_MM)
            for x in np.unique(lab[i[d < CONTACT_MM]]):
                if (m, st[x]) not in interface:
                    hits.append((m, st[x]))
        return hits

    def first_contact(k, q0, sign, intrinsic, span=360., step=3.):
        q = list(q0)
        prev = None
        for deg in np.arange(step, span + step, step):
            q[k - 1] = q0[k - 1] + sign * np.radians(deg)
            if contacts(k, q, intrinsic):
                lo, hi = (prev or 0.), deg
                while hi - lo > 0.05:
                    mid = (lo + hi) / 2
                    q[k - 1] = q0[k - 1] + sign * np.radians(mid)
                    lo, hi = (lo, mid) if contacts(k, q, intrinsic) else (mid, hi)
                q[k - 1] = q0[k - 1] + sign * np.radians(hi)
                return float(np.degrees(q[k - 1])), contacts(k, q, intrinsic)
            prev = deg
        return None, []

    def label(n):
        return 'table' if n == 'table' else f'{link_desc[n]} ({n})'

    def stops(k, q0, intrinsic=False):
        out = {}
        for side, sign in (('lo', -1), ('hi', 1)):
            ang, hit = first_contact(k, q0, sign, intrinsic)
            out[side] = None if ang is None else dict(deg=round(ang, 2), contact=sorted({f'{label(m)} vs {label(s)}' for m, s in hit})[:3],
                                                      movers=sorted({link_body[m] for m, _ in hit}))
        return out

    print(f'Contact model: {sum(len(p) for p in pts.values())} surface points, {len(interface)} interface pairs', flush=True)
    q_zero = [0., 0., 0.]
    hard = {}
    for k in (1, 2, 3):  # mechanical stops: the joint's own body against its parents, no table
        hard[names[k - 1]] = stops(k, q_zero, intrinsic=True)
        print(f'  {names[k - 1]} mechanical stops: {hard[names[k - 1]]}', flush=True)

    # ---- calibration plan
    plan = []
    j2_mech = hard[names[1]]
    # J2 first. The forearm is uncalibrated, so pick a hand-placed J3 start pose from which the J2
    # sweep is stopped by J2's own body on both sides (mechanical stop, turret, base or table), at
    # the same angles for any J3 within +-25 deg of it.
    q2_mid = np.radians((j2_mech['lo']['deg'] + j2_mech['hi']['deg']) / 2) if j2_mech['lo'] and j2_mech['hi'] else 0.
    j2_key = body_key[actuated[1]]

    def own(s):
        return s['lo'] and s['hi'] and s['lo']['movers'] == [j2_key] and s['hi']['movers'] == [j2_key]

    def j3_down(q2):
        qi2 = analytic['sign'][1] * q2 + analytic['offset'][1]
        return (np.pi - qi2 - analytic['offset'][2]) / analytic['sign'][2]  # forearm hanging straight down

    start_q3, j2 = None, None
    for cand in [j3_down(q2_mid) + np.radians(d) for d in (180, 150, -150, 120, -120, 90, -90, 60, -60, 30, -30, 0)]:
        ref = stops(2, [0., q2_mid, cand])
        ok = own(ref)
        for dq in (-25, 25):
            if not ok:
                break
            s = stops(2, [0., q2_mid, cand + np.radians(dq)])
            ok = own(s) and abs(s['lo']['deg'] - ref['lo']['deg']) < .2 and abs(s['hi']['deg'] - ref['hi']['deg']) < .2
        print(f"    J2 sweep with J3 at {np.degrees(cand):7.1f}: {'ok ' if ok else 'no '}{[ref[k] and (ref[k]['deg'], ref[k]['movers']) for k in ('lo', 'hi')]}", flush=True)
        if ok:
            start_q3, j2 = cand, ref
            break
    if start_q3 is None:
        raise SystemExit('No J3 start pose keeps the J2 stop sweep clear')
    start_q3 = float(np.radians(round(np.degrees((start_q3 + np.pi) % (2 * np.pi) - np.pi))))
    q2_mid = np.radians((j2['lo']['deg'] + j2['hi']['deg']) / 2)
    plan.append(dict(joint=1, method='hard_stops', predicted=j2, reference_deg=round(float(np.degrees(q2_mid)), 2),
                     note='Upper arm swept until its own body stops it on each side; reference = midpoint.'))

    # J3: with J2 calibrated and parked, swing the forearm both ways until it touches something
    # (table or turret). Choose the J2 park angle that gives contacts on both sides.
    best = None
    lo_lim, hi_lim = np.radians(j2['lo']['deg'] + 8), np.radians(j2['hi']['deg'] - 8)
    for q2p in np.linspace(lo_lim, hi_lim, 9):
        s = stops(3, [0., q2p, start_q3])
        if s['lo'] and s['hi']:
            span = s['hi']['deg'] - s['lo']['deg']
            tables = sum('table' in ' '.join(s[side]['contact']) for side in ('lo', 'hi'))
            score = (tables, -abs(span - 180))
            if best is None or score > best[0]:
                best = (score, q2p, s)
    if best is None:
        raise SystemExit('No J2 park angle gives forearm contacts on both sides')
    _, q2_park, j3s = best
    plan.append(dict(joint=2, method='contacts', park_deg={'1': round(float(np.degrees(q2_park)), 2)}, predicted=j3s,
                     reference_deg=round((j3s['lo']['deg'] + j3s['hi']['deg']) / 2, 2),
                     note='J2 parked; forearm swung both ways to its first contact (table or turret).'))
    j1 = hard[names[0]]
    if j1['lo'] and j1['hi']:
        j1_step = dict(joint=0, method='hard_stops', predicted=j1, reference_deg=round((j1['lo']['deg'] + j1['hi']['deg']) / 2, 2),
                       note='Turret swept between its stops.')
    else:
        j1_step = dict(joint=0, method='manual', predicted=j1, reference_deg=0.,
                       note='No mechanical stop in the CAD. Align the turret to its zero mark (shown in the sim) and confirm.')
    plan.insert(0, j1_step)  # J1 first (hand-aligned with the start pose), then J2, then J3
    calibration = dict(start_pose_deg=[0., round(float(np.degrees(q2_mid)), 2), round(float(np.degrees(start_q3)), 2)],
                       table_z_mm=table_z, contact_mm=CONTACT_MM, steps=plan)
    print('Calibration plan:', json.dumps(calibration, indent=1), flush=True)

    # ---- browser collision spheres (zero pose); pairs overlapping at zero are joint interfaces
    sph = {b: spheres(np.concatenate([pts[n] for n in pts if link_body[n] == b])) for b in body_key.values()}
    skip = []
    keys = list(body_key.values())
    for i, a in enumerate(keys):
        for b in keys[:i]:
            A, B = np.array(sph[a]), np.array(sph[b])
            d = np.linalg.norm(A[:, None, :3] - B[None, :, :3], axis=2)
            ii, jj = np.where(d < A[:, None, 3] + B[None, :, 3])
            skip += [[a, int(x), b, int(y)] for x, y in zip(ii, jj)]

    manifest = dict(
        source=dict(file=src.name, urdf=urdf.name, sha256=hashlib.sha256(src.read_bytes()).hexdigest() if src.is_file() else None,
                    welded=welded, dropped=sorted(drop)),
        units='mm', up='+z', frame='world = diag(1,-1,-1) @ (CAD - origin); origin on J1 at the table',
        origin_cad_mm=np.round(origin, 3).tolist(), bodies=parts, joints=joints, tool=np.round(tool, 3).tolist(),
        tool_links=tool_links,
        analytic=analytic, hard_stops=hard, calibration=calibration,
        collision=dict(spheres=sph, skip_pairs=skip, table_z_mm=table_z))
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=1) + '\n')
    tris = sum(p['triangles'] for b in parts.values() for p in b.values())
    print(f'Wrote {len(list(OUT.glob("*.bin")))} meshes ({tris} triangles), manifest in {time.time() - t_start:.0f}s')
    print(f"Closed form: h={analytic['h']:.1f} L2={analytic['L2']:.1f} L3={analytic['L3']:.1f} d={analytic['d']:.1f} mm, "
          f"FK error {err:.1e} mm")


if __name__ == '__main__':
    main()
