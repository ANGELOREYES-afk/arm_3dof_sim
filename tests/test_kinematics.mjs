// node tests/test_kinematics.mjs
import { readFileSync } from 'node:fs';
import assert from 'node:assert/strict';
import { createArm, bodyTransform, rad, deg } from '../web/js/kinematics.js';

const root = new URL('..', import.meta.url);
const cfg = JSON.parse(readFileSync(new URL('config/arm.json', root)));
const manifest = JSON.parse(readFileSync(new URL('web/meshes/manifest.json', root)));
const arm = createArm(cfg, manifest);
const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);
const poe = (q) => {
  const { R, t } = bodyTransform(manifest.joints, 3, q);
  const p = manifest.tool;
  return [0, 1, 2].map((r) => R[3 * r] * p[0] + R[3 * r + 1] * p[1] + R[3 * r + 2] * p[2] + t[r]);
};

// 1. The closed form matches the URDF chain (product of exponentials on the exported axes).
{
  let worst = 0;
  for (let k = 0; k < 500; k++) {
    const q = [0, 1, 2].map(() => (Math.random() * 2 - 1) * Math.PI);
    const a = arm.fk(q).tool, b = poe(q);
    worst = Math.max(worst, Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]));
  }
  assert.ok(worst < 0.5, `closed form vs URDF chain: ${worst} mm`);
  const z = arm.fk([0, 0, 0]).tool;
  for (let i = 0; i < 3; i++) close(z[i], manifest.tool[i], 0.5, 'zero pose = CAD tool point');
}

// 2. The CAD pose, ready pose and calibration start pose are valid.
for (const [name, q] of [['zero', [0, 0, 0]], ['ready', cfg.ready_pose_deg], ['cal start', manifest.calibration.start_pose_deg]]) {
  const qr = q.map(rad);
  assert.equal(arm.collision(qr), null, `${name} pose collides: ${arm.collision(qr)}`);
}

// 3. Round trip over a grid of valid poses.
let n = 0;
const [l1, l2, l3] = arm.limits;
for (let a = deg(l1[0]); a <= deg(l1[1]); a += 30)
  for (let b = deg(l2[0]); b <= deg(l2[1]); b += 6)
    for (let c = deg(l3[0]); c <= deg(l3[1]); c += 15) {
      const q = [rad(a), rad(b), rad(c)];
      if (!arm.valid(q)) continue;
      const p = arm.fk(q).tool;
      for (const s of arm.solutions(p)) {
        const pp = arm.fk(s).tool;
        for (let i = 0; i < 3; i++) close(pp[i], p[i], 1e-6, `solution FK ${[a, b, c]}`);
      }
      const back = arm.ik(p, q);
      assert.ok(back, `ik failed at ${[a, b, c]}`);
      for (let i = 0; i < 3; i++) close(back[i], q[i], 1e-6, `ik ${[a, b, c]} joint ${i}`);
      n++;
    }
assert.ok(n > 200, `only ${n} valid poses sampled`);

// 4. Unreachable targets.
assert.equal(arm.ik([2000, 0, 200], [0, 0, 0]), null);
assert.equal(arm.why([2000, 0, 200], [0, 0, 0]), 'out of reach');

// 5. Cartesian steps keep a straight line within joint speed limits, and a dive at the table
//    slides along it instead of stopping.
{
  let q = cfg.ready_pose_deg.map(rad);
  let p = arm.fk(q).tool;
  const p0 = p.slice(), dt = 1 / 60;
  for (let k = 0; k < 90; k++) {
    const r = arm.cartesianStep(p, q, [0, 1.5, 0], dt);
    for (let i = 0; i < 3; i++) assert.ok(Math.abs(r.q[i] - q[i]) <= arm.vmax[i] * dt + 1e-9, 'speed limit');
    ({ p, q } = r);
  }
  close(p[0], p0[0], 1e-6, 'line x');
  close(p[2], p0[2], 1e-6, 'line z');
  assert.ok(Math.abs(p[1] - p0[1]) > 60, 'moved along y');

  // The ready pose is near full reach, so dive while drawing the tool in toward the base.
  q = cfg.ready_pose_deg.map(rad);
  p = arm.fk(q).tool;
  const inward = [-p[0], -p[1]].map((v) => (0.6 * v) / Math.hypot(p[0], p[1]));
  let landed = null;
  for (let k = 0; k < 900; k++) {
    const r = arm.cartesianStep(p, q, [inward[0], inward[1], -2], dt);
    ({ p, q } = r);
    assert.equal(arm.collision(q), null, 'never steps into a collision');
    if (landed === null && r.blocked === 'collision: table') landed = Math.hypot(p[0], p[1]);
  }
  assert.ok(landed !== null, 'reached the table');
  const radius = Math.hypot(p[0], p[1]);
  assert.ok(radius < landed - 10, `slid along the table (${landed.toFixed(1)} -> ${radius.toFixed(1)} mm from the yaw axis)`);
}

// 6. Joint mode respects limits.
{
  let q = [0, rad(20), rad(-60)];
  for (let k = 0; k < 400; k++) q = arm.jointStep(q, [0.05, 0, 0]).q;
  close(deg(q[0]), cfg.joints[0].max_deg, 1e-9, 'J1 stops at its limit');
}

// 7. Gravity: same model and sign as bridge/arm_session.py (3.96 Nm with the arm straight out).
{
  const A = manifest.analytic;
  const q = [0, (Math.PI / 2 - A.offset[1]) * A.sign[1], (0 - A.offset[2]) * A.sign[2]];
  const g = arm.gravity(q);
  close(Math.abs(g[1]), 3.96, 0.01, 'J2 hold torque');
  assert.ok(g[1] * A.sign[1] < 0 && g[2] * A.sign[2] < 0 && g[0] === 0);
}

console.log(`kinematics: ok (${n} poses round-tripped; closed form within 0.5 mm of the URDF chain)`);
