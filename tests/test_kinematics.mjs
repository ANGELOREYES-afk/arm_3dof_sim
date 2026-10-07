// node tests/test_kinematics.mjs
import { readFileSync } from 'node:fs';
import assert from 'node:assert/strict';
import { createArm, rad, deg, LINKS } from '../web/js/kinematics.js';

const root = new URL('..', import.meta.url);
const cfg = JSON.parse(readFileSync(new URL('config/arm.json', root)));
const manifest = JSON.parse(readFileSync(new URL('web/meshes/manifest.json', root)));
const arm = createArm(cfg, manifest);
const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);

// 1. Zero pose: both links straight up, tool offset sideways by d.
{
  const { tool } = arm.fk([0, 0, 0]);
  close(tool[0], 0, 1e-9, 'x');
  close(tool[1], arm.d, 1e-9, 'y');
  close(tool[2], arm.h + arm.L2 + arm.L3, 1e-9, 'z');
  // +q2 leans toward +x; +q1 yaws +x toward +y.
  assert.ok(arm.fk([0, rad(30), 0]).tool[0] > 0);
  const t = arm.fk([rad(90), rad(30), 0]).tool;
  assert.ok(t[1] > 100 && Math.abs(t[0] + arm.d) < 1e-9, 'yaw 90 swings the reach onto +y');
}

// 2. Round trip over a grid of valid poses: every analytic solution reproduces the point, and
//    the nearest-solution IK returns the original pose.
let n = 0;
for (let a = -160; a <= 160; a += 20)
  for (let b = -90; b <= 90; b += 15)
    for (let c = -140; c <= 140; c += 20) {
      const q = [rad(a), rad(b), rad(c)];
      if (!arm.valid(q) || Math.abs(c) < 1) continue;
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
assert.ok(n > 300, `only ${n} valid poses sampled`);

// 3. Unreachable / invalid targets.
assert.equal(arm.ik([1000, 0, 200], [0, 0, 0]), null);
assert.equal(arm.why([1000, 0, 200], [0, 0, 0]), 'out of reach');
assert.equal(arm.ik([0, 0, 200], [0, 0, 0]), null, 'inside the lateral offset cylinder');
assert.equal(arm.collision([0, rad(100), rad(80)]), 'floor', 'tool driven into the table');
assert.match(arm.collision([0, rad(100), rad(-150)]) || '', /floor|base/);

// 4. Cartesian steps: a straight line stays straight, never exceeds joint speed, and a step into
//    the floor slides along it instead of stopping.
{
  let q = [0, rad(35), rad(80)];
  let p = arm.fk(q).tool;
  const dt = 1 / 60;
  for (let k = 0; k < 120; k++) {
    const r = arm.cartesianStep(p, q, [0, 2, 0], dt);
    for (let i = 0; i < 3; i++) assert.ok(Math.abs(r.q[i] - q[i]) <= arm.vmax[i] * dt + 1e-9, 'speed limit');
    ({ p, q } = r);
  }
  const p0 = arm.fk([0, rad(35), rad(80)]).tool;
  close(p[0], p0[0], 1e-6, 'line x');
  close(p[2], p0[2], 1e-6, 'line z');
  assert.ok(p[1] - p0[1] > 200, 'moved along y');

  // Dive at the floor while pushing outward: it must land on the floor, then keep sliding outward.
  q = [0, rad(30), rad(110)];
  p = arm.fk(q).tool;
  let landedX = null;
  for (let k = 0; k < 600; k++) {
    const r = arm.cartesianStep(p, q, [0.5, 0, -3], dt);
    ({ p, q } = r);
    assert.equal(arm.collision(q), null, 'never steps into a collision');
    assert.ok(p[2] >= cfg.geometry.floor_clearance_mm + LINKS.fore.r1 - 1e-6, 'tip hub stays above the floor');
    if (landedX === null && r.blocked === 'collision: floor') landedX = p[0];
  }
  assert.ok(landedX !== null, 'reached the floor');
  assert.ok(p[0] > landedX + 20, `slid along the floor (${landedX.toFixed(1)} -> ${p[0].toFixed(1)})`);
}

// 5. Joint mode respects limits.
{
  let q = [0, 0, 0];
  for (let k = 0; k < 400; k++) q = arm.jointStep(q, [0.05, 0, 0]).q;
  close(deg(q[0]), cfg.joints[0].max_deg, 1e-9, 'J1 stops at its limit');
}

// 6. Gravity: same model and sign as bridge/arm_session.py (-1.04 Nm horizontal at J2).
{
  const g = arm.gravity([0, rad(90), 0]);
  close(g[1], -1.04, 0.01, 'J2 hold torque');
  assert.ok(g[2] < 0 && g[0] === 0);
}

console.log(`kinematics: ok (${n} poses round-tripped)`);
