// Kinematics for the URDF arm. Millimetres and radians, world Z up. Joint angles are the URDF's
// (J1 = revolute_2, J2 = revolute_1, J3 = revolute_3), so they match RViz and the motor bridge.
//
// FK/IK use a closed form fitted by tools/import_urdf.py: J1 yaws about world z; J2 and J3 pitch
// about parallel horizontal axes; the tool sits a fixed distance d sideways along those axes. In
// closed-form ("planar") angles, qp = sign * q + offset, and qp = [0, 0, 0] points the arm along
// +x with both links straight up. The importer checks it against the URDF chain (< 0.5 mm).

export const rad = (d) => (d * Math.PI) / 180;
export const deg = (r) => (r * 180) / Math.PI;

const TAU = Math.PI * 2;
const unwrapNear = (a, ref) => a + TAU * Math.round((ref - a) / TAU);

// Rigid motion of body k (1..3) at q about the zero-pose joint axes (product of exponentials).
export function bodyTransform(joints, k, q) {
  let R = [1, 0, 0, 0, 1, 0, 0, 0, 1], t = [0, 0, 0];
  for (let i = 0; i < k; i++) {
    const [x, y, z] = joints[i].axis, c = Math.cos(q[i]), s = Math.sin(q[i]), v = 1 - c;
    const Ri = [c + x * x * v, x * y * v - z * s, x * z * v + y * s,
      y * x * v + z * s, c + y * y * v, y * z * v - x * s,
      z * x * v - y * s, z * y * v + x * s, c + z * z * v];
    const o = joints[i].origin;
    const ti = [0, 1, 2].map((r) => o[r] - (Ri[3 * r] * o[0] + Ri[3 * r + 1] * o[1] + Ri[3 * r + 2] * o[2]));
    const Rn = [];
    for (let r = 0; r < 3; r++) for (let col = 0; col < 3; col++) Rn.push(R[3 * r] * Ri[col] + R[3 * r + 1] * Ri[3 + col] + R[3 * r + 2] * Ri[6 + col]);
    t = [0, 1, 2].map((r) => R[3 * r] * ti[0] + R[3 * r + 1] * ti[1] + R[3 * r + 2] * ti[2] + t[r]);
    R = Rn;
  }
  return { R, t };
}

export function createArm(cfg, manifest) {
  const A = manifest.analytic;
  const { h, L2, L3, d } = A;
  const sign = A.sign, offset = A.offset;
  const toP = (q) => q.map((v, i) => sign[i] * v + offset[i]);
  const fromP = (qp) => qp.map((v, i) => sign[i] * (v - offset[i]));
  const vmax = cfg.joints.map((j) => rad(j.max_speed_dps));
  let limits = cfg.joints.map((j) => [rad(j.min_deg), rad(j.max_deg)]);
  const floor = (manifest.collision.table_z_mm || 0) + cfg.geometry.floor_clearance_mm;

  // Collision spheres per body; pairs already overlapping at the zero pose are joint interfaces.
  const BODIES = ['base', 'j1', 'j2', 'j3'];
  const BODY_NAME = { base: 'base', j1: 'turret', j2: 'upper arm', j3: 'forearm' };
  const sph = BODIES.map((b) => manifest.collision.spheres[b]);
  const skip = {};
  for (const [a, i, b, j] of manifest.collision.skip_pairs) {
    (skip[`${BODIES.indexOf(a)}${BODIES.indexOf(b)}`] ||= new Set()).add(i * 4096 + j);
  }

  function planar(qp) {
    const s2 = Math.sin(qp[1]), c2 = Math.cos(qp[1]), s23 = Math.sin(qp[1] + qp[2]), c23 = Math.cos(qp[1] + qp[2]);
    return { tr: L2 * s2 + L3 * s23, tz: h + L2 * c2 + L3 * c23, s2, s23 };
  }

  const toWorld = (q1, r, z) => [r * Math.cos(q1) - d * Math.sin(q1), r * Math.sin(q1) + d * Math.cos(q1), z];

  function fk(q) {
    const qp = toP(q);
    const p = planar(qp);
    return { tool: toWorld(qp[0], p.tr, p.tz) };
  }

  // All closed-form solutions: reach in front of / behind the yaw axis, elbow either way.
  function solutions([x, y, z]) {
    const rho2 = x * x + y * y;
    if (rho2 < d * d - 1e-6) return [];
    const rr = Math.sqrt(Math.max(0, rho2 - d * d));
    const bearing = Math.atan2(y, x);
    const out = [];
    for (const r of [rr, -rr]) {
      const q1 = bearing - Math.atan2(d, r);
      const v = z - h;
      const c3 = (r * r + v * v - L2 * L2 - L3 * L3) / (2 * L2 * L3);
      if (c3 > 1 || c3 < -1) continue;
      for (const s of [1, -1]) {
        const q3 = s * Math.acos(c3);
        const q2 = Math.atan2(r, v) - Math.atan2(L3 * Math.sin(q3), L2 + L3 * Math.cos(q3));
        out.push(fromP([q1, q2, q3]));
      }
    }
    return out;
  }

  const withinLimits = (q) => q.every((v, i) => v >= limits[i][0] - 1e-9 && v <= limits[i][1] + 1e-9);

  function posed(k, q) {
    if (k === 0) return sph[0];
    const { R, t } = bodyTransform(manifest.joints, k, q);
    return sph[k].map(([x, y, z, r]) => [
      R[0] * x + R[1] * y + R[2] * z + t[0], R[3] * x + R[4] * y + R[5] * z + t[1], R[6] * x + R[7] * y + R[8] * z + t[2], r]);
  }

  // null when clear, else what would be hit: "table", "base" or "turret". A body touching its
  // own parent is that joint's hard stop, which the joint limits already enforce (and the
  // importer measured exactly), so only non-adjacent bodies and the table are checked here.
  function collision(q) {
    const P = [0, 1, 2, 3].map((k) => posed(k, q));
    for (let k = 1; k <= 3; k++) for (const s of P[k]) if (s[2] - s[3] < floor) return 'table';
    for (let a = 2; a <= 3; a++) {
      for (let b = 0; b < a - 1; b++) {
        const sk = skip[`${a}${b}`];
        const PA = P[a], PB = P[b];
        for (let i = 0; i < PA.length; i++) {
          const [x, y, z, r] = PA[i];
          for (let j = 0; j < PB.length; j++) {
            const s = PB[j], rr = r + s[3];
            const dx = x - s[0], dy = y - s[1], dz = z - s[2];
            if (dx * dx + dy * dy + dz * dz < rr * rr && !sk?.has(i * 4096 + j)) return BODY_NAME[BODIES[b]];
          }
        }
      }
    }
    return null;
  }

  const valid = (q) => withinLimits(q) && !collision(q);

  // Nearest valid solution to qRef, so motion stays on its branch unless it passes a singularity.
  function ik(p, qRef) {
    let best = null, bestCost = Infinity;
    for (const raw of solutions(p)) {
      const q = raw.map((a, i) => unwrapNear(a, qRef[i]));
      const cost = q.reduce((acc, a, i) => acc + (a - qRef[i]) ** 2, 0);
      if (cost < bestCost && valid(q)) [best, bestCost] = [q, cost];
    }
    return best;
  }

  function why(p, qRef) {
    const all = solutions(p).map((raw) => raw.map((a, i) => unwrapNear(a, qRef[i])));
    if (!all.length) return 'out of reach';
    const inLimits = all.filter(withinLimits);
    if (!inLimits.length) return 'joint limit';
    return 'collision: ' + (inLimits.map(collision).find(Boolean) || 'arm');
  }

  // Move the tool target by dp within dt: joint speed limits scale the step along the same line,
  // and a blocked step slides along the boundary one axis at a time.
  function cartesianStep(p, q, dp, dt) {
    const attempt = (pp, qq) => {
      const from = qq === q ? p : fk(qq).tool;
      for (let k = 0; k < 6; k++) { // IK is nonlinear: shrink the step until every joint is within speed
        const qn = ik(pp, qq);
        if (!qn) return null;
        const ratio = Math.max(...qn.map((a, i) => Math.abs(a - qq[i]) / (vmax[i] * dt)));
        if (ratio <= 1) return { p: pp, q: qn };
        pp = pp.map((v, i) => from[i] + (v - from[i]) / (ratio * 1.05));
      }
      return null;
    };
    const whole = attempt(p.map((v, i) => v + dp[i]), q);
    if (whole) return { ...whole, blocked: null };
    let cur = { p, q };
    for (const i of [0, 1, 2].sort((a, b) => Math.abs(dp[b]) - Math.abs(dp[a]))) {
      if (!dp[i]) continue;
      const pp = cur.p.slice();
      pp[i] += dp[i];
      cur = attempt(pp, cur.q) || cur;
    }
    return { ...cur, blocked: why(p.map((v, i) => v + dp[i]), q) };
  }

  function jointStep(q, dq) {
    const qn = q.map((v, i) => Math.min(limits[i][1], Math.max(limits[i][0], v + dq[i])));
    if (valid(qn)) return { q: qn, blocked: null };
    let cur = q.slice();
    for (let i = 0; i < 3; i++) {
      const t = cur.slice();
      t[i] = qn[i];
      if (valid(t)) cur = t;
    }
    const hit = collision(qn);
    return { q: cur, blocked: hit ? 'collision: ' + hit : 'joint limit' };
  }

  // Joint torques (Nm, URDF sign) holding the pose against gravity, from the config masses:
  // upper arm at mid-link, elbow assembly on the J3 axis, forearm at mid-link, payload at the tool.
  function gravity(q) {
    const m = cfg.geometry.mass_kg;
    const l2 = L2 / 1000, l3 = L3 / 1000;
    const { s2, s23 } = planar(toP(q));
    const t3 = (m.forearm * l3) / 2 * s23 + m.payload * l3 * s23;
    const t2 = ((m.upper_arm * l2) / 2 + m.elbow_assembly * l2 + (m.forearm + m.payload) * l2) * s2 + t3;
    return [0, -9.81 * t2 * sign[1], -9.81 * t3 * sign[2]];
  }

  return {
    h, d, L2, L3, vmax, toP, fromP,
    get limits() { return limits; },
    setLimits(next) { limits = next; },
    fk, ik, solutions, collision, withinLimits, valid, why, cartesianStep, jointStep, gravity,
  };
}
