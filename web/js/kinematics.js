// Forward/inverse kinematics for the yaw–pitch–pitch arm. Millimetres and radians, Z up.
//
// J1 yaws about world z. J2 (shoulder) and J3 (elbow) pitch about axes parallel to the J1-frame
// y axis. q = [0, 0, 0] points the upper arm and forearm straight up; +q2/+q3 lean toward +x.
// The tool sits in the forearm plane, offset sideways from the yaw axis by d (from the CAD).

export const rad = (d) => (d * Math.PI) / 180;
export const deg = (r) => (r * 180) / Math.PI;

// Plate outlines drawn by the renderer; the collision checks use the same radii.
export const LINKS = {
  upper: { r0: 25, r1: 33 }, // shoulder hub covers the flange's r = 17 bolt circle
  fore: { r0: 30, r1: 12 },
};

const TAU = Math.PI * 2;
const unwrapNear = (a, ref) => a + TAU * Math.round((ref - a) / TAU);

export function createArm(cfg, manifest) {
  const geo = manifest.geometry;
  const h = geo.shoulder_height_mm;
  const d = geo.tool_offset_y_mm;
  const L2 = cfg.geometry.upper_arm_mm;
  const L3 = cfg.geometry.forearm_mm;
  const motorR = geo.motor_diameter_mm / 2;
  const floor = cfg.geometry.floor_clearance_mm;
  const keepUpper = geo.keepout_upper_arm_mm; // [x0, x1, z0, z1] boxes in the arm plane
  const keepElbow = geo.keepout_elbow_motor_mm;
  const limits = cfg.joints.map((j) => [rad(j.min_deg), rad(j.max_deg)]);
  const vmax = cfg.joints.map((j) => rad(j.max_speed_dps));

  function planar(q) {
    const s2 = Math.sin(q[1]), c2 = Math.cos(q[1]);
    const s23 = Math.sin(q[1] + q[2]), c23 = Math.cos(q[1] + q[2]);
    return { er: L2 * s2, ez: h + L2 * c2, tr: L2 * s2 + L3 * s23, tz: h + L2 * c2 + L3 * c23, s2, c2, s23, c23 };
  }

  const toWorld = (q1, r, lateral, z) => [
    r * Math.cos(q1) - lateral * Math.sin(q1),
    r * Math.sin(q1) + lateral * Math.cos(q1),
    z,
  ];

  function fk(q) {
    const p = planar(q);
    return { tool: toWorld(q[0], p.tr, d, p.tz), elbow: toWorld(q[0], p.er, d, p.ez), r: p.tr };
  }

  // Every analytic solution: two reach branches (in front of / behind the yaw axis) x two elbows.
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
      for (const sign of [1, -1]) {
        const q3 = sign * Math.acos(c3);
        const q2 = Math.atan2(r, v) - Math.atan2(L3 * Math.sin(q3), L2 + L3 * Math.cos(q3));
        out.push([q1, q2, q3]);
      }
    }
    return out;
  }

  const withinLimits = (q) => q.every((v, i) => v >= limits[i][0] - 1e-9 && v <= limits[i][1] + 1e-9);

  const keepoutDist = (boxes, x, z) =>
    Math.min(...boxes.map(([x0, x1, z0, z1]) => Math.hypot(Math.max(x0 - x, 0, x - x1), Math.max(z0 - z, 0, z - z1))));

  // Returns null when clear, else what the arm would hit. No body may come within floor_clearance
  // of the table (so the tool centre bottoms out at tip radius + clearance). The forearm and tool sit outside the
  // turret's slab, so they only meet the floor; the upper arm and elbow motor can meet the turret.
  function collision(q) {
    const p = planar(q);
    if (p.ez - motorR < floor) return 'floor';
    for (let s = 0; s <= L3; s += L3 / 10) {
      const r = LINKS.fore.r0 + ((LINKS.fore.r1 - LINKS.fore.r0) * s) / L3;
      if (p.ez + s * p.c23 - r < floor) return 'floor';
    }
    if (keepoutDist(keepElbow, p.er, p.ez) < motorR + 3) return 'base';
    for (let s = 0; s <= L2; s += L2 / 15) {
      const r = LINKS.upper.r0 + ((LINKS.upper.r1 - LINKS.upper.r0) * s) / L2;
      if (keepoutDist(keepUpper, s * p.s2, h + s * p.c2) < r + 2) return 'base';
    }
    return null;
  }

  const valid = (q) => withinLimits(q) && !collision(q);

  // Nearest valid solution to qRef, so motion stays on its branch unless it passes a singularity.
  function ik(p, qRef) {
    let best = null;
    let bestCost = Infinity;
    for (const raw of solutions(p)) {
      const q = raw.map((a, i) => unwrapNear(a, qRef[i]));
      if (!valid(q)) continue;
      const cost = q.reduce((acc, a, i) => acc + (a - qRef[i]) ** 2, 0);
      if (cost < bestCost) [best, bestCost] = [q, cost];
    }
    return best;
  }

  // Why a target has no valid solution (for the HUD).
  function why(p, qRef) {
    const all = solutions(p);
    if (!all.length) return 'out of reach';
    const qs = all.map((raw) => raw.map((a, i) => unwrapNear(a, qRef[i])));
    if (!qs.some(withinLimits)) return 'joint limit';
    return 'collision: ' + (qs.filter(withinLimits).map(collision).find(Boolean) || 'base');
  }

  // Move the tool target by dp within dt. Respects joint speed limits (scales the step along the
  // same line) and slides along whatever boundary blocks it, one axis at a time.
  function cartesianStep(p, q, dp, dt) {
    const attempt = (pp, qq) => {
      let qn = ik(pp, qq);
      if (!qn) return null;
      const ratio = Math.max(...qn.map((a, i) => Math.abs(a - qq[i]) / (vmax[i] * dt)));
      if (ratio > 1) {
        const scaled = pp.map((v, i) => p[i] + (v - p[i]) / ratio);
        qn = ik(scaled, qq);
        return qn ? { p: scaled, q: qn } : null;
      }
      return { p: pp, q: qn };
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

  // Joint-space step: clamp to limits, drop any joint whose move would collide.
  function jointStep(q, dq) {
    let qn = q.map((v, i) => Math.min(limits[i][1], Math.max(limits[i][0], v + dq[i])));
    if (valid(qn)) return { q: qn, blocked: null };
    let cur = q.slice();
    for (let i = 0; i < 3; i++) {
      const t = cur.slice();
      t[i] = qn[i];
      if (valid(t)) cur = t;
    }
    return { q: cur, blocked: collision(qn) ? 'collision: ' + collision(qn) : 'joint limit' };
  }

  // Joint torques (Nm) that hold the pose against gravity, from the config masses.
  function gravity(q) {
    const m = cfg.geometry.mass_kg;
    const l2 = L2 / 1000, l3 = L3 / 1000;
    const { s2, s23 } = planar(q);
    const t3 = (m.forearm * l3) / 2 * s23 + m.payload * l3 * s23;
    const t2 = ((m.upper_arm * l2) / 2 + m.elbow_motor * l2 + (m.forearm + m.payload) * l2) * s2 + t3;
    return [0, -9.81 * t2, -9.81 * t3];
  }

  return { h, d, L2, L3, limits, vmax, fk, ik, solutions, collision, withinLimits, valid, why, cartesianStep, jointStep, gravity };
}
