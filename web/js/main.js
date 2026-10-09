import * as THREE from '../vendor/three.module.js';
import { createArm, rad, deg } from './kinematics.js';
import { Controls } from './gamepad.js';
import { Orbit } from './orbit.js';
import { Bridge } from './bridge.js';

const $ = (id) => document.getElementById(id);
const [cfg, manifest] = await Promise.all([
  fetch('../config/arm.json').then((r) => r.json()),
  fetch('meshes/manifest.json').then((r) => r.json()),
]);
const arm = createArm(cfg, manifest);
const PLAN = manifest.calibration;

// ---------------------------------------------------------------- scene
const viewport = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
viewport.prepend(renderer.domElement);
renderer.domElement.tabIndex = 0;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12151a);
scene.fog = new THREE.Fog(0x12151a, 2600, 5200);
const camera = new THREE.PerspectiveCamera(38, 1, 5, 10000);
const orbit = new Orbit(camera, renderer.domElement, {
  target: [140, -200, 190], azimuth: rad(-150), elevation: rad(20), distance: 1650,
});

scene.add(new THREE.HemisphereLight(0xe4ecff, 0x2b2622, 1.15));
const key = new THREE.DirectionalLight(0xffffff, 2.4);
key.position.set(-500, -900, 1300);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
Object.assign(key.shadow.camera, { left: -850, right: 850, top: 850, bottom: -850, near: 200, far: 3500 });
key.shadow.bias = -0.0004;
scene.add(key);
const rim = new THREE.DirectionalLight(0xa9c8ff, 0.9);
rim.position.set(900, 600, 500);
scene.add(rim);

const floor = new THREE.Mesh(new THREE.CircleGeometry(2200, 96), new THREE.MeshStandardMaterial({ color: 0x1a1e25, roughness: 0.96 }));
floor.position.z = manifest.collision.table_z_mm;
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(1600, 32, 0x3a4350, 0x262c35);
grid.rotation.x = Math.PI / 2;
grid.position.z = manifest.collision.table_z_mm + 0.3;
scene.add(grid);

// ---------------------------------------------------------------- CAD meshes on the URDF chain
const MAT = {
  motor: new THREE.MeshStandardMaterial({ color: 0x2e333b, metalness: 0.35, roughness: 0.45, flatShading: true }),
  frame: new THREE.MeshStandardMaterial({ color: 0x7d91ad, metalness: 0.12, roughness: 0.62, flatShading: true }),
  link: new THREE.MeshStandardMaterial({ color: 0xe27d3c, metalness: 0.1, roughness: 0.5, flatShading: true }),
  tube: new THREE.MeshStandardMaterial({ color: 0x3b4048, metalness: 0.5, roughness: 0.35, flatShading: true }),
  hub: new THREE.MeshStandardMaterial({ color: 0x4a515c, metalness: 0.45, roughness: 0.4, flatShading: true }),
  hardware: new THREE.MeshStandardMaterial({ color: 0xb4bac2, metalness: 0.4, roughness: 0.35, flatShading: true }),
  tool: new THREE.MeshStandardMaterial({ color: 0xf2c14e, metalness: 0.2, roughness: 0.35, emissive: 0x2e2000 }),
};
// Measured pose. Pushed back in depth so it only shows where it differs from the sim.
const GHOST = new THREE.MeshBasicMaterial({
  color: 0x46d39a, transparent: true, opacity: 0.22, depthWrite: false,
  polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 8,
});

async function loadBody(name) {
  const out = {};
  await Promise.all(Object.entries(manifest.bodies[name]).map(async ([mat, info]) => {
    const buf = await fetch(`meshes/${info.file}`).then((r) => r.arrayBuffer());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buf, 0, info.vertices * 3), 3));
    g.setIndex(new THREE.BufferAttribute(new Uint32Array(buf, info.vertices * 12, info.triangles * 3), 1));
    g.computeBoundingSphere();
    out[mat] = g;
  }));
  return out;
}
const bodies = Object.fromEntries(await Promise.all(['base', 'j1', 'j2', 'j3'].map(async (b) => [b, await loadBody(b)])));

// Meshes are in world coordinates at the URDF zero pose, so each joint is a rotation about its
// zero-pose axis through its zero-pose origin, nested (product of exponentials).
function buildRig(ghost) {
  const mesh = (geom, mat) => {
    const m = new THREE.Mesh(geom, ghost ? GHOST : MAT[mat]);
    m.castShadow = m.receiveShadow = !ghost;
    return m;
  };
  const addAll = (parent, body) => Object.entries(body).forEach(([mat, g]) => parent.add(mesh(g, mat)));
  const root = new THREE.Group();
  if (!ghost) addAll(root, bodies.base);
  const pivots = [];
  let parent = root;
  manifest.joints.forEach((j, i) => {
    const at = new THREE.Group();
    at.position.fromArray(j.origin);
    parent.add(at);
    const spin = new THREE.Group();
    at.add(spin);
    const back = new THREE.Group();
    back.position.fromArray(j.origin).negate();
    spin.add(back);
    addAll(back, bodies[`j${i + 1}`]);
    pivots.push({ spin, axis: new THREE.Vector3().fromArray(j.axis) });
    parent = back;
  });
  const tool = mesh(new THREE.SphereGeometry(9, 24, 12), 'tool');
  tool.position.fromArray(manifest.tool);
  parent.add(tool);
  scene.add(root);
  return { root, set(q) { pivots.forEach((p, i) => p.spin.quaternion.setFromAxisAngle(p.axis, q[i])); } };
}
const rig = buildRig(false);
const ghost = buildRig(true);
ghost.root.visible = false;

// Target marker: sphere, drop line and floor ring so its height reads in 3D.
const target = new THREE.Group();
target.add(new THREE.Mesh(new THREE.SphereGeometry(14, 32, 16), new THREE.MeshBasicMaterial({ color: 0x3cc3dc, transparent: true, opacity: 0.3, depthWrite: false })));
target.add(new THREE.Mesh(new THREE.TorusGeometry(18, 1, 8, 48), new THREE.MeshBasicMaterial({ color: 0x3cc3dc })));
scene.add(target);
const dropGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const drop = new THREE.Line(dropGeom, new THREE.LineDashedMaterial({ color: 0x3cc3dc, dashSize: 8, gapSize: 6, transparent: true, opacity: 0.6 }));
scene.add(drop);
const floorRing = new THREE.Mesh(new THREE.RingGeometry(12, 16, 40), new THREE.MeshBasicMaterial({ color: 0x3cc3dc, transparent: true, opacity: 0.55 }));
scene.add(floorRing);

const TRAIL = 600;
const trailPos = new Float32Array(TRAIL * 3);
const trailGeom = new THREE.BufferGeometry();
trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
trailGeom.setDrawRange(0, 0);
scene.add(new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: 0xf2c14e, transparent: true, opacity: 0.75 })));
let trailN = 0;
const clearTrail = () => { trailN = 0; trailGeom.setDrawRange(0, 0); };
function pushTrail(p) {
  const last = trailN ? trailPos.subarray((trailN - 1) * 3, trailN * 3) : null;
  if (last && Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]) < 2) return;
  if (trailN === TRAIL) { trailPos.copyWithin(0, 3); trailN--; }
  trailPos.set(p, trailN * 3);
  trailN++;
  trailGeom.setDrawRange(0, trailN);
  trailGeom.attributes.position.needsUpdate = true;
}

$('loading').remove();

// ---------------------------------------------------------------- state
const sim = {
  mode: 'ik',
  q: cfg.ready_pose_deg.map(rad),
  p: null,
  speed: 0.6,
  anim: [],   // queued joint-space moves: { goal, label, force }
  demo: null, // { t } figure-8
  blocked: null,
  msg: null,
  msgUntil: 0,
};
sim.p = arm.fk(sim.q).tool;
const controls = new Controls();
let input = null;

function flash(text, seconds = 3) {
  sim.msg = text;
  sim.msgUntil = performance.now() + seconds * 1000;
}

function moveTo(goal, label, force = false) {
  sim.demo = null;
  sim.anim = [{ goal, label, force }];
}

function goReady() { moveTo(cfg.ready_pose_deg.map(rad), 'ready pose'); }

function flipElbow() {
  const alt = arm.solutions(sim.p)
    .map((s) => s.map((a, i) => a + 2 * Math.PI * Math.round((sim.q[i] - a) / (2 * Math.PI))))
    .filter((s) => arm.valid(s) && Math.abs(s[2] - sim.q[2]) > 0.05)
    .sort((a, b) => Math.abs(a[0] - sim.q[0]) - Math.abs(b[0] - sim.q[0]))[0];
  if (alt) moveTo(alt, 'elbow flip');
  else flash('No valid flipped-elbow solution for this target');
}

function toggleDemo() {
  sim.anim = [];
  sim.demo = sim.demo ? null : { t: 0 };
  if (sim.demo) {
    sim.mode = 'ik';
    clearTrail();
  }
}

// Figure-8 in a vertical plane across the arm's reach, a little inside the ready pose.
const demoCentre = (() => {
  const p = arm.fk(cfg.ready_pose_deg.map(rad)).tool;
  const r = Math.hypot(p[0], p[1]);
  return { c: [p[0] * (r - 170) / r, p[1] * (r - 170) / r, p[2] - 60], u: [-p[1] / r, p[0] / r] };
})();
const demoPath = (t) => {
  const w = (2 * Math.PI) / 10;
  const { c, u } = demoCentre;
  const a = 150 * Math.sin(w * t);
  return [c[0] + u[0] * a, c[1] + u[1] * a, c[2] + 70 * Math.sin(2 * w * t)];
};

// Show what the calibration will do, on the CAD-predicted contacts (sim only, no motors).
function previewCalibration() {
  const s = PLAN.start_pose_deg.map(rad);
  const moves = [{ goal: s.slice(), label: 'start pose (pose the arm like this)', force: true }];
  const q = s.slice();
  for (const step of PLAN.steps) {
    const k = step.joint;
    for (const [j, a] of Object.entries(step.park_deg || {})) q[+j] = rad(a);
    if (step.park_deg) moves.push({ goal: q.slice(), label: `park for ${cfg.joints[k].name}`, force: true });
    if (step.method === 'manual') {
      q[k] = rad(step.reference_deg);
      moves.push({ goal: q.slice(), label: `${cfg.joints[k].name}: align by hand to ${step.reference_deg}°`, force: true });
      continue;
    }
    for (const side of ['lo', 'hi']) {
      q[k] = rad(step.predicted[side].deg);
      moves.push({ goal: q.slice(), label: `${cfg.joints[k].name}: ${side} contact ${step.predicted[side].deg}°`, force: true });
    }
    q[k] = rad(step.reference_deg);
    moves.push({ goal: q.slice(), label: `${cfg.joints[k].name}: reference ${step.reference_deg}°`, force: true });
  }
  sim.demo = null;
  sim.anim = moves;
  clearTrail();
}

function stepAnim(dt) {
  const m = sim.anim[0];
  const dq = m.goal.map((g, i) => g - sim.q[i]);
  const t = Math.max(...dq.map((d, i) => Math.abs(d) / (arm.vmax[i] * sim.speed)));
  if (t < 1e-4) { sim.anim.shift(); return; }
  const qn = sim.q.map((v, i) => v + dq[i] * Math.min(1, dt / t)); // joints arrive together
  if (!m.force && !arm.valid(qn)) {
    flash(`${m.label} blocked: ${arm.collision(qn) || 'joint limit'}`);
    sim.anim = [];
    return;
  }
  sim.q = qn;
  sim.p = arm.fk(qn).tool;
}

function stepDemo(dt) {
  sim.demo.t += dt * sim.speed / 0.6;
  const goal = demoPath(sim.demo.t);
  const v = 320 * sim.speed * dt;
  const d = goal.map((g, i) => g - sim.p[i]);
  const n = Math.hypot(...d);
  const r = arm.cartesianStep(sim.p, sim.q, n > v ? d.map((x) => (x * v) / n) : d, dt);
  Object.assign(sim, { p: r.p, q: r.q, blocked: r.blocked });
}

function stepInput(dt) {
  const gain = input.rt > 0.2 ? 1 / sim.speed : input.lt > 0.2 ? 0.25 : 1;
  const k = Math.min(1, sim.speed * gain);
  if (sim.mode === 'ik') {
    const { forward, right } = orbit.groundAxes();
    const mx = input.lx, my = -input.ly;
    const v = 260 * k * dt;
    const dp = [(right[0] * mx + forward[0] * my) * v, (right[1] * mx + forward[1] * my) * v, -input.ry * v];
    if (dp.some((x) => x)) {
      const r = arm.cartesianStep(sim.p, sim.q, dp, dt);
      Object.assign(sim, { p: r.p, q: r.q, blocked: r.blocked });
    } else sim.blocked = null;
  } else {
    const dq = [input.lx, -input.ly, -input.ry].map((u, i) => u * arm.vmax[i] * k * dt);
    if (dq.some((x) => x)) {
      const r = arm.jointStep(sim.q, dq);
      Object.assign(sim, { q: r.q, blocked: r.blocked, p: arm.fk(r.q).tool });
    } else sim.blocked = null;
  }
}

// ---------------------------------------------------------------- bridge
let motors = null;
let wasArmed = false;
let limitsShown = '';
const events = [];
const bridge = new Bridge((s) => {
  motors = s;
  if (s?.armed && !wasArmed && s.q_cmd) {
    // Armed at the motors' measured pose: start the sim there so nothing jumps.
    sim.q = s.q_cmd.map(rad);
    sim.p = arm.fk(sim.q).tool;
    sim.anim = [];
    sim.demo = null;
    clearTrail();
    flash('Armed — sim moved to the measured pose. Release and hold RB to drive.', 4);
  }
  wasArmed = !!s?.armed;
  const lim = s?.limits_deg;
  if (lim && JSON.stringify(lim) !== limitsShown) {  // calibrated limits from the bridge
    limitsShown = JSON.stringify(lim);
    arm.setLimits(lim.map(([a, b]) => [rad(a), rad(b)]));
    buildJointRows();
  }
  if (s?.events?.length) {
    events.push(...s.events);
    events.splice(0, Math.max(0, events.length - 60));
    renderEvents();
  }
});
const calibrating = () => !!motors?.calibration?.active;

function menuButton() {
  const st = motors?.state || '';
  if (!motors?.connected) return flash('Connect the motors in the panel first');
  if (motors.calibration?.awaiting_confirm) return bridge.cmd('cal_confirm');
  if (!$('m-supported').checked) return flash('Tick “The arm is supported” in the panel first');
  if (st.startsWith('Stopped') || st.startsWith('Fault') || st.startsWith('Calibration failed')) return bridge.cmd('check');
  if (st === 'Ready · Stop') {
    if (!motors.zeroed) return flash('Not calibrated: run Calibrate in the panel');
    return bridge.cmd('arm', { supported: true });
  }
}

function stopAll() {
  bridge.stop();
  sim.demo = null;
  sim.anim = [];
  if (motors?.armed || calibrating()) flash('STOP sent — motors disarmed', 3);
}

// ---------------------------------------------------------------- panel
let jointRows = [];
function buildJointRows() {
  $('joints').innerHTML = '';
  jointRows = cfg.joints.map((j, i) => {
    const [lo, hi] = arm.limits[i].map(deg);
    const el = document.createElement('div');
    el.className = 'joint';
    const pct = (a) => `${Math.min(100, Math.max(0, ((a - lo) / (hi - lo)) * 100))}%`;
    el.innerHTML = `<span class="name">${j.name}</span>
      <div class="track">${lo < 0 && hi > 0 ? `<i class="zero" style="left:${pct(0)}"></i>` : ''}<i class="meas" hidden></i><i class="tgt"></i></div>
      <span class="val">—</span><div class="sub"><span>${lo.toFixed(1)}° … ${hi.toFixed(1)}°</span><span class="meas-txt"></span><span class="tau"></span></div>`;
    $('joints').append(el);
    return {
      set(tgt, meas, tau) {
        el.querySelector('.tgt').style.left = pct(tgt);
        el.querySelector('.val').textContent = `${tgt.toFixed(1)}°`;
        const m = el.querySelector('.meas');
        m.hidden = meas == null;
        if (meas != null) m.style.left = pct(meas);
        el.querySelector('.meas-txt').textContent = meas == null ? '' : `meas ${meas.toFixed(1)}°`;
        el.querySelector('.tau').textContent = i ? `τg ${Math.abs(tau).toFixed(2)} Nm` : '';
      },
    };
  });
}
buildJointRows();

// Calibration plan, as the importer planned it from the CAD.
const METHOD = { manual: 'align by hand', hard_stops: 'own-body stops', contacts: 'contacts' };
function describeStep(s) {
  const name = cfg.joints[s.joint].name;
  if (s.method === 'manual') return `${name}: no mechanical stop in the CAD. Align it to ${s.reference_deg}° by eye, then confirm.`;
  const c = (side) => `${s.predicted[side].deg}° (${s.predicted[side].contact[0].replace(/ \([^)]*\)/g, '').replace(/_8_32_x_+2_1_\d_+Star_Drive_Screw_+276_801\d_/g, '#8-32 screw')})`;
  const park = s.park_deg ? ` with ${Object.entries(s.park_deg).map(([j, a]) => `${cfg.joints[+j].name.split(' ')[0]} at ${a}°`).join(', ')}` : '';
  return `${name}${park}: seek ${c('lo')} and ${c('hi')}; reference ${s.reference_deg}°.`;
}
function renderPlan() {
  const steps = motors?.calibration?.steps?.length ? motors.calibration.steps : PLAN.steps.map((s) => ({ ...s, status: 'pending' }));
  $('cal-steps').innerHTML = steps.map((st, i) => {
    const s = PLAN.steps[i];
    const result = st.status === 'done' && st.measured_span_deg != null
      ? `<div class="res">span ${st.measured_span_deg}° (${st.span_error_deg >= 0 ? '+' : ''}${st.span_error_deg}° vs CAD) · ref ${st.q_ref_deg}°${st.limits_deg ? ` · limits ${st.limits_deg[0]}…${st.limits_deg[1]}°` : ''}</div>`
      : st.error ? `<div class="res err">${st.error}</div>` : '';
    return `<li class="${st.status}"><b>${i + 1}. ${METHOD[s.method]}</b> <span class="st">${st.status}</span><div>${describeStep(s)}</div>${result}</li>`;
  }).join('');
}

function setMode(m) {
  sim.mode = m;
  if (m === 'joint') sim.demo = null;
}
document.querySelectorAll('#mode-seg button').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
$('speed').addEventListener('input', (e) => { sim.speed = +e.target.value; });
$('btn-ready').onclick = goReady;
$('btn-flip').onclick = flipElbow;
$('btn-demo').onclick = toggleDemo;
$('btn-cal').onclick = () => moveTo(PLAN.start_pose_deg.map(rad), 'calibration start pose', true);
$('cal-preview').onclick = previewCalibration;
$('cal-start').onclick = () => bridge.cmd('calibrate', { supported: $('m-supported').checked });
$('cal-confirm').onclick = () => bridge.cmd('cal_confirm');
$('m-source').onchange = () => { $('real-opts').hidden = $('m-source').value !== 'real'; };
$('m-connect').onclick = () => {
  const real = $('m-source').value === 'real';
  bridge.cmd('connect', real ? { demo: false, port: $('m-port').value, baud: +$('m-baud').value } : { demo: true });
};
$('m-disconnect').onclick = () => bridge.cmd('disconnect');
$('m-check').onclick = () => bridge.cmd('check');
$('m-ack').onclick = () => bridge.cmd('ack');
$('m-home').onclick = () => bridge.cmd('home', { supported: $('m-supported').checked });
$('m-arm').onclick = () => bridge.cmd('arm', { supported: $('m-supported').checked });
$('m-stop').onclick = stopAll;
$('m-supported').onchange = (e) => { if (!e.target.checked && (motors?.armed || calibrating())) stopAll(); };

let portsShown = '';
function renderPorts() {
  const key = JSON.stringify(bridge.ports);
  if (key === portsShown) return;
  portsShown = key;
  const sel = $('m-port');
  const cur = sel.value;
  sel.innerHTML = '';
  const opts = [{ device: cfg.bus.port, description: 'config/arm.json' }, ...bridge.ports.filter((p) => p.device && p.device !== cfg.bus.port)];
  for (const p of opts) sel.add(new Option(`${p.device.split('/').pop()} — ${p.description}`, p.device));
  if (cur) sel.value = cur;
}

function renderEvents() {
  const ol = $('m-events');
  ol.innerHTML = '';
  for (const e of events.slice(-40)) {
    const li = document.createElement('li');
    li.textContent = e.trim();
    ol.append(li);
  }
  ol.scrollTop = ol.scrollHeight;
}

function updatePanel() {
  const pad = input;
  $('pad-dot').className = 'dot' + (pad.connected ? ' on' : '');
  $('pad-name').textContent = pad.connected ? pad.name : 'No controller seen — press any button on it (keyboard works too)';
  document.querySelectorAll('#mode-seg button').forEach((b) => b.classList.toggle('on', b.dataset.mode === sim.mode));
  $('speed').value = sim.speed;
  $('speed-out').textContent = `${Math.round(sim.speed * 100)}%`;
  $('btn-demo').classList.toggle('on', !!sim.demo);
  $('btn-demo').textContent = sim.demo ? 'Stop figure-8' : 'Figure-8 demo';

  $('tx').textContent = sim.p[0].toFixed(1);
  $('ty').textContent = sim.p[1].toFixed(1);
  $('tz').textContent = sim.p[2].toFixed(1);
  const tau = arm.gravity(sim.q);
  const fb = motors?.feedback;
  jointRows.forEach((row, i) => row.set(deg(sim.q[i]), fb?.[i]?.q_deg ?? null, tau[i]));
  $('ik-state').textContent = sim.anim.length ? `→ ${sim.anim[0].label}` : sim.demo ? 'figure-8' : sim.mode === 'ik' ? 'IK' : 'joint';

  // Motors panel
  const online = bridge.online && !!motors;
  $('bridge-offline').hidden = online;
  $('bridge-online').hidden = !online;
  $('bridge-link').textContent = online ? (motors.controller ? 'bridge · in control' : 'bridge · view only') : 'offline';
  const badge = $('source-badge');
  badge.className = 'badge' + (online && motors.connected ? (motors.demo ? ' demo' : ' real') : '');
  badge.textContent = online && motors.connected ? (motors.demo ? 'DEMO MOTORS' : 'REAL MOTORS') : 'SIM ONLY';
  ghost.root.visible = !!(online && fb?.every((f) => f && f.q_deg != null));
  const cal = motors?.calibration;
  renderPlan();
  $('cal-saved').textContent = !online || !motors.connected ? 'Connect to see the saved calibration.'
    : cal?.saved ? `Saved ${cal.saved.created} in config/${cal.saved.file}` : 'Not calibrated yet.';
  $('cal-msg').textContent = cal?.message || '';
  $('cal-msg').hidden = !cal?.message;
  $('cal-confirm').hidden = !cal?.awaiting_confirm;
  if (online) {
    renderPorts();
    const st = motors.state;
    $('observer-note').hidden = !motors.observer;
    $('m-state').textContent = st;
    $('m-reason').textContent = motors.reason;
    $('m-state-box').className = 'state' + (motors.armed || cal?.active ? ' armed' : /^(Fault|Calibration failed)/.test(st) ? ' fault' : st.startsWith('Ready') ? ' ready' : '');
    const supported = $('m-supported').checked;
    const ctl = !motors.observer;
    $('m-source').disabled = $('m-port').disabled = $('m-baud').disabled = motors.connected;
    $('m-connect').disabled = !ctl || motors.connected;
    $('m-disconnect').disabled = !ctl || !motors.connected;
    $('m-check').disabled = !ctl || !motors.connected || motors.armed || cal?.active;
    $('m-ack').disabled = !ctl || !st.startsWith('Timeout');
    $('m-home').disabled = !ctl || st !== 'Ready · Stop' || !supported;
    $('m-arm').disabled = !ctl || st !== 'Ready · Stop' || !motors.zeroed || !supported;
    $('cal-start').disabled = !ctl || st !== 'Ready · Stop' || !supported;
    $('k-hz').textContent = motors.connected ? `${motors.loop_hz.toFixed(0)} Hz` : '—';
    $('k-pk').textContent = `${motors.tx}/${motors.rx}`;
    $('k-crc').textContent = motors.crc_errors;
    $('k-to').textContent = motors.reply_timeouts;
    $('m-table').innerHTML = cfg.joints.map((j, i) => {
      const f = fb?.[i];
      const cell = (v, d = 1) => (v == null ? '—' : (+v).toFixed(d));
      return `<tr><td>${j.name.split(' ')[0]}</td><td>${j.id}</td><td>${cell(f?.q_deg)}</td><td>${cell(f?.position_deg)}</td><td>${cell(f?.torque_nm, 2)}</td>
        <td>${cell(f?.voltage, 1)}</td><td>${f ? `${f.driver_c}/${f.winding_c}` : '—'}</td><td>${f ? f.age_ms + 'ms' : '—'}</td></tr>`;
    }).join('');
  } else {
    $('cal-start').disabled = true;
  }

  // HUD
  $('hud-mode').textContent = sim.demo ? 'Figure-8 demo (IK)' : sim.anim.length ? `Sim → ${sim.anim[0].label}` : sim.mode === 'ik' ? 'IK · Cartesian' : 'Joint mode';
  const armPill = $('hud-arm');
  armPill.hidden = !(online && motors.connected);
  if (online && motors.connected) {
    const driving = motors.armed && motors.moving;
    armPill.className = 'pill ' + (motors.armed || cal?.active ? (driving || (cal?.active && controls.deadman()) ? 'live' : 'warn') : '');
    armPill.textContent = cal?.active
      ? (cal.awaiting_confirm ? 'CALIBRATING — align, then ☰ / Confirm' : controls.deadman() ? '● CALIBRATING' : 'CALIBRATING — hold RB / Space to run')
      : motors.armed
        ? (driving ? '● MOTORS FOLLOWING SIM' : 'ARMED · holding — release, then hold RB to drive')
        : `Motors: ${motors.state}`;
    if (/^(Fault|Calibration failed)/.test(motors.state)) armPill.className = 'pill danger';
  }
  $('hud-blocked').hidden = !sim.blocked;
  $('hud-blocked').textContent = sim.blocked ? `Blocked: ${sim.blocked}` : '';
  const showMsg = sim.msg && performance.now() < sim.msgUntil;
  $('hud-msg').hidden = !showMsg;
  if (showMsg) $('hud-msg').textContent = sim.msg;
}

// ---------------------------------------------------------------- loop
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);
resize();

// 50 Hz targets, independent of the render rate; they double as the bridge heartbeat.
// Hidden tabs throttle timers, so the bridge stops the motors if this page is hidden.
setInterval(() => bridge.send({
  type: 'target', q: sim.q, deadman: !!((motors?.armed || calibrating()) && controls.deadman()),
}), 20);

let last = performance.now(), lastPanel = 0;
function frame(now) {
  requestAnimationFrame(frame); // keep running even if this frame throws
  try {
    step(now);
  } catch (error) {
    console.error(error);
    flash(`Sim error: ${error.message}`, 5);
  }
}
function step(now) {
  const dt = Math.min(0.1, (now - last) / 1000); // slow frames still move in real time
  last = now;
  input = controls.poll();
  const pr = input.pressed;
  if (pr.b) stopAll();
  if (pr.a) setMode(sim.mode === 'ik' ? 'joint' : 'ik');
  if (pr.x) goReady();
  if (pr.y) flipElbow();
  if (pr.lb) toggleDemo();
  if (pr.view) orbit.reset();
  if (pr.menu) menuButton();
  if (pr.up) sim.speed = Math.min(1, +(sim.speed + 0.1).toFixed(2));
  if (pr.down) sim.speed = Math.max(0.1, +(sim.speed - 0.1).toFixed(2));

  const sticks = Math.abs(input.lx) + Math.abs(input.ly) + Math.abs(input.ry) > 0;
  if (sticks && (sim.anim.length || sim.demo)) { sim.anim = []; sim.demo = null; }
  if (sim.anim.length) stepAnim(dt);
  else if (sim.demo) stepDemo(dt);
  else stepInput(dt);
  if (input.rx) orbit.rotate(-input.rx * dt * 1.6, 0);

  rig.set(sim.q);
  pushTrail(arm.fk(sim.q).tool);
  target.position.set(...sim.p);
  target.lookAt(camera.position);
  dropGeom.attributes.position.setXYZ(0, sim.p[0], sim.p[1], sim.p[2]);
  dropGeom.attributes.position.setXYZ(1, sim.p[0], sim.p[1], manifest.collision.table_z_mm);
  dropGeom.attributes.position.needsUpdate = true;
  drop.computeLineDistances();
  floorRing.position.set(sim.p[0], sim.p[1], manifest.collision.table_z_mm + 0.6);
  const fbq = motors?.feedback;
  if (ghost.root.visible && fbq?.every((f) => f && f.q_deg != null)) ghost.set(fbq.map((f) => rad(f.q_deg)));

  if (now - lastPanel > 100) {
    lastPanel = now;
    updatePanel();
  }
  renderer.render(scene, camera);
}
requestAnimationFrame(frame);
