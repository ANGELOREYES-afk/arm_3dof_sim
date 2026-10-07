import * as THREE from '../vendor/three.module.js';
import { createArm, rad, deg, LINKS } from './kinematics.js';
import { Controls } from './gamepad.js';
import { Orbit } from './orbit.js';
import { Bridge } from './bridge.js';

const $ = (id) => document.getElementById(id);
const [cfg, manifest] = await Promise.all([
  fetch('../config/arm.json').then((r) => r.json()),
  fetch('meshes/manifest.json').then((r) => r.json()),
]);
const arm = createArm(cfg, manifest);
const G = manifest.geometry;
const N = cfg.joints.length;

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
scene.fog = new THREE.Fog(0x12151a, 1600, 3400);
const camera = new THREE.PerspectiveCamera(38, 1, 5, 8000);
const orbit = new Orbit(camera, renderer.domElement, {
  target: [110, -20, 150], azimuth: rad(-58), elevation: rad(20), distance: 980,
});

scene.add(new THREE.HemisphereLight(0xe4ecff, 0x2b2622, 1.15));
const key = new THREE.DirectionalLight(0xffffff, 2.4);
key.position.set(-350, -650, 950);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
Object.assign(key.shadow.camera, { left: -520, right: 520, top: 520, bottom: -520, near: 200, far: 2400 });
key.shadow.bias = -0.0004;
scene.add(key);
const rim = new THREE.DirectionalLight(0xa9c8ff, 0.9);
rim.position.set(700, 450, 380);
scene.add(rim);

const floor = new THREE.Mesh(new THREE.CircleGeometry(1500, 96), new THREE.MeshStandardMaterial({ color: 0x1a1e25, roughness: 0.96 }));
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(1000, 40, 0x3a4350, 0x262c35);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.3;
scene.add(grid);

// ---------------------------------------------------------------- materials and meshes
const MAT = {
  motor: new THREE.MeshStandardMaterial({ color: 0x2e333b, metalness: 0.35, roughness: 0.45, flatShading: true }),
  frame: new THREE.MeshStandardMaterial({ color: 0x7d91ad, metalness: 0.12, roughness: 0.62, flatShading: true }),
  hardware: new THREE.MeshStandardMaterial({ color: 0xb4bac2, metalness: 0.4, roughness: 0.35, flatShading: true }),
  link: new THREE.MeshStandardMaterial({ color: 0xe27d3c, metalness: 0.1, roughness: 0.5 }),
  tool: new THREE.MeshStandardMaterial({ color: 0xf2c14e, metalness: 0.2, roughness: 0.35, emissive: 0x2e2000 }),
};
// Measured pose. Pushed back in depth so it only shows where it differs from the sim.
const GHOST = new THREE.MeshBasicMaterial({
  color: 0x46d39a, transparent: true, opacity: 0.22, depthWrite: false,
  polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 8,
});

async function loadLink(name) {
  const out = {};
  await Promise.all(Object.entries(manifest.parts[name]).map(async ([mat, info]) => {
    const buf = await fetch(`meshes/${info.file}`).then((r) => r.arrayBuffer());
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buf, 0, info.vertices * 3), 3));
    g.setIndex(new THREE.BufferAttribute(new Uint32Array(buf, info.vertices * 12, info.triangles * 3), 1));
    g.computeBoundingSphere();
    out[mat] = g;
  }));
  return out;
}

// Parametric plate: two hubs joined by tangent lines, with bolt circles and a lightening slot.
// Drawn in the (x, z) plane of its joint frame, extruded along -y from yHi.
function plateGeometry(len, r0, r1, yHi, thick, bolt0, bolt1) {
  const s = new THREE.Shape();
  const nv = (r0 - r1) / len, nu = Math.sqrt(1 - nv * nv);
  const aR = Math.atan2(nv, nu), aL = Math.PI - aR;
  s.moveTo(r0 * nu, r0 * nv);
  s.lineTo(r1 * nu, len + r1 * nv);
  s.absarc(0, len, r1, aR, aL, false);
  s.lineTo(-r0 * nu, r0 * nv);
  s.absarc(0, 0, r0, aL, aR + Math.PI * 2, false);
  const circle = (x, y, r) => { const p = new THREE.Path(); p.absarc(x, y, r, 0, Math.PI * 2, true); return p; };
  for (const [cz, bore, ring] of [[0, ...bolt0], [len, ...bolt1]]) {
    s.holes.push(circle(0, cz, bore));
    if (ring) for (let k = 0; k < 6; k++) s.holes.push(circle(ring * Math.sin((k * Math.PI) / 3), cz + ring * Math.cos((k * Math.PI) / 3), 1.7));
  }
  const v0 = r0 + 10, v1 = len - r1 - 10, w = Math.min(r0, r1) * 0.42;
  if (v1 - v0 > 2 * w) {
    const slot = new THREE.Path();
    slot.moveTo(w, v0); slot.lineTo(w, v1); slot.absarc(0, v1, w, 0, Math.PI, false);
    slot.lineTo(-w, v0); slot.absarc(0, v0, w, Math.PI, Math.PI * 2, false);
    s.holes.push(slot);
  }
  const bevel = 0.6;
  const g = new THREE.ExtrudeGeometry(s, { depth: thick - 2 * bevel, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, curveSegments: 40 });
  g.rotateX(Math.PI / 2); // shape (u, v) -> (x, z); extrusion -> -y
  g.translate(0, yHi - bevel, 0);
  return g;
}

const upperGeom = plateGeometry(arm.L2, LINKS.upper.r0, LINKS.upper.r1, G.upper_arm_y_mm[1], G.plate_mm, [6, 17], [8, 22]);
const foreGeom = plateGeometry(arm.L3, LINKS.fore.r0, LINKS.fore.r1, G.forearm_y_mm[1], G.plate_mm, [8, 17], [4.5, 0]);
const toolGeom = new THREE.CylinderGeometry(7, 7, 28, 32); // tool pin through the forearm tip, along y

const [base, turret, flange, elbowMotor] = await Promise.all(['base', 'turret', 'shoulder_flange', 'elbow_motor'].map(loadLink));

function buildRig(ghost) {
  const mesh = (geom, mat) => {
    const m = new THREE.Mesh(geom, ghost ? GHOST : MAT[mat]);
    m.castShadow = m.receiveShadow = !ghost;
    return m;
  };
  const addAll = (parent, link) => Object.entries(link).forEach(([mat, g]) => parent.add(mesh(g, mat)));
  const root = new THREE.Group();
  const j1 = new THREE.Group(), j2 = new THREE.Group(), j3 = new THREE.Group(), elbowMount = new THREE.Group();
  if (!ghost) addAll(root, base);
  root.add(j1);
  addAll(j1, turret);
  j2.position.set(0, 0, G.shoulder_height_mm);
  j1.add(j2);
  addAll(j2, flange);
  j2.add(mesh(upperGeom, 'link'));
  elbowMount.position.set(0, 0, arm.L2);
  j2.add(elbowMount);
  addAll(elbowMount, elbowMotor);
  j3.position.set(0, 0, arm.L2);
  j2.add(j3);
  j3.add(mesh(foreGeom, 'link'));
  const tool = mesh(toolGeom, 'tool');
  tool.position.set(0, G.tool_offset_y_mm, arm.L3);
  j3.add(tool);
  scene.add(root);
  return {
    root,
    set(q) { j1.rotation.z = q[0]; j2.rotation.y = q[1]; j3.rotation.y = q[2]; },
  };
}
const rig = buildRig(false);
const ghost = buildRig(true);
ghost.root.visible = false;

// Target marker: sphere, drop line and floor ring so its height reads in 3D.
const target = new THREE.Group();
const targetBall = new THREE.Mesh(new THREE.SphereGeometry(10, 32, 16), new THREE.MeshBasicMaterial({ color: 0x3cc3dc, transparent: true, opacity: 0.35, depthWrite: false }));
target.add(targetBall);
target.add(new THREE.Mesh(new THREE.TorusGeometry(13, 0.8, 8, 48), new THREE.MeshBasicMaterial({ color: 0x3cc3dc })));
scene.add(target);
const dropGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const drop = new THREE.Line(dropGeom, new THREE.LineDashedMaterial({ color: 0x3cc3dc, dashSize: 6, gapSize: 5, transparent: true, opacity: 0.6 }));
scene.add(drop);
const floorRing = new THREE.Mesh(new THREE.RingGeometry(9, 12, 40), new THREE.MeshBasicMaterial({ color: 0x3cc3dc, transparent: true, opacity: 0.55 }));
floorRing.position.z = 0.6;
scene.add(floorRing);

const TRAIL = 480;
const trailPos = new Float32Array(TRAIL * 3);
const trailGeom = new THREE.BufferGeometry();
trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
trailGeom.setDrawRange(0, 0);
scene.add(new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: 0xf2c14e, transparent: true, opacity: 0.75 })));
let trailN = 0;
const clearTrail = () => { trailN = 0; trailGeom.setDrawRange(0, 0); };
function pushTrail(p) {
  const last = trailN ? trailPos.subarray((trailN - 1) * 3, trailN * 3) : null;
  if (last && Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]) < 1.5) return;
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
  anim: null, // { goal, label } joint-space move
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

function moveTo(goal, label) {
  sim.demo = null;
  sim.anim = { goal, label };
}

function goReady() { moveTo(cfg.ready_pose_deg.map(rad), 'ready pose'); }

function flipElbow() {
  const alt = arm.solutions(sim.p)
    .map((s) => s.map((a, i) => a + 2 * Math.PI * Math.round((sim.q[i] - a) / (2 * Math.PI))))
    .filter((s) => arm.valid(s) && Math.sign(s[2]) !== Math.sign(sim.q[2]))
    .sort((a, b) => Math.abs(a[0] - sim.q[0]) - Math.abs(b[0] - sim.q[0]))[0];
  if (alt) moveTo(alt, 'elbow flip');
  else flash('No valid flipped-elbow solution for this target');
}

function toggleDemo() {
  sim.anim = null;
  sim.demo = sim.demo ? null : { t: 0 };
  if (sim.demo) {
    sim.mode = 'ik';
    clearTrail();
  }
}

// Figure-8 in a vertical plane in front of the arm, in its tool plane.
const demoPath = (t) => {
  const w = (2 * Math.PI) / 9;
  return [215, G.tool_offset_y_mm + 115 * Math.sin(w * t), 185 + 55 * Math.sin(2 * w * t)];
};

function stepAnim(dt) {
  const dq = sim.anim.goal.map((g, i) => g - sim.q[i]);
  const t = Math.max(...dq.map((d, i) => Math.abs(d) / (arm.vmax[i] * sim.speed)));
  if (t < 1e-4) { sim.anim = null; return; }
  const qn = sim.q.map((v, i) => v + dq[i] * Math.min(1, dt / t)); // joints arrive together
  if (!arm.valid(qn)) {
    flash(`${sim.anim.label} blocked: ${arm.collision(qn) || 'joint limit'}`);
    sim.anim = null;
    return;
  }
  sim.q = qn;
  sim.p = arm.fk(qn).tool;
}

function stepDemo(dt) {
  sim.demo.t += dt * sim.speed / 0.6;
  const goal = demoPath(sim.demo.t);
  const v = 260 * sim.speed * dt;
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
    const v = 220 * k * dt;
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
const events = [];
const bridge = new Bridge((s) => {
  motors = s;
  if (s?.armed && !wasArmed && s.q_cmd) {
    // Armed at the motors' measured pose: start the sim there so nothing jumps.
    sim.q = s.q_cmd.map(rad);
    sim.p = arm.fk(sim.q).tool;
    sim.anim = sim.demo = null;
    clearTrail();
    flash('Armed — sim moved to the measured pose. Hold RB to drive.', 4);
  }
  wasArmed = !!s?.armed;
  if (s?.events?.length) {
    events.push(...s.events);
    events.splice(0, Math.max(0, events.length - 60));
    renderEvents();
  }
});

function armOrCheck() {
  const st = motors?.state || '';
  if (!motors?.connected) return flash('Connect the motors in the panel first');
  if (!$('m-supported').checked) return flash('Tick “The arm is supported” in the panel first');
  if (st.startsWith('Stopped') || st.startsWith('Fault')) return bridge.cmd('check');
  if (st === 'Ready · Stop') {
    if (!motors.zeroed) return flash('Set home first (arm at the calibration pose)');
    return bridge.cmd('arm', { supported: true });
  }
}

function stopAll() {
  bridge.stop();
  sim.demo = sim.anim = null;
  if (motors?.armed) flash('STOP sent — motors disarmed', 3);
}

// ---------------------------------------------------------------- panel
const jointRows = cfg.joints.map((j, i) => {
  const el = document.createElement('div');
  el.className = 'joint';
  const [lo, hi] = [j.min_deg, j.max_deg];
  el.innerHTML = `<span class="name">${j.name}</span>
    <div class="track"><i class="zero" style="left:${((0 - lo) / (hi - lo)) * 100}%"></i><i class="meas" hidden></i><i class="tgt"></i></div>
    <span class="val">—</span><div class="sub"><span>${lo}° … ${hi}°</span><span class="meas-txt"></span><span class="tau"></span></div>`;
  $('joints').append(el);
  const pct = (a) => `${Math.min(100, Math.max(0, ((a - lo) / (hi - lo)) * 100))}%`;
  return {
    set(tgt, meas, tau) {
      el.querySelector('.tgt').style.left = pct(tgt);
      el.querySelector('.val').textContent = `${tgt.toFixed(1)}°`;
      const m = el.querySelector('.meas');
      m.hidden = meas == null;
      if (meas != null) m.style.left = pct(meas);
      el.querySelector('.meas-txt').textContent = meas == null ? '' : `meas ${meas.toFixed(1)}°`;
      el.querySelector('.tau').textContent = i ? `τg ${tau.toFixed(2)} Nm` : '';
    },
  };
});

function setMode(m) {
  sim.mode = m;
  if (m === 'joint') sim.demo = null;
}
document.querySelectorAll('#mode-seg button').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
$('speed').addEventListener('input', (e) => { sim.speed = +e.target.value; });
$('btn-ready').onclick = goReady;
$('btn-flip').onclick = flipElbow;
$('btn-demo').onclick = toggleDemo;
$('btn-cal').onclick = () => moveTo(cfg.calibration_pose_deg.map(rad), 'calibration pose');
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
$('m-supported').onchange = (e) => { if (!e.target.checked && motors?.armed) stopAll(); };

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
  $('ik-state').textContent = sim.anim ? `→ ${sim.anim.label}` : sim.demo ? 'figure-8' : sim.mode === 'ik' ? 'IK' : 'joint';

  // Motors panel
  const online = bridge.online && !!motors;
  $('bridge-offline').hidden = online;
  $('bridge-online').hidden = !online;
  $('bridge-link').textContent = online ? (motors.controller ? 'bridge · in control' : 'bridge · view only') : 'offline';
  const badge = $('source-badge');
  badge.className = 'badge' + (online && motors.connected ? (motors.demo ? ' demo' : ' real') : '');
  badge.textContent = online && motors.connected ? (motors.demo ? 'DEMO MOTORS' : 'REAL MOTORS') : 'SIM ONLY';
  ghost.root.visible = !!(online && fb?.every((f) => f && f.q_deg != null));
  if (online) {
    renderPorts();
    const st = motors.state;
    $('observer-note').hidden = !motors.observer;
    $('m-state').textContent = st;
    $('m-reason').textContent = motors.reason;
    $('m-state-box').className = 'state' + (motors.armed ? ' armed' : st.startsWith('Fault') ? ' fault' : st.startsWith('Ready') ? ' ready' : '');
    const supported = $('m-supported').checked;
    const ctl = !motors.observer;
    $('m-source').disabled = $('m-port').disabled = $('m-baud').disabled = motors.connected;
    $('m-connect').disabled = !ctl || motors.connected;
    $('m-disconnect').disabled = !ctl || !motors.connected;
    $('m-check').disabled = !ctl || !motors.connected || motors.armed;
    $('m-ack').disabled = !ctl || !st.startsWith('Timeout');
    $('m-home').disabled = !ctl || st !== 'Ready · Stop' || !supported;
    $('m-arm').disabled = !ctl || st !== 'Ready · Stop' || !motors.zeroed || !supported;
    $('k-hz').textContent = motors.connected ? `${motors.loop_hz.toFixed(0)} Hz` : '—';
    $('k-pk').textContent = `${motors.tx}/${motors.rx}`;
    $('k-crc').textContent = motors.crc_errors;
    $('k-to').textContent = motors.reply_timeouts;
    $('m-table').innerHTML = cfg.joints.map((j, i) => {
      const f = fb?.[i];
      const cell = (v, d = 1) => (v == null ? '—' : (+v).toFixed(d));
      return `<tr><td>${j.name.split(' ')[0]}</td><td>${j.id}</td><td>${cell(f?.q_deg)}</td><td>${cell(f?.torque_nm, 2)}</td>
        <td>${cell(f?.voltage, 1)}</td><td>${f ? `${f.driver_c}/${f.winding_c}` : '—'}</td><td>${f ? f.age_ms + 'ms' : '—'}</td></tr>`;
    }).join('');
  }

  // HUD
  $('hud-mode').textContent = sim.demo ? 'Figure-8 demo (IK)' : sim.mode === 'ik' ? 'IK · Cartesian' : 'Joint mode';
  const armPill = $('hud-arm');
  armPill.hidden = !(online && motors.connected);
  if (online && motors.connected) {
    const driving = motors.armed && motors.moving;
    armPill.className = 'pill ' + (motors.armed ? (driving ? 'live' : 'warn') : '');
    armPill.textContent = motors.armed
      ? (driving ? '● MOTORS FOLLOWING SIM' : 'ARMED · holding — hold RB / Space to drive')
      : motors.state.startsWith('Fault') ? `FAULT — ${motors.state}` : `Motors: ${motors.state}`;
    if (motors.state.startsWith('Fault')) armPill.className = 'pill danger';
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
setInterval(() => bridge.send({ type: 'target', q: sim.q, deadman: !!(motors?.armed && controls.deadman()) }), 20);

let last = performance.now(), lastPanel = 0;
function frame(now) {
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
  if (pr.menu) armOrCheck();
  if (pr.up) sim.speed = Math.min(1, +(sim.speed + 0.1).toFixed(2));
  if (pr.down) sim.speed = Math.max(0.1, +(sim.speed - 0.1).toFixed(2));

  const sticks = Math.abs(input.lx) + Math.abs(input.ly) + Math.abs(input.ry) > 0;
  if (sticks && (sim.anim || sim.demo)) { sim.anim = sim.demo = null; }
  if (sim.anim) stepAnim(dt);
  else if (sim.demo) stepDemo(dt);
  else stepInput(dt);
  if (sim.mode === 'ik' && input.rx) orbit.rotate(-input.rx * dt * 1.6, 0);
  if (sim.mode === 'joint' && input.rx) orbit.rotate(-input.rx * dt * 1.6, 0);

  rig.set(sim.q);
  const tool = arm.fk(sim.q).tool;
  pushTrail(tool);
  target.position.set(...sim.p);
  target.lookAt(camera.position);
  dropGeom.attributes.position.setXYZ(0, sim.p[0], sim.p[1], sim.p[2]);
  dropGeom.attributes.position.setXYZ(1, sim.p[0], sim.p[1], 0);
  dropGeom.attributes.position.needsUpdate = true;
  drop.computeLineDistances();
  floorRing.position.set(sim.p[0], sim.p[1], 0.6);
  if (ghost.root.visible) ghost.set(motors.feedback.map((f) => rad(f.q_deg)));

  if (now - lastPanel > 100) {
    lastPanel = now;
    updatePanel();
  }
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
