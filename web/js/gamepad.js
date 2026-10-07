// Xbox controller via the browser Gamepad API ("standard" mapping), with a keyboard fallback.
// Browsers expose a pad only after a button press on that page, so prompt for one.

const BUTTONS = ['a', 'b', 'x', 'y', 'lb', 'rb', 'lt', 'rt', 'view', 'menu', 'ls', 'rs', 'up', 'down', 'left', 'right', 'guide'];
const DEADZONE = 0.14;

// Radial dead zone, rescaled, with a squared response for fine control near centre.
function stick(x, y) {
  const m = Math.hypot(x, y);
  if (m < DEADZONE) return [0, 0];
  const t = Math.min(1, (m - DEADZONE) / (1 - DEADZONE));
  const k = (t * t) / m;
  return [x * k, y * k];
}

// Keyboard stands in for the pad: same actions, digital axes.
const KEYMAP = {
  KeyW: ['ly', -1], KeyS: ['ly', 1], KeyA: ['lx', -1], KeyD: ['lx', 1],
  KeyR: ['ry', -1], KeyF: ['ry', 1], KeyQ: ['rx', -1], KeyE: ['rx', 1],
};
const KEYBUTTONS = {
  Space: 'rb', ShiftLeft: 'lt', ShiftRight: 'lt', Escape: 'b', KeyH: 'x', KeyG: 'y', KeyM: 'a', KeyP: 'lb',
  KeyC: 'view', ArrowUp: 'up', ArrowDown: 'down',
};

export class Controls {
  constructor() {
    this.prev = {};
    this.keys = new Set();
    const typing = (e) => /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName);
    addEventListener('keydown', (e) => {
      if (typing(e) && e.code !== 'Escape') return;
      if (e.code in KEYMAP || e.code in KEYBUTTONS) e.preventDefault();
      this.keys.add(e.code);
    });
    addEventListener('keyup', (e) => this.keys.delete(e.code));
    addEventListener('blur', () => this.keys.clear());
  }

  // Read fresh at send time so a slow render frame never holds a stale "pressed".
  deadman() {
    return !!this.pad()?.buttons[5]?.pressed || this.keys.has('Space');
  }

  pad() {
    const pads = [...(navigator.getGamepads?.() || [])].filter(Boolean);
    return pads.find((p) => p.mapping === 'standard') || pads[0] || null;
  }

  // One snapshot per frame: analog axes, held buttons, and rising edges ("pressed").
  poll() {
    const pad = this.pad();
    const held = {};
    let lx = 0, ly = 0, rx = 0, ry = 0, lt = 0, rt = 0;
    if (pad) {
      [lx, ly] = stick(pad.axes[0] || 0, pad.axes[1] || 0);
      [rx, ry] = stick(pad.axes[2] || 0, pad.axes[3] || 0);
      BUTTONS.forEach((name, i) => { held[name] = !!pad.buttons[i]?.pressed; });
      lt = pad.buttons[6]?.value || 0;
      rt = pad.buttons[7]?.value || 0;
    }
    const axes = { lx, ly, rx, ry };
    for (const code of this.keys) {
      if (KEYMAP[code]) axes[KEYMAP[code][0]] = KEYMAP[code][1];
      if (KEYBUTTONS[code]) held[KEYBUTTONS[code]] = true;
    }
    if (held.lt && !lt) lt = 1;
    const pressed = {};
    for (const name of BUTTONS) {
      pressed[name] = !!held[name] && !this.prev[name];
    }
    this.prev = held;
    return {
      connected: !!pad,
      name: pad ? pad.id.replace(/\s*\(.*$/, '') : '',
      keyboard: this.keys.size > 0,
      ...axes, lt, rt, held, pressed,
    };
  }
}
