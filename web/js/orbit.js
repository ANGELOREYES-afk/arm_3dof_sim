// Z-up orbit camera: left-drag rotates, right-drag (or shift-drag) pans, wheel zooms.
import { Vector3 } from '../vendor/three.module.js';

export class Orbit {
  constructor(camera, dom, home) {
    this.camera = camera;
    this.home = home;
    this.reset();
    let drag = null;
    dom.addEventListener('contextmenu', (e) => e.preventDefault());
    dom.addEventListener('pointerdown', (e) => {
      drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey };
      dom.setPointerCapture(e.pointerId);
    });
    dom.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX;
      drag.y = e.clientY;
      if (drag.pan) this.pan(-dx, dy);
      else this.rotate(-dx * 0.006, dy * 0.006);
    });
    dom.addEventListener('pointerup', () => { drag = null; });
    dom.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.zoom(Math.exp(e.deltaY * 0.0012));
    }, { passive: false });
  }

  reset() {
    this.target = new Vector3(...this.home.target);
    this.azimuth = this.home.azimuth;
    this.elevation = this.home.elevation;
    this.distance = this.home.distance;
    this.update();
  }

  rotate(dAz, dEl) {
    this.azimuth += dAz;
    this.elevation = Math.min(1.5, Math.max(-0.2, this.elevation + dEl));
    this.update();
  }

  pan(dx, dy) {
    const s = this.distance * 0.0016;
    const right = new Vector3(-Math.sin(this.azimuth), Math.cos(this.azimuth), 0);
    const up = new Vector3(0, 0, 1);
    this.target.addScaledVector(right, dx * s).addScaledVector(up, dy * s);
    this.update();
  }

  zoom(f) {
    this.distance = Math.min(3000, Math.max(150, this.distance * f));
    this.update();
  }

  // Unit vectors on the floor plane: away from the camera, and to its right.
  groundAxes() {
    return { forward: [-Math.cos(this.azimuth), -Math.sin(this.azimuth)], right: [-Math.sin(this.azimuth), Math.cos(this.azimuth)] };
  }

  update() {
    const c = Math.cos(this.elevation);
    this.camera.position.set(
      this.target.x + this.distance * c * Math.cos(this.azimuth),
      this.target.y + this.distance * c * Math.sin(this.azimuth),
      this.target.z + this.distance * Math.sin(this.elevation),
    );
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(this.target);
  }
}
