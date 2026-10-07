// WebSocket client for bridge/arm_bridge.py. Reconnects quietly; the sim works without it.

export class Bridge {
  constructor(onState) {
    this.onState = onState;
    this.ws = null;
    this.online = false;
    this.state = null;
    this.ports = [];
    this.url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
    this.connect();
  }

  connect() {
    let ws;
    try {
      ws = new WebSocket(this.url);
    } catch {
      return;
    }
    this.ws = ws;
    ws.onopen = () => { this.online = true; };
    ws.onmessage = (e) => {
      const msg = JSON.parse(e.data);
      if (msg.type === 'hello' || msg.type === 'ports') this.ports = msg.ports || [];
      if (msg.type === 'state') {
        this.state = msg;
        this.onState(msg);
      }
      if (msg.type === 'error') console.warn('bridge:', msg.message);
    };
    ws.onclose = () => {
      this.online = false;
      this.state = null;
      this.onState(null);
      setTimeout(() => this.connect(), 2000);
    };
  }

  send(obj) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  cmd(action, extra = {}) {
    this.send({ type: 'cmd', action, ...extra });
  }

  stop() {
    this.send({ type: 'stop' });
  }
}
