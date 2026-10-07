"""Serve the 3-DOF simulator and bridge it to IM6014 motors over the im6014Development codec.

    python3 bridge/arm_bridge.py                  # then open http://127.0.0.1:8765
    python3 bridge/arm_bridge.py --list-ports

Standard library + pyserial only. Motors stay disconnected until the page asks to connect,
and Demo uses that repo's packet-level simulator without touching any serial device.

Only one browser tab controls the arm; others watch. Any tab can Stop. The page's targets act
only while armed and while the deadman is held. Losing the controlling tab stops the motors.
"""
import argparse
import base64
import hashlib
import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import struct
import sys
import threading
import time
from urllib.parse import urlparse

from arm_session import ArmSession
import im6014

ROOT = Path(__file__).resolve().parent.parent
SERVED = (ROOT / 'web', ROOT / 'config')
GUID = b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
STATE_HZ = 30
CONTROL_ACTIONS = {'connect', 'disconnect', 'check', 'ack', 'home', 'arm'}
mimetypes.add_type('text/javascript', '.js')
mimetypes.add_type('application/octet-stream', '.bin')


class WebSocket:
    """Minimal RFC 6455 server side: text frames, ping/pong, close."""

    def __init__(self, sock, rfile):
        self.sock, self.rfile = sock, rfile
        self.wlock = threading.Lock()
        self.closed = False

    def _read(self, n):
        data = self.rfile.read(n)
        if len(data) < n:
            raise ConnectionError('socket closed')
        return data

    def _frame(self, opcode, data):
        n = len(data)
        head = struct.pack('!BB', 0x80 | opcode, n) if n < 126 else \
            struct.pack('!BBH', 0x80 | opcode, 126, n) if n < 65536 else struct.pack('!BBQ', 0x80 | opcode, 127, n)
        with self.wlock:
            self.sock.sendall(head + data)

    def send(self, obj):
        self._frame(0x1, json.dumps(obj, separators=(',', ':')).encode())

    def recv(self):
        """Next text message, or None when the client closes."""
        parts = []
        while True:
            b1, b2 = self._read(2)
            n = b2 & 0x7f
            if n == 126:
                n = struct.unpack('!H', self._read(2))[0]
            elif n == 127:
                n = struct.unpack('!Q', self._read(8))[0]
            if n > 1 << 16 or not b2 & 0x80:
                raise ConnectionError('oversized or unmasked client frame')
            mask = self._read(4)
            raw = self._read(n)
            data = (int.from_bytes(raw, 'big') ^ int.from_bytes((mask * (n // 4 + 1))[:n], 'big')).to_bytes(n, 'big')
            opcode = b1 & 0x0f
            if opcode == 0x8:
                self._frame(0x8, b'')
                return None
            if opcode == 0x9:
                self._frame(0xA, data)
                continue
            if opcode == 0xA:
                continue
            parts.append(data)
            if b1 & 0x80:
                return b''.join(parts).decode()


class Hub:
    def __init__(self, session, ports):
        self.session = session
        self.list_ports = ports
        self.lock = threading.Lock()
        self.controller = None

    def claim(self, ws):
        with self.lock:
            if self.controller is None:
                self.controller = ws
            return self.controller is ws

    def release(self, ws):
        with self.lock:
            if self.controller is not ws:
                return
            self.controller = None
        if self.session.snapshot()['armed']:
            self.session.request('stop')
            self.session._event('Controlling page disconnected; Stop requested.')


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [dict(device=p.device, description=p.description, serial=p.serial_number or '') for p in lp.comports()]
    except Exception as error:
        return [dict(device='', description=f'Port listing failed: {error}', serial='')]


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    hub = None
    hosts = set()

    def log_message(self, fmt, *args):
        if not str(args[1] if len(args) > 1 else '').startswith(('2', '3', '101')):
            super().log_message(fmt, *args)

    def _plain(self, code, text):
        body = text.encode()
        self.send_response(code)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        # Reject other hostnames (DNS rebinding) and cross-site pages (WebSocket hijacking).
        if self.headers.get('Host') not in self.hosts:
            return self._plain(403, 'Host not allowed')
        origin = self.headers.get('Origin')
        if origin and urlparse(origin).netloc not in self.hosts:
            return self._plain(403, 'Origin not allowed')
        path = urlparse(self.path).path
        if path == '/ws':
            return self._websocket()
        if path in ('/', '/web'):
            self.send_response(302)
            self.send_header('Location', '/web/')
            self.send_header('Content-Length', '0')
            return self.end_headers()
        file = (ROOT / path.lstrip('/')).resolve()
        if file.is_dir():
            file = file / 'index.html'
        if not any(file.is_relative_to(base) for base in SERVED) or not file.is_file():
            return self._plain(404, 'Not found')
        body = file.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', mimetypes.guess_type(file.name)[0] or 'application/octet-stream')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _websocket(self):
        key = self.headers.get('Sec-WebSocket-Key')
        if not key or 'websocket' not in self.headers.get('Upgrade', '').lower():
            return self._plain(400, 'WebSocket upgrade required')
        self.send_response(101, 'Switching Protocols')
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', base64.b64encode(hashlib.sha1(key.encode() + GUID).digest()).decode())
        self.end_headers()
        self.close_connection = True
        ws = WebSocket(self.connection, self.rfile)
        hub, session = self.hub, self.hub.session
        try:
            ws.send(dict(type='hello', config=session.cfg, ports=hub.list_ports()))
            threading.Thread(target=self._publish, args=(ws,), daemon=True).start()
            while (text := ws.recv()) is not None:
                try:
                    msg = json.loads(text)
                    kind = msg.get('type')
                    if kind == 'stop' or kind == 'cmd' and msg.get('action') == 'stop':
                        session.request('stop')  # any page may stop
                        continue
                    if not hub.claim(ws):
                        continue
                    if kind == 'target':
                        session.set_target(msg['q'], msg.get('deadman', False))
                    elif kind == 'ping':
                        session.ping()
                    elif kind == 'ports':
                        ws.send(dict(type='ports', ports=hub.list_ports()))
                    elif kind == 'cmd' and msg.get('action') in CONTROL_ACTIONS:
                        session.ping()
                        args = {k: v for k, v in msg.items() if k not in ('type', 'action')}
                        session.request(msg['action'], **args)
                except (ValueError, KeyError, TypeError) as error:
                    ws.send(dict(type='error', message=str(error)))
        except (ConnectionError, OSError):
            pass
        finally:
            ws.closed = True
            hub.release(ws)

    def _publish(self, ws):
        seq = 0
        while not ws.closed:
            try:
                snap = self.hub.session.snapshot(since=seq)
                if snap['events']:
                    seq = snap['events'][-1][0]
                snap['events'] = [e[1] for e in snap['events']]
                snap.update(type='state', controller=self.hub.controller is ws,
                            observer=self.hub.controller not in (None, ws))
                ws.send(snap)
            except OSError:
                break
            time.sleep(1 / STATE_HZ)


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        if not isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):  # page closed mid-request
            super().handle_error(request, client_address)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='bind address (default: this machine only)')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--config', default=str(ROOT / 'config' / 'arm.json'))
    ap.add_argument('--im6014', help='path to a clone of im6014Development')
    ap.add_argument('--list-ports', action='store_true', help='print serial ports and exit')
    args = ap.parse_args()
    if args.list_ports:
        for p in list_ports():
            print(f"{p['device']:60s} {p['description']} {p['serial']}")
        return
    codec, sim = im6014.load(args.im6014)
    cfg = json.loads(Path(args.config).read_text())
    session = ArmSession(cfg, codec, sim)
    Handler.hub = Hub(session, list_ports)
    Handler.hosts = {f'{h}:{args.port}' for h in ('127.0.0.1', 'localhost', '[::1]', args.host)}
    server = Server((args.host, args.port), Handler)
    print(f'IM6014 codec: {Path(codec.__file__).parent}')
    print(f'Simulator:    http://127.0.0.1:{args.port}/   (motors stay disconnected until you connect in the page)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        server.server_close()


if __name__ == '__main__':
    main()
