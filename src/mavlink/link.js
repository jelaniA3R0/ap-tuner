// MAVLink link: TCP client / UDP client / UDP listener / serial (through a PowerShell bridge, no
// native modules), heartbeat, parameter protocol, message intervals. Emits decoded messages.
'use strict';
const net = require('net');
const dgram = require('dgram');
const path = require('path');
const { spawn, execFileSync } = require('child_process');
const EventEmitter = require('events');
const { Parser, Encoder, ENUMS } = require('./mavlink');

class Link extends EventEmitter {
  constructor() {
    super();
    this.parser = new Parser();
    this.enc = new Encoder(255, 190);       // GCS system id 255 = SYSID_MYGCS default (RC overrides are only accepted from it)
    this.target = { sysid: 1, compid: 1 };
    this.connected = false;
    this.transport = null;
    this.params = new Map();                // name -> {value, type, index}
    this.pendingParams = new Map();         // name -> [{resolve, reject, timer}]
    this.heartbeat = null; this.lastHeartbeatAt = 0;
    this.stats = { rx: 0, tx: 0, bytes: 0 };
    this.msgCounts = {}; this._rateWin = { t: Date.now(), counts: {} }; this.msgRates = {};
    this._hbTimer = null;
    this.vehicle = null;                    // {type, autopilot, mode, armed, modeName}
    this.generation = 0;                    // increments on every connect (used to re-request streams)
    this.description = '';
    this.opts = null;
  }

  /** opts: {kind:'tcp'|'udp'|'udpin'|'serial', host, port, baud} */
  connect(opts) {
    this.disconnect();
    this.opts = opts; this.generation++; this.serialOpened = false;
    this.parser = new Parser();
    this.lastHeartbeatAt = 0; this.heartbeat = null; this.vehicle = null;
    if (opts.kind === 'tcp') {
      this.description = `TCP ${opts.host || '127.0.0.1'}:${opts.port || 5760}`;
      const sock = net.createConnection({ host: opts.host || '127.0.0.1', port: opts.port || 5760 });
      sock.setNoDelay(true);
      sock.on('connect', () => { this.connected = true; this.emit('status', 'tcp connected'); this._start(); });
      sock.on('data', d => this._rx(d));
      sock.on('error', e => this.emit('status', 'tcp error: ' + e.message));
      sock.on('close', () => { this.connected = false; this.emit('status', 'tcp closed'); this._stop(); });
      this.transport = { send: b => { if (sock.writable) sock.write(b); }, close: () => sock.destroy() };
    } else if (opts.kind === 'serial') {
      this.description = `${opts.port} @ ${opts.baud || 115200}`;
      const script = path.join(__dirname, 'serialbridge.ps1');
      const child = spawn('powershell.exe', ['-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-File', script, opts.port, String(opts.baud || 115200)], { stdio: ['pipe', 'pipe', 'pipe'], windowsHide: true });
      child.stdout.on('data', d => this._rx(d));
      child.stderr.on('data', d => {
        for (const line of d.toString().split(/\r?\n/).map(x => x.trim()).filter(Boolean)) {
          if (/^open /.test(line)) { if (this.opts === opts) this.serialOpened = true; this.emit('status', 'serial port opened ' + this.description); continue; }
          this.emit('status', 'serial: ' + line);
        }
      });
      child.on('exit', code => { this.connected = false; this.emit('status', 'serial closed' + (code ? ' (code ' + code + ')' : '')); this._stop(); });
      child.on('error', e => this.emit('status', 'serial error: ' + e.message));
      this.transport = { send: b => { try { child.stdin.write(b); } catch (e) { /* closed */ } }, close: () => { try { child.stdin.end(); } catch (e) { /* ignore */ } setTimeout(() => { try { child.kill(); } catch (e) { /* ignore */ } }, 300); } };
      // the bridge prints nothing until bytes arrive; treat the port as open once the process survives its start
      setTimeout(() => { if (!this.connected && child.exitCode === null && this.transport && this.opts === opts) { this.connected = true; this.emit('status', 'serial open ' + this.description); this._start(); } }, 800);
    } else {
      const sock = dgram.createSocket('udp4');
      let remote = null;
      if (opts.kind === 'udpin') {
        this.description = `UDP listen ${opts.port || 14550}`;
        sock.bind(opts.port || 14550, opts.host || '0.0.0.0', () => { this.connected = true; this.emit('status', `udp listening on ${opts.port || 14550}`); this._start(); });
      } else {
        this.description = `UDP ${opts.host || '127.0.0.1'}:${opts.port || 14550}`;
        remote = { address: opts.host || '127.0.0.1', port: opts.port || 14550 };
        sock.bind(0, () => { this.connected = true; this.emit('status', `udp client -> ${remote.address}:${remote.port}`); this._start(); });
      }
      sock.on('message', (d, rinfo) => { if (!remote || opts.kind === 'udpin') remote = { address: rinfo.address, port: rinfo.port }; this._rx(d); });
      sock.on('error', e => this.emit('status', 'udp error: ' + e.message));
      this.transport = { send: b => { if (remote) sock.send(b, remote.port, remote.address); }, close: () => { try { sock.close(); } catch (e) { /* ignore */ } } };
    }
  }

  disconnect() {
    this._stop();
    if (this.transport) { try { this.transport.close(); } catch (e) { /* ignore */ } }
    this.transport = null; this.connected = false; this.opts = null;
  }

  /** a connection on a given transport object {send(buf), close()} (tests: a fake vehicle); bytes come in via feed() */
  attachTransport(transport, description = 'test transport') {
    this.disconnect();
    this.opts = { kind: 'test' }; this.generation++; this.serialOpened = false;
    this.parser = new Parser();
    this.lastHeartbeatAt = 0; this.heartbeat = null; this.vehicle = null;
    this.description = description; this.transport = transport; this.connected = true;
    this._start();
  }
  feed(data) { this._rx(data); }

  _start() {
    if (this._hbTimer) clearInterval(this._hbTimer);
    this._hbTimer = setInterval(() => this.send('HEARTBEAT', { type: 6, autopilot: 8, base_mode: 0, custom_mode: 0, system_status: 4, mavlink_version: 3 }), 1000);
    this.send('HEARTBEAT', { type: 6, autopilot: 8, base_mode: 0, custom_mode: 0, system_status: 4, mavlink_version: 3 });
  }
  _stop() { if (this._hbTimer) clearInterval(this._hbTimer); this._hbTimer = null; }

  _rx(data) {
    this.stats.bytes += data.length;
    const msgs = this.parser.feed(data);
    const now = Date.now();
    if (now - this._rateWin.t >= 2000) { const dt = (now - this._rateWin.t) / 1000; this.msgRates = {}; for (const [k, v] of Object.entries(this._rateWin.counts)) this.msgRates[k] = +(v / dt).toFixed(1); this._rateWin = { t: now, counts: {} }; }
    for (const m of msgs) {
      this.stats.rx++; this._rateWin.counts[m.name] = (this._rateWin.counts[m.name] || 0) + 1;
      m.ts = now;
      if (m.name === 'HEARTBEAT') {
        if (m.fields.type === 6 || m.compid !== 1) continue; // ignore GCS / companion heartbeats
        this.target = { sysid: m.sysid, compid: m.compid };
        this.lastHeartbeatAt = now;
        this.heartbeat = m.fields;
        const armed = !!(m.fields.base_mode & 128);
        const type = m.fields.type;
        this.vehicle = { type, autopilot: m.fields.autopilot, mode: m.fields.custom_mode, armed, modeName: modeName(type, m.fields.custom_mode), typeName: ENUMS.MAV_TYPE?.[type] || String(type) };
        this.emit('heartbeat', this.vehicle);
      } else if (m.name === 'PARAM_VALUE') {
        const f = m.fields;
        this.params.set(f.param_id, { value: f.param_value, type: f.param_type, index: f.param_index, count: f.param_count, ts: now });
        const waiters = this.pendingParams.get(f.param_id);
        if (waiters) { this.pendingParams.delete(f.param_id); for (const w of waiters) { clearTimeout(w.timer); w.resolve(f.param_value); } }
        this.emit('param', f.param_id, f.param_value);
      }
      this.emit('message', m);
      this.emit(m.name, m.fields, m);
    }
  }

  send(name, fields) {
    if (!this.transport) return;
    const buf = this.enc.pack(name, fields);
    this.transport.send(buf); this.stats.tx++;
  }

  /** request one parameter by name; resolves to value (rejects on timeout: the parameter does not exist or the link is slow) */
  requestParam(name, timeoutMs = 1500, retries = 4) {
    return new Promise((resolve, reject) => {
      const attempt = (n) => {
        const timer = setTimeout(() => {
          const ws = this.pendingParams.get(name) || [];
          this.pendingParams.set(name, ws.filter(w => w.timer !== timer));
          if (n < retries) attempt(n + 1); else { this.pendingParams.delete(name); reject(new Error('param timeout ' + name)); }
        }, timeoutMs);
        const ws = this.pendingParams.get(name) || []; ws.push({ resolve, reject, timer }); this.pendingParams.set(name, ws);
        this.send('PARAM_REQUEST_READ', { target_system: this.target.sysid, target_component: this.target.compid, param_id: name, param_index: -1 });
      };
      attempt(0);
    });
  }

  /** fetch many params (small concurrency to be gentle on telemetry radios) */
  async requestParams(names, concurrency = 3, onProgress = null) {
    const out = {}; let i = 0, done = 0;
    const worker = async () => {
      while (i < names.length) {
        const n = names[i++];
        try { out[n] = await this.requestParam(n); } catch (e) { out[n] = undefined; }
        done++; if (onProgress) onProgress(done, names.length);
      }
    };
    await Promise.all(Array.from({ length: concurrency }, worker));
    return out;
  }

  /**
   * set a parameter and wait for the echo; resolves the value the vehicle reports.
   * keepGoing (optional): checked before every PARAM_SET, retries included; when it returns false nothing more is
   * sent and the promise rejects with code 'cancelled' (an earlier attempt may still have reached the vehicle)
   */
  setParam(name, value, timeoutMs = 1500, retries = 3, keepGoing = null) {
    const known = this.params.get(name);
    const type = known ? known.type : 9;
    return new Promise((resolve, reject) => {
      const attempt = (n) => {
        if (keepGoing && !keepGoing()) {
          const ws = this.pendingParams.get(name);
          if (ws && !ws.length) this.pendingParams.delete(name);
          reject(Object.assign(new Error(`${name} not written: cancelled`), { code: 'cancelled', attempts: n })); return;
        }
        const timer = setTimeout(() => {
          const ws = this.pendingParams.get(name) || [];
          this.pendingParams.set(name, ws.filter(w => w.timer !== timer));
          if (n < retries) attempt(n + 1); else { this.pendingParams.delete(name); reject(new Error('param set timeout ' + name)); }
        }, timeoutMs);
        const ws = this.pendingParams.get(name) || []; ws.push({ resolve, reject, timer }); this.pendingParams.set(name, ws);
        this.send('PARAM_SET', { target_system: this.target.sysid, target_component: this.target.compid, param_id: name, param_value: value, param_type: type });
      };
      attempt(0);
    });
  }

  /** ask for a message at a rate (Hz) using MAV_CMD_SET_MESSAGE_INTERVAL (511) */
  setMessageRate(msgId, hz) {
    const interval = hz > 0 ? Math.round(1e6 / hz) : -1;
    this.commandLong(511, [msgId, interval]);
  }

  /** legacy stream request (works through older firmware and the Mission Planner mirror) */
  requestDataStream(streamId, hz) {
    this.send('REQUEST_DATA_STREAM', { target_system: this.target.sysid, target_component: this.target.compid, req_stream_id: streamId, req_message_rate: hz, start_stop: hz > 0 ? 1 : 0 });
  }

  commandLong(command, p = []) {
    this.send('COMMAND_LONG', { target_system: this.target.sysid, target_component: this.target.compid, command, confirmation: 0,
      param1: p[0] || 0, param2: p[1] || 0, param3: p[2] || 0, param4: p[3] || 0, param5: p[4] || 0, param6: p[5] || 0, param7: p[6] || 0 });
  }

  /** send a command and wait for its COMMAND_ACK; resolves {result} (0 = accepted) or rejects on timeout */
  commandAck(command, p = [], timeoutMs = 3000) {
    return new Promise((resolve, reject) => {
      const onAck = (f) => { if (f.command === command) { clearTimeout(t); this.off('COMMAND_ACK', onAck); resolve({ result: f.result }); } };
      const t = setTimeout(() => { this.off('COMMAND_ACK', onAck); reject(new Error('no ack for command ' + command)); }, timeoutMs);
      this.on('COMMAND_ACK', onAck);
      this.commandLong(command, p);
    });
  }

  get alive() { return this.connected && (Date.now() - this.lastHeartbeatAt) < 5000; }
}

function modeName(type, mode) {
  const copterTypes = new Set([2, 13, 14, 15, 29, 4, 3]);
  const table = copterTypes.has(type) ? ENUMS.COPTER_MODE : ENUMS.PLANE_MODE;
  const n = table?.[mode];
  return n ? n.replace(/^(COPTER|PLANE)_MODE_/, '') : String(mode);
}

/** serial ports with friendly names (Windows) */
function listSerialPorts() {
  try {
    const out = execFileSync('powershell.exe', ['-NoProfile', '-NonInteractive', '-Command',
      "$n = [System.IO.Ports.SerialPort]::GetPortNames(); $p = Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match '\\((COM\\d+)\\)' } | ForEach-Object { [pscustomobject]@{ port = [regex]::Match($_.Name, '\\((COM\\d+)\\)').Groups[1].Value; name = $_.Name } }; $n | ForEach-Object { $port = $_; $f = $p | Where-Object { $_.port -eq $port } | Select-Object -First 1; if ($f) { $f.name } else { $port } } | ConvertTo-Json -Compress"], { windowsHide: true, timeout: 15000 }).toString().trim();
    if (!out) return [];
    const arr = JSON.parse(out); const list = Array.isArray(arr) ? arr : [arr];
    return list.map(n => { const m = /\((COM\d+)\)/.exec(n); return { port: m ? m[1] : n, name: n }; });
  } catch (e) { return []; }
}

module.exports = { Link, modeName, listSerialPorts };
