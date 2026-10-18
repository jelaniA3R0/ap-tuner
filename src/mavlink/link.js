// MAVLink link: TCP client / UDP client / UDP listener / serial (through a PowerShell bridge, no
// native modules), heartbeat, parameter protocol, message intervals. Emits decoded messages.
//
// Bound to one vehicle (2026-10-17 audit F-25, F-28): the first ArduPilot autopilot heard (autopilot 3, component 1,
// not a ground station, antenna tracker or other peripheral) becomes the target and stays it; messages from every
// other system or component are dropped, another vehicle on the link is only reported. A different vehicle (the
// target silent for 5 s while another one answers, a heartbeat type change, another board UID) counts like a new
// connection: the generation goes up, every pending request is rejected and the cached values are dropped.
// Parameter echoes and command acknowledgements only resolve requests of the current generation (an ACK addressed to
// another ground station does not count); AUTOPILOT_VERSION (board UID, firmware) is asked for on every new target
// and after a reboot (the autopilot's boot clock going back).
'use strict';
const net = require('net');
const dgram = require('dgram');
const path = require('path');
const { spawn, execFileSync } = require('child_process');
const EventEmitter = require('events');
const { Parser, Encoder, ENUMS } = require('./mavlink');

const MAV_AUTOPILOT_ARDUPILOTMEGA = 3;
// heartbeat types that are never the vehicle: antenna tracker, ground station, onboard computer, gimbal, ADS-B, camera
const NOT_A_VEHICLE = new Set([5, 6, 18, 26, 27, 30]);
const FW_TYPE = { 0: 'dev', 64: 'alpha', 128: 'beta', 192: 'rc', 255: 'official' };

function linkError(message, code, extra = {}) { return Object.assign(new Error(message), { code }, extra); }
function typeName(type) { return ENUMS.MAV_TYPE?.[type] || String(type); }

/** AUTOPILOT_VERSION fields -> {fw: '4.5.7' (+ '-beta' etc.), gitHash, board, uid (hex), ...} */
function parseAutopilotVersion(f) {
  const v = (f.flight_sw_version || 0) >>> 0;
  const type = v & 0xff;
  const fw = v ? `${(v >>> 24) & 0xff}.${(v >>> 16) & 0xff}.${(v >>> 8) & 0xff}${type !== 255 ? '-' + (FW_TYPE[type] || type) : ''}` : null;
  const bytes = a => (Array.isArray(a) ? a : []);
  const hex = a => bytes(a).map(x => (x & 0xff).toString(16).padStart(2, '0')).join('');
  // uid2 (18 bytes) supersedes uid; both 0 when the board has no hardware id (SITL)
  const uid = bytes(f.uid2).some(x => x) ? hex(f.uid2) : (f.uid_hex && /[1-9a-f]/.test(f.uid_hex) ? f.uid_hex : null);
  const git = bytes(f.flight_custom_version).filter(x => x > 32 && x < 127).map(x => String.fromCharCode(x)).join('');
  return { fw, fwRaw: v, gitHash: git || null, board: (f.board_version || 0) >>> 0, vendor: f.vendor_id || 0, product: f.product_id || 0, uid };
}

class Link extends EventEmitter {
  constructor() {
    super();
    this.parser = new Parser();
    this.enc = new Encoder(255, 190);       // GCS system id 255 = SYSID_MYGCS default (RC overrides are only accepted from it)
    this.target = { sysid: 1, compid: 1 };
    this.pinned = false;                    // true once a vehicle answered: the target is fixed for this generation
    this.connected = false;
    this.transport = null;
    this.params = new Map();                // name -> {value, type, index}
    this.pendingParams = new Map();         // name -> [{resolve, reject, timer, attempt}]
    this._pendingCmds = new Set();          // commands waiting for their COMMAND_ACK: {cancel(why)}
    this.heartbeat = null; this.lastHeartbeatAt = 0;
    this.stats = { rx: 0, tx: 0, bytes: 0, foreign: 0 };
    this.msgCounts = {}; this._rateWin = { t: Date.now(), counts: {} }; this.msgRates = {};
    this._hbTimer = null; this._apTimer = null; this._tseq = 0;
    this.vehicle = null;                    // {type, autopilot, mode, armed, modeName}
    this.autopilot = null;                  // AUTOPILOT_VERSION of the target (null until it arrived)
    this._prevAutopilot = null;             // the one from before a reboot (a different board UID after it = another vehicle)
    this.others = new Map();                // other vehicles heard on this link: sysid -> {sysid, compid, type, lastSeen}
    this._warned = new Set();
    this.ignored = null;                    // the last heartbeat that could not be the target: {sysid, compid, autopilot, type}
    this.boot = { ms: null, at: 0, count: 0 };   // the autopilot's boot clock: count goes up on a reboot
    this.generation = 0;                    // increments on every connect and on a different vehicle
    this.description = '';
    this.opts = null;
  }

  /** opts: {kind:'tcp'|'udp'|'udpin'|'serial', host, port, baud} */
  connect(opts) {
    this.disconnect();
    this._newGeneration('the connection changed');
    this.opts = opts; this.serialOpened = false;
    this.parser = new Parser();
    // an older transport's late events (data, close, exit) are ignored: they must not touch this connection (F-40)
    const seq = ++this._tseq; const mine = () => this._tseq === seq;
    if (opts.kind === 'tcp') {
      this.description = `TCP ${opts.host || '127.0.0.1'}:${opts.port || 5760}`;
      const sock = net.createConnection({ host: opts.host || '127.0.0.1', port: opts.port || 5760 });
      sock.setNoDelay(true);
      sock.on('connect', () => { if (!mine()) return; this.connected = true; this.emit('status', 'tcp connected'); this._start(); });
      sock.on('data', d => { if (mine()) this._rx(d); });
      sock.on('error', e => { if (mine()) this.emit('status', 'tcp error: ' + e.message); });
      sock.on('close', () => { if (!mine()) return; this.connected = false; this.emit('status', 'tcp closed'); this._stop(); });
      this.transport = { send: b => { if (sock.writable) sock.write(b); }, close: () => sock.destroy() };
    } else if (opts.kind === 'serial') {
      this.description = `${opts.port} @ ${opts.baud || 115200}`;
      const script = path.join(__dirname, 'serialbridge.ps1');
      const child = spawn('powershell.exe', ['-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-File', script, opts.port, String(opts.baud || 115200)], { stdio: ['pipe', 'pipe', 'pipe'], windowsHide: true });
      child.stdout.on('data', d => { if (mine()) this._rx(d); });
      child.stderr.on('data', d => {
        if (!mine()) return;
        for (const line of d.toString().split(/\r?\n/).map(x => x.trim()).filter(Boolean)) {
          if (/^open /.test(line)) { this.serialOpened = true; this.emit('status', 'serial port opened ' + this.description); continue; }
          this.emit('status', 'serial: ' + line);
        }
      });
      child.on('exit', code => { if (!mine()) return; this.connected = false; this.emit('status', 'serial closed' + (code ? ' (code ' + code + ')' : '')); this._stop(); });
      child.on('error', e => { if (mine()) this.emit('status', 'serial error: ' + e.message); });
      this.transport = { send: b => { try { child.stdin.write(b); } catch (e) { /* closed */ } }, close: () => { try { child.stdin.end(); } catch (e) { /* ignore */ } setTimeout(() => { try { child.kill(); } catch (e) { /* ignore */ } }, 300); } };
      // the bridge prints nothing until bytes arrive; treat the port as open once the process survives its start
      setTimeout(() => { if (mine() && !this.connected && child.exitCode === null && this.transport) { this.connected = true; this.emit('status', 'serial open ' + this.description); this._start(); } }, 800);
    } else {
      const sock = dgram.createSocket('udp4');
      let remote = null;
      if (opts.kind === 'udpin') {
        this.description = `UDP listen ${opts.port || 14550}`;
        sock.bind(opts.port || 14550, opts.host || '0.0.0.0', () => { if (!mine()) return; this.connected = true; this.emit('status', `udp listening on ${opts.port || 14550}`); this._start(); });
      } else {
        this.description = `UDP ${opts.host || '127.0.0.1'}:${opts.port || 14550}`;
        remote = { address: opts.host || '127.0.0.1', port: opts.port || 14550 };
        sock.bind(0, () => { if (!mine()) return; this.connected = true; this.emit('status', `udp client -> ${remote.address}:${remote.port}`); this._start(); });
      }
      sock.on('message', (d, rinfo) => { if (!mine()) return; if (!remote || opts.kind === 'udpin') remote = { address: rinfo.address, port: rinfo.port }; this._rx(d); });
      sock.on('error', e => { if (mine()) this.emit('status', 'udp error: ' + e.message); });
      this.transport = { send: b => { if (remote) sock.send(b, remote.port, remote.address); }, close: () => { try { sock.close(); } catch (e) { /* ignore */ } } };
    }
  }

  disconnect() {
    this._stop();
    this._tseq++;
    if (this.transport) { try { this.transport.close(); } catch (e) { /* ignore */ } }
    this.transport = null; this.connected = false; this.opts = null;
    this._rejectPending('the connection was closed');
  }

  /** a connection on a given transport object {send(buf), close()} (tests: a fake vehicle); bytes come in via feed() */
  attachTransport(transport, description = 'test transport') {
    this.disconnect();
    this._newGeneration('the connection changed');
    this.opts = { kind: 'test' }; this.serialOpened = false;
    this.parser = new Parser();
    this.description = description; this.transport = transport; this.connected = true;
    this._start();
  }
  feed(data) { this._rx(data); }

  /** a new generation: nothing pending, cached or pinned from before survives */
  _newGeneration(why) {
    this.generation++;
    this._rejectPending(why);
    this.params.clear();
    this.pinned = false; this.target = { sysid: 1, compid: 1 };
    this.lastHeartbeatAt = 0; this.heartbeat = null; this.vehicle = null;
    this.autopilot = null; this._prevAutopilot = null; if (this._apTimer) clearTimeout(this._apTimer); this._apTimer = null;
    this.boot = { ms: null, at: 0, count: 0 };
    this.others.clear(); this._warned.clear(); this.ignored = null;
  }
  /** another vehicle behind the same transport: like a new connection, the target becomes the vehicle now heard */
  _vehicleChanged(why) {
    this.generation++;
    this._rejectPending('another vehicle answered (' + why + ')');
    this.params.clear();
    this.autopilot = null; this._prevAutopilot = null;
    this.boot = { ms: null, at: 0, count: 0 };
    this.emit('status', 'another vehicle: ' + why);
    this._changedWhy = why;
  }
  /** every request waiting for an answer is rejected now (code 'link-changed', or the code given) */
  _rejectPending(why, code = 'link-changed') {
    const pend = this.pendingParams; this.pendingParams = new Map();
    for (const [name, ws] of pend) for (const w of ws) { clearTimeout(w.timer); w.reject(linkError(`${name}: ${why}`, code, { attempts: w.attempt + 1 })); }
    for (const c of [...this._pendingCmds]) c.cancel(why, code);
  }

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
      if (m.name === 'HEARTBEAT') { this._onHeartbeat(m, now); continue; }
      // only the target speaks: other systems and components (another vehicle, a gimbal, a ground station) are dropped
      if (!this.pinned || m.sysid !== this.target.sysid || m.compid !== this.target.compid) { this.stats.foreign++; continue; }
      const f = m.fields;
      if (f.time_boot_ms !== undefined) this._bootClock(f.time_boot_ms, now);
      if (m.name === 'PARAM_VALUE') {
        this.params.set(f.param_id, { value: f.param_value, type: f.param_type, index: f.param_index, count: f.param_count, ts: now });
        const waiters = this.pendingParams.get(f.param_id);
        if (waiters) { this.pendingParams.delete(f.param_id); for (const w of waiters) { clearTimeout(w.timer); w.resolve(f.param_value); } }
        this.emit('param', f.param_id, f.param_value, f.param_type);
      } else if (m.name === 'AUTOPILOT_VERSION') this._onAutopilotVersion(f);
      this.emit('message', m);
      this.emit(m.name, m.fields, m);
    }
  }

  _onHeartbeat(m, now) {
    const f = m.fields;
    if (f.autopilot !== MAV_AUTOPILOT_ARDUPILOTMEGA || m.compid !== 1 || NOT_A_VEHICLE.has(f.type)) {
      // a ground station, companion, tracker, gimbal or a non-ArduPilot autopilot: never the target
      if (f.type !== 6 && m.compid === 1 && (!this.ignored || this.ignored.sysid !== m.sysid)) {
        this.ignored = { sysid: m.sysid, compid: m.compid, autopilot: f.autopilot, type: f.type };
        if (f.autopilot !== MAV_AUTOPILOT_ARDUPILOTMEGA && f.autopilot !== 8) this.emit('status', `system id ${m.sysid} is not an ArduPilot autopilot (autopilot ${f.autopilot}): AP Tuner only works with ArduPilot`);
      }
      return;
    }
    const isTarget = this.pinned && m.sysid === this.target.sysid && m.compid === this.target.compid;
    let changed = false;
    if (!isTarget) {
      if (this.pinned && now - this.lastHeartbeatAt < 5000) { this._noteOther(m, now); return; }   // ours is alive: only report the other
      if (this.pinned) { this._vehicleChanged(`system id ${this.target.sysid} went silent and system id ${m.sysid} answered`); changed = true; }
      this.pinned = true; this.target = { sysid: m.sysid, compid: m.compid };
      this.others.delete(m.sysid);
      this.vehicle = null;
      this._requestAutopilotVersion();
    } else if (this.vehicle && this.vehicle.type !== f.type) {
      // the same system id with another airframe type: a different vehicle behind the same link (e.g. the mirror)
      this._vehicleChanged(`the vehicle type changed from ${this.vehicle.typeName} to ${typeName(f.type)}`);
      changed = true;
      this._requestAutopilotVersion();
    }
    this.lastHeartbeatAt = now;
    this.heartbeat = f;
    const armed = !!(f.base_mode & 128);
    this.vehicle = { type: f.type, autopilot: f.autopilot, mode: f.custom_mode, armed, modeName: modeName(f.type, f.custom_mode), typeName: typeName(f.type) };
    if (changed) this.emit('vehicle-change', { reason: this._changedWhy, generation: this.generation });
    this.emit('heartbeat', this.vehicle);
    this.emit('message', m);
    this.emit('HEARTBEAT', f, m);
  }

  _noteOther(m, now) {
    this.others.set(m.sysid, { sysid: m.sysid, compid: m.compid, type: m.fields.type, lastSeen: now });
    if (this._warned.has(m.sysid)) return;
    this._warned.add(m.sysid);
    this.emit('status', `another vehicle is on this link (system id ${m.sysid}, ${typeName(m.fields.type)}): the app stays with system id ${this.target.sysid}`);
  }
  /** other vehicles heard in the last 10 s: [{sysid, compid, type}] */
  othersSeen() { const now = Date.now(); return [...this.others.values()].filter(o => now - o.lastSeen < 10000).map(o => ({ sysid: o.sysid, compid: o.compid, type: o.type })); }

  /** the autopilot's time since boot going back by more than 3 s is a reboot */
  _bootClock(ms, now) {
    const b = this.boot;
    if (b.ms !== null && ms + 3000 < b.ms + (now - b.at)) {
      b.count++;
      this._rejectPending('the autopilot rebooted', 'rebooted');
      if (this.autopilot) this._prevAutopilot = this.autopilot;
      this.autopilot = null;
      this.emit('status', 'the autopilot rebooted');
      this.emit('reboot', { count: b.count });
      this._requestAutopilotVersion();
    }
    b.ms = ms; b.at = now;
  }

  _onAutopilotVersion(f) {
    const info = parseAutopilotVersion(f);
    const prev = this.autopilot || this._prevAutopilot;
    if (this._apTimer) clearTimeout(this._apTimer); this._apTimer = null;
    if (prev && prev.uid && info.uid && prev.uid !== info.uid) {
      this._vehicleChanged(`another board answered (UID ${info.uid.slice(0, 24)})`);
      this.autopilot = info;
      this.emit('vehicle-change', { reason: this._changedWhy, generation: this.generation });
    } else this.autopilot = info;
    this._prevAutopilot = null;
    this.emit('autopilot', info);
  }

  /** ask the target for AUTOPILOT_VERSION (REQUEST_MESSAGE 512, older firmware: REQUEST_AUTOPILOT_CAPABILITIES 520) */
  _requestAutopilotVersion() {
    if (this._apTimer) clearTimeout(this._apTimer);
    const gen = this.generation; let n = 0;
    const ask = () => {
      this._apTimer = null;
      if (this.generation !== gen || this.autopilot || !this.transport || !this.pinned) return;
      if (n % 2 === 0) this.commandLong(512, [148]); else this.commandLong(520, [1]);
      if (++n < 6) this._apTimer = setTimeout(ask, 1000);
    };
    ask();
  }
  /** resolves with AUTOPILOT_VERSION once known, or null after ms */
  async waitAutopilot(ms = 3000) {
    const t0 = Date.now();
    if (!this.autopilot && this.pinned && !this._apTimer) this._requestAutopilotVersion();
    while (!this.autopilot && Date.now() - t0 < ms) await new Promise(r => setTimeout(r, 50));
    return this.autopilot;
  }

  /** who the app talks to on this connection (gen = generation: a reconnect or another vehicle changes it) */
  identity() {
    const v = this.vehicle, a = this.autopilot;
    return { gen: this.generation, sysid: this.pinned ? this.target.sysid : null, compid: this.pinned ? this.target.compid : null,
      autopilot: v ? v.autopilot : null, type: v ? v.type : null, typeName: v ? v.typeName : null,
      uid: a ? a.uid : null, fw: a ? a.fw : null, gitHash: a ? a.gitHash : null, board: a ? a.board : null, link: this.description };
  }

  send(name, fields) {
    if (!this.transport) return;
    const buf = this.enc.pack(name, fields);
    this.transport.send(buf); this.stats.tx++;
  }

  _addWaiter(name, w) { const ws = this.pendingParams.get(name) || []; ws.push(w); this.pendingParams.set(name, ws); }
  _dropWaiter(name, w) { const ws = this.pendingParams.get(name); if (!ws) return; const i = ws.indexOf(w); if (i >= 0) ws.splice(i, 1); if (!ws.length) this.pendingParams.delete(name); }

  /** request one parameter by name; resolves to value (rejects on timeout: the parameter does not exist or the link is slow) */
  requestParam(name, timeoutMs = 1500, retries = 4) {
    const gen = this.generation;
    return new Promise((resolve, reject) => {
      const attempt = (n) => {
        if (this.generation !== gen) { reject(linkError(`${name}: the connection changed`, 'link-changed', { attempts: n })); return; }
        const w = { resolve, reject, attempt: n, timer: null };
        w.timer = setTimeout(() => { this._dropWaiter(name, w); if (n < retries) attempt(n + 1); else reject(linkError('param timeout ' + name, 'timeout', { attempts: n + 1 })); }, timeoutMs);
        this._addWaiter(name, w);
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
   * set a parameter and wait for the echo; resolves the value the vehicle reports (compare it: see paramwrite.js).
   * keepGoing (optional): checked before every PARAM_SET, retries included; when it returns false nothing more is
   * sent and the promise rejects with code 'cancelled' (attempts > 0: an earlier attempt may have reached the vehicle).
   * A new connection or another vehicle rejects it with code 'link-changed'.
   */
  setParam(name, value, timeoutMs = 1500, retries = 3, keepGoing = null) {
    const gen = this.generation;
    const known = this.params.get(name);
    const type = known ? known.type : 9;
    return new Promise((resolve, reject) => {
      const attempt = (n) => {
        if (this.generation !== gen) { reject(linkError(`${name} not written: the connection changed`, 'link-changed', { attempts: n })); return; }
        if (keepGoing && !keepGoing()) { reject(linkError(`${name} not written: cancelled`, 'cancelled', { attempts: n })); return; }
        const w = { resolve, reject, attempt: n, timer: null };
        w.timer = setTimeout(() => { this._dropWaiter(name, w); if (n < retries) attempt(n + 1); else reject(linkError('param set timeout ' + name, 'timeout', { attempts: n + 1 })); }, timeoutMs);
        this._addWaiter(name, w);
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

  /**
   * send a command and wait for its COMMAND_ACK from the target; resolves {result} (0 = accepted) or rejects on timeout
   * or when the connection changes. An ACK naming another requester (ArduPilot fills target_system / component) is not ours.
   */
  commandAck(command, p = [], timeoutMs = 3000) {
    return new Promise((resolve, reject) => {
      const done = () => { clearTimeout(t); this.off('COMMAND_ACK', onAck); this._pendingCmds.delete(entry); };
      const onAck = (f) => {
        if (f.command !== command) return;
        if ((f.target_system && f.target_system !== this.enc.sysid) || (f.target_component && f.target_component !== this.enc.compid)) return;
        done(); resolve({ result: f.result });
      };
      const entry = { cancel: (why, code) => { done(); reject(linkError(`command ${command}: ${why}`, code || 'link-changed')); } };
      const t = setTimeout(() => { done(); reject(linkError('no ack for command ' + command, 'timeout')); }, timeoutMs);
      this.on('COMMAND_ACK', onAck); this._pendingCmds.add(entry);
      this.commandLong(command, p);
    });
  }

  get alive() { return this.connected && this.pinned && (Date.now() - this.lastHeartbeatAt) < 5000; }
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

module.exports = { Link, modeName, listSerialPorts, parseAutopilotVersion, MAV_AUTOPILOT_ARDUPILOTMEGA };
