// AP Tuner server: HTTP + Server-Sent Events, no dependencies. Serves the web UI and drives the tuner.
// Only this PC's own app page may drive it: the Host (and Origin, when sent) must be 127.0.0.1 / localhost on
// this port, every state-changing request must be a POST carrying the per-run token that is written into the
// page, and the page is served with a Content-Security-Policy (2026-10-17 audit F-05, F-90).
'use strict';
const http = require('http');
const fs = require('fs');
const path = require('path');
const os = require('os');
const crypto = require('crypto');
const { Link, listSerialPorts } = require('./mavlink/link');
const { Tuner, AXES } = require('./engine/tuner');
const { NotchLive } = require('./engine/notchlive');
const { FlightSession } = require('./engine/flight');
const { GroundTune } = require('./engine/groundtune');
const { analyzeLog } = require('./engine/loganalysis');
const { proposalRows, paramFile } = require('./engine/report');
const { PRESETS } = require('./model/rules');
const { axisParamNames } = require('./engine/vehicle');

const PORT = parseInt(process.env.PORT || '8090', 10);
const APP_MODE = process.argv.includes('--app');
const WEB = path.join(__dirname, '..', 'web');
const SESSIONS = path.join(__dirname, '..', 'sessions');
const TOKEN = crypto.randomBytes(24).toString('hex');   // per run; the app page gets it, other pages cannot read it
const link = new Link();
const tuner = new Tuner(link);
tuner.startMonitor();
const notch = new NotchLive(link, tuner);
const flight = new FlightSession(link, tuner, notch);
flight.on('phase', p => sse('flight', p));
const ground = new GroundTune(link, { sessionsDir: SESSIONS });
ground.on('phase', p => sse('ground', p));
const clients = new Set();
let lastLogResult = null;
let autoState = { running: false, tried: [], found: null };
let everHadClient = false, lastClientAt = Date.now();

function sse(event, data) { const s = `event: ${event}\ndata: ${JSON.stringify(data)}\n\n`; for (const c of clients) c.write(s); }
link.on('status', s => { tuner.log('link', s); });
tuner.on('log', e => sse('log', e));
tuner.on('state', s => sse('state', s));
tuner.on('analysis', r => sse('analysis', r));
tuner.on('param', p => sse('param', p));
tuner.on('progress', p => sse('progress', p));

// periodic snapshot with recent series for the charts
setInterval(() => {
  if (!clients.size) return;
  const snap = tuner.snapshot();
  snap.series = {};
  for (const ax of AXES) {
    const la = tuner.axes[ax]; const n = la.t.length; const keep = 600; const step = Math.max(1, Math.floor(Math.min(n, 2500) / keep));
    const i0 = Math.max(0, n - keep * step);
    const s = { t: [], target: [], actual: [], P: [], I: [], D: [], FF: [], srate: [], dmod: [] };
    for (let i = i0; i < n; i += step) { s.t.push(+la.t[i].toFixed(3)); s.target.push(+la.target[i].toFixed(2)); s.actual.push(+la.actual[i].toFixed(2)); s.P.push(+la.P[i].toFixed(4)); s.I.push(+la.I[i].toFixed(4)); s.D.push(+la.D[i].toFixed(4)); s.FF.push(+la.FF[i].toFixed(4)); s.srate.push(+la.srate[i].toFixed(2)); s.dmod.push(+la.dmod[i].toFixed(3)); }
    snap.series[ax] = s;
  }
  snap.connected = link.connected; snap.alive = link.alive; snap.linkDesc = link.description; snap.auto = autoState;
  snap.escTelemetry = notch.esc.seen && Date.now() - notch.esc.lastTs < 20000;
  snap.flight = flight.snapshot();
  snap.ground = ground.snapshot();
  sse('snapshot', snap);
}, 250);

// in app mode the server exits when the app window has been closed for a while
// in app mode the server exits when the app window has really been closed (no client for 20 s AND no
// Edge app window on our URL any more); a sleeping laptop or a short network hiccup must not kill it
if (APP_MODE) setInterval(() => {
  if (!(everHadClient && !clients.size && Date.now() - lastClientAt > 20000)) return;
  try {
    const out = require('child_process').execFileSync('powershell.exe', ['-NoProfile', '-NonInteractive', '-Command', "(Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'msedge.exe' -and $_.CommandLine -like '*--app=http://127.0.0.1:" + PORT + "*' } | Measure-Object).Count"], { windowsHide: true, timeout: 10000 }).toString().trim();
    if (parseInt(out, 10) > 0) { lastClientAt = Date.now(); return; }   // window still open: keep serving
  } catch (e) { /* if the check fails, fall through and exit as before */ }
  console.log('app window closed, exiting'); process.exit(0);
}, 5000);

function json(res, code, obj, headers = {}) { const b = JSON.stringify(obj); res.writeHead(code, Object.assign({ 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(b), 'X-Content-Type-Options': 'nosniff' }, headers)); res.end(b); }

// ---------- request gate (F-05) ----------
const ALLOWED_HOSTS = new Set([`127.0.0.1:${PORT}`, `localhost:${PORT}`]);
const ALLOWED_ORIGINS = new Set([`http://127.0.0.1:${PORT}`, `http://localhost:${PORT}`]);
// API routes that only read: GET, no token. Everything else under /api/ changes state: POST + token.
const READ_ROUTES = new Set(['/api/events', '/api/ports', '/api/presets', '/api/notch/status', '/api/live/rows', '/api/live/param', '/api/log/list', '/api/log/param', '/api/ground/result', '/api/ground/state', '/api/ground/file']);
const CSP = "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'";
function sameToken(t) { const a = Buffer.from(String(t || '')), b = Buffer.from(TOKEN); return a.length === b.length && crypto.timingSafeEqual(a, b); }
/** null when the request may go on, else [status, error, extra headers] */
function gate(req, url) {
  if (!ALLOWED_HOSTS.has(String(req.headers.host || '').toLowerCase())) return [403, 'forbidden: unknown Host'];
  const origin = req.headers.origin;
  if (origin !== undefined && !ALLOWED_ORIGINS.has(String(origin).toLowerCase())) return [403, 'forbidden: request from another site'];
  if (req.headers['sec-fetch-site'] === 'cross-site') return [403, 'forbidden: request from another site'];
  if (!url.pathname.startsWith('/api/')) return null;
  const write = !READ_ROUTES.has(url.pathname) || (url.pathname === '/api/log/param' && url.searchParams.get('save') === '1');
  if (!write) return req.method === 'GET' || req.method === 'HEAD' ? null : [405, 'use GET', { Allow: 'GET' }];
  if (req.method !== 'POST') return [405, 'this request changes something: it must be a POST', { Allow: 'POST' }];
  if (!sameToken(req.headers['x-aptuner-token'])) return [403, 'missing or wrong app token: reload the page (the app may have been restarted)'];
  return null;
}
function readBody(req) { return new Promise(r => { let b = ''; req.on('data', c => b += c); req.on('end', () => { try { r(b ? JSON.parse(b) : {}); } catch (e) { r({}); } }); }); }
function readRaw(req) { return new Promise(r => { const chunks = []; req.on('data', c => chunks.push(c)); req.on('end', () => r(Buffer.concat(chunks))); }); }

const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.css': 'text/css', '.png': 'image/png', '.svg': 'image/svg+xml', '.ico': 'image/x-icon', '.woff2': 'font/woff2' };

function listLogs() {
  const dirs = ['D:\\', path.join(os.homedir(), 'Documents', 'Mission Planner', 'logs'), path.join(os.homedir(), 'Desktop'), path.join(os.homedir(), 'Downloads'), path.join(SESSIONS, 'uploads')];
  const out = [];
  const walk = (d, depth) => {
    let ents; try { ents = fs.readdirSync(d, { withFileTypes: true }); } catch (e) { return; }
    for (const e of ents) {
      const p = path.join(d, e.name);
      if (e.isDirectory()) { if (depth > 0 && !e.name.startsWith('$') && e.name !== 'System Volume Information') walk(p, depth - 1); }
      else if (/\.bin$/i.test(e.name)) { try { const st = fs.statSync(p); out.push({ path: p, size: st.size, mtime: st.mtimeMs }); } catch (err) { /* ignore */ } }
    }
  };
  for (const d of dirs) walk(d, 2);
  const seen = new Set();
  return out.filter(x => { const k = x.path.toLowerCase(); if (seen.has(k)) return false; seen.add(k); return true; }).sort((a, b) => b.mtime - a.mtime).slice(0, 60);
}

/** try the usual places for a vehicle: Mission Planner mirror, SITL, then serial ports */
/** true when Mission Planner is running on this PC (it then owns the USB / telemetry port) */
function missionPlannerRunning() {
  try {
    const out = require('child_process').execFileSync('tasklist', ['/FI', 'IMAGENAME eq MissionPlanner.exe', '/NH'], { windowsHide: true, timeout: 8000 }).toString();
    return /MissionPlanner.exe/i.test(out);
  } catch (e) { return false; }
}

// telemetry radios: SiK / RFD900 default 57600; USB and many radios 115200; RFD900x / ELRS / long-range links faster
const SERIAL_BAUDS = [57600, 115200, 921600, 460800, 230400];

async function autoconnect(onProgress) {
  const cands = [
    { kind: 'tcp', host: '127.0.0.1', port: 14550, label: 'Mission Planner mirror (TCP 14550)', wait: 3000 },
    { kind: 'tcp', host: '127.0.0.1', port: 5762, label: 'Simulator (SITL, port 5762)', wait: 2500 },
    { kind: 'tcp', host: '127.0.0.1', port: 5763, label: 'Simulator (SITL, port 5763)', wait: 2500 },
    { kind: 'tcp', host: '127.0.0.1', port: 5760, label: 'Simulator (SITL, port 5760)', wait: 2500 },
    { kind: 'udpin', port: 14550, label: 'UDP 14550 (Mission Planner UDP mirror or a Wi-Fi link)', wait: 3500 },
  ];
  const mpOpen = missionPlannerRunning();
  // Bluetooth COM ports are skipped (opening one with no paired device hangs); use the manual connection for them
  const ports = listSerialPorts().filter(p => !/bluetooth/i.test(p.name));
  for (const p of ports) for (const baud of SERIAL_BAUDS) cands.push({ kind: 'serial', port: p.port, baud, label: `${p.name} at ${baud}`, wait: 3500 });
  autoState = { running: true, tried: [], found: null, missionPlanner: mpOpen };
  const busyPorts = new Set(), silentPorts = new Set();
  for (const c of cands) {
    if (c.kind === 'serial' && (busyPorts.has(c.port) || silentPorts.has(c.port))) continue; // held by another program, or nothing transmits on it
    autoState.current = c.label; onProgress({ trying: c.label });
    let lastStatus = ''; const bytes0 = link.stats.bytes;
    const onStatus = st => { lastStatus = st; };
    link.on('status', onStatus);
    try { link.connect(c); } catch (e) { link.off('status', onStatus); autoState.tried.push({ label: c.label, result: e.message }); continue; }
    let t0 = Date.now(), ok = false, opened = c.kind !== 'serial';
    // serial: the listening time starts when the port is really open (the bridge can take seconds to start)
    if (c.kind === 'serial') {
      while (Date.now() - t0 < 10000 && !link.serialOpened && !/cannot open|serial closed/.test(lastStatus) && !link.alive) await new Promise(r => setTimeout(r, 100));
      opened = link.serialOpened || link.alive;
      t0 = Date.now();
    }
    while (opened && Date.now() - t0 < c.wait) {
      await new Promise(r => setTimeout(r, 200));
      if (link.alive) { ok = true; break; }
      if (c.kind === 'serial' && /cannot open/.test(lastStatus)) break;
      if (!link.connected && Date.now() - t0 > 1200 && c.kind !== 'serial') break;
    }
    link.off('status', onStatus);
    if (ok) { autoState = { running: false, tried: autoState.tried, found: Object.assign({}, c), missionPlanner: mpOpen }; onProgress({ found: c.label }); return autoState.found; }
    let result = 'no heartbeat';
    // a wrong baud still gives garbage bytes; no bytes at all means nothing is transmitting on this port at any baud
    // only a port that really opened and stayed quiet for the whole wait counts as silent
    if (c.kind === 'serial' && opened && link.stats.bytes === bytes0 && !/cannot open/.test(lastStatus)) { silentPorts.add(c.port); result = 'no data (port open, nothing received: is the aircraft powered and the radio link LED solid?)'; }
    else if (c.kind === 'serial' && !opened && !/cannot open/.test(lastStatus)) { busyPorts.add(c.port); result = 'port did not open within 10 s'; }
    else if (c.kind === 'serial' && link.stats.bytes > bytes0 && !ok) result = 'data but no MAVLink heartbeat at this speed';
    if (c.kind === 'serial' && /cannot open/.test(lastStatus)) {
      busyPorts.add(c.port);
      result = /denied|in use|access/i.test(lastStatus) ? 'port in use by another program' : lastStatus.replace(/^serial: /, '');
    }
    autoState.tried.push({ label: c.kind === 'serial' && busyPorts.has(c.port) ? c.port + ' (' + c.label.replace(/ at d+$/, '') + ')' : c.label, result, busy: busyPorts.has(c.port) });
    link.disconnect();
    await new Promise(r => setTimeout(r, c.kind === 'serial' ? 600 : 150)); // let Windows release the COM port before the next try
  }
  autoState = { running: false, tried: autoState.tried, found: null, missionPlanner: mpOpen, busyPorts: [...busyPorts] };
  return null;
}

function liveRows() {
  const snap = tuner.snapshot(); const rows = [];
  if (!tuner.family || !snap.gains) return rows;
  for (const ax of AXES) {
    const n = axisParamNames(tuner.family, ax); const g = snap.gains[ax]; const o = tuner.originals[ax] || {};
    for (const k of ['P', 'I', 'D', 'FF', 'FLTT', 'FLTE', 'FLTD', 'SMAX', 'angleP', 'tconst']) if (n[k] && g[k] !== undefined) rows.push({ group: ax, name: n[k], value: +g[k].toPrecision(6), from: o[k], label: k, changed: o[k] !== undefined && Math.abs(o[k] - g[k]) > 1e-6 * Math.max(1, Math.abs(o[k])) });
  }
  for (const k of ['INS_HNTCH_ENABLE', 'INS_HNTCH_MODE', 'INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_ATT', 'INS_HNTCH_REF', 'INS_HNTCH_HMNCS', 'INS_HNTCH_OPTS', 'INS_HNTCH_FM_RAT', 'INS_GYRO_FILTER']) if (tuner.params[k] !== undefined) rows.push({ group: 'notch', name: k, value: tuner.params[k], label: k });
  return rows;
}

const server = http.createServer(async (req, res) => {
  const url = new URL(req.url, 'http://x');
  try {
    const refused = gate(req, url);
    if (refused) { req.resume(); return json(res, refused[0], { error: refused[1] }, refused[2] || {}); }
    if (url.pathname === '/api/events') {
      res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache', Connection: 'keep-alive' });
      res.write('retry: 2000\n\n'); clients.add(res); everHadClient = true; req.on('close', () => { clients.delete(res); lastClientAt = Date.now(); });
      for (const r of Object.values(tuner.results)) res.write(`event: analysis\ndata: ${JSON.stringify(r)}\n\n`);
      return;
    }
    if (url.pathname.startsWith('/api/')) {
      const isJson = /application\/json/.test(req.headers['content-type'] || '');
      const body = req.method === 'POST' && isJson ? await readBody(req) : {};
      switch (url.pathname) {
        case '/api/autoconnect': {
          if (autoState.running) return json(res, 200, { ok: false, running: true });
          const found = await autoconnect(p => sse('connect', p));
          return json(res, 200, found ? { ok: true, found } : { ok: false, tried: autoState.tried, missionPlanner: autoState.missionPlanner, busyPorts: autoState.busyPorts || [] });
        }
        case '/api/connect': link.connect({ kind: body.kind || 'tcp', host: body.host || '127.0.0.1', port: body.kind === 'serial' ? body.port : parseInt(body.port || 14550, 10), baud: parseInt(body.baud || 115200, 10) }); return json(res, 200, { ok: true });
        case '/api/disconnect': link.disconnect(); return json(res, 200, { ok: true });
        case '/api/ports': return json(res, 200, listSerialPorts());
        case '/api/prepare': { const r = await tuner.prepare({ confirm: !!body.confirm }); return json(res, 200, Object.assign({ ok: true }, r)); }
        case '/api/start': setOpts(body); tuner.start(); return json(res, 200, { ok: true, opts: tuner.opts });
        case '/api/options': setOpts(body); return json(res, 200, { ok: true, opts: tuner.opts });
        case '/api/pause': tuner.pause('paused by user'); return json(res, 200, { ok: true });
        case '/api/resume': tuner.resume(); return json(res, 200, { ok: true });
        case '/api/stop': await tuner.stop(); return json(res, 200, { ok: true });
        case '/api/apply': { const ok = await tuner.applyProposal(body.axis); if (ok && tuner.state !== 'paused') tuner.resume(); return json(res, 200, { ok }); }
        case '/api/undo': await tuner.undoAxis(body.axis); return json(res, 200, { ok: true });
        case '/api/restore': await tuner.restoreOriginals(); return json(res, 200, { ok: true });
        case '/api/analyze': { const r = tuner.analyze(body.axis || tuner.currentAxis(), body.seconds || null); return json(res, 200, r); }
        // (no /api/setparam: arbitrary parameter writes are not offered, F-05)
        case '/api/presets': return json(res, 200, PRESETS);
        case '/api/notch/status': return json(res, 200, await notch.status());
        case '/api/notch/esc': return json(res, 200, { ok: true, applied: await notch.applyEsc() });
        case '/api/notch/fft/enable': return json(res, 200, Object.assign({ ok: true }, await notch.enableFft()));
        case '/api/notch/reboot': return json(res, 200, Object.assign({ ok: true }, await notch.reboot()));
        case '/api/notch/fft/start': return json(res, 200, Object.assign({ ok: true }, await notch.startFftTune()));
        case '/api/notch/fft/stop': return json(res, 200, Object.assign({ ok: true }, await notch.stopFftTune()));
        case '/api/flight/start': { setOpts(body); flight.start({ axes: tuner.opts.axes, preset: tuner.opts.preset, auto: !!body.auto, observeSeconds: tuner.opts.observeSeconds }); return json(res, 200, { ok: true, flight: flight.snapshot() }); }
        case '/api/flight/continue': flight.continue(); return json(res, 200, { ok: true });
        case '/api/flight/repeat': flight.repeatCheckup(); return json(res, 200, { ok: true });
        case '/api/flight/skip': flight.skipFilters(); return json(res, 200, { ok: true });
        case '/api/flight/resume': flight.resume(); return json(res, 200, { ok: true });
        case '/api/flight/pause': tuner.pause('paused by you'); return json(res, 200, { ok: true });
        case '/api/flight/stop': flight.stop(); return json(res, 200, { ok: true });
        case '/api/ground/start': {
          // developer hook: APT_TEST_USB=5763 makes a SITL TCP port stand in for the USB cable
          if (!body.testUsb && process.env.APT_TEST_USB) body.testUsb = { kind: 'tcp', port: process.env.APT_TEST_USB };
          const axes = Array.isArray(body.axes) ? body.axes.filter(a => AXES.includes(a)) : undefined;
          const usb = body.testUsb && body.testUsb.kind === 'tcp' ? { kind: 'tcp', host: '127.0.0.1', port: parseInt(body.testUsb.port, 10), label: 'test TCP ' + body.testUsb.port } : null;
          ground.start({ axes: axes && axes.length ? axes : undefined, preset: body.preset || 'normal', seconds: Math.max(15, Math.min(90, parseInt(body.seconds || 30, 10))), usb });
          return json(res, 200, { ok: true });
        }
        case '/api/ground/again': ground.captureAgain(); return json(res, 200, { ok: true });
        case '/api/ground/usb': {
          if (body.testUsb && body.testUsb.kind === 'tcp') ground.opts.usb = { kind: 'tcp', host: '127.0.0.1', port: parseInt(body.testUsb.port, 10), label: 'test TCP ' + body.testUsb.port };
          await ground.goUsb({ useSaved: !!body.useSaved }); return json(res, 200, { ok: true });
        }
        case '/api/ground/write': return json(res, 200, { ok: true, results: await ground.write(Array.isArray(body.names) ? body.names : []) });
        case '/api/ground/undo': await ground.undo(); return json(res, 200, { ok: true });
        case '/api/ground/stop': ground.stop(); return json(res, 200, { ok: true });
        case '/api/ground/result': return json(res, 200, ground.result || {});
        case '/api/ground/state': return json(res, 200, ground.snapshot());
        case '/api/ground/file': {
          const which = url.searchParams.get('which'); const fp = ground.files[which];
          if (!fp || !fs.existsSync(fp)) return json(res, 404, { error: 'no file' });
          res.writeHead(200, { 'Content-Type': 'application/octet-stream', 'Content-Disposition': `attachment; filename="${path.basename(fp)}"` }); return fs.createReadStream(fp).pipe(res);
        }
        case '/api/live/rows': return json(res, 200, { family: tuner.family, rows: liveRows(), journal: tuner.journal });
        case '/api/live/param': { const rows = liveRows(); const txt = paramFile(rows, [`Live session gains read from the vehicle on ${new Date().toLocaleString()}`]); res.writeHead(200, { 'Content-Type': 'text/plain', 'Content-Disposition': 'attachment; filename="AP Tuner live gains.param"' }); return res.end(txt); }
        case '/api/log/list': return json(res, 200, listLogs());
        case '/api/log/analyze': { if (!body.path || !fs.existsSync(body.path)) return json(res, 400, { error: 'file not found: ' + body.path }); lastLogResult = analyzeLog(body.path, { preset: body.preset || 'normal', family: body.family || null }); lastLogResult.rows = proposalRows(lastLogResult.family, lastLogResult.axes, lastLogResult.noise); return json(res, 200, lastLogResult); }
        case '/api/log/upload': {
          const name = (url.searchParams.get('name') || 'upload.bin').replace(/[^\w .()-]/g, '_');
          const dir = path.join(SESSIONS, 'uploads'); fs.mkdirSync(dir, { recursive: true });
          const file = path.join(dir, name); fs.writeFileSync(file, await readRaw(req));
          lastLogResult = analyzeLog(file, { preset: url.searchParams.get('preset') || 'normal', family: url.searchParams.get('family') || null });
          lastLogResult.rows = proposalRows(lastLogResult.family, lastLogResult.axes, lastLogResult.noise);
          return json(res, 200, lastLogResult);
        }
        case '/api/log/param': {
          if (!lastLogResult) return json(res, 400, { error: 'analyse a log first' });
          const txt = paramFile(lastLogResult.rows, [`From log ${path.basename(lastLogResult.file)} (${lastLogResult.firmware || ''}), ${lastLogResult.family}, preset ${Object.values(lastLogResult.axes).find(a => a && a.preset)?.preset || 'normal'}`, 'Fly carefully after loading: one axis at a time, ready to switch back to your previous parameters.']);
          const fname = path.basename(lastLogResult.file).replace(/\.bin$/i, '') + ' - AP Tuner.param';
          if (url.searchParams.get('save') === '1') { const out = lastLogResult.file.replace(/\.bin$/i, '') + ' - AP Tuner.param'; fs.writeFileSync(out, txt); return json(res, 200, { ok: true, file: out }); }
          res.writeHead(200, { 'Content-Type': 'text/plain', 'Content-Disposition': `attachment; filename="${fname}"` }); return res.end(txt);
        }
        default: return json(res, 404, { error: 'unknown api' });
      }
    }
    // static
    let p = url.pathname === '/' ? '/index.html' : url.pathname;
    const file = path.join(WEB, path.normalize(p).replace(/^([.][.][/\\])+/, ''));
    if (!file.startsWith(WEB) || !fs.existsSync(file) || !fs.statSync(file).isFile()) { res.writeHead(404); res.end('not found'); return; }
    if (path.extname(file) === '.html') {
      // the page carries this run's token (only a page served from here can read it) and a strict CSP
      const html = fs.readFileSync(file, 'utf8').replace('__APTUNER_TOKEN__', TOKEN);
      res.writeHead(200, { 'Content-Type': MIME['.html'], 'Cache-Control': 'no-store', 'Content-Security-Policy': CSP, 'X-Frame-Options': 'DENY', 'X-Content-Type-Options': 'nosniff', 'Referrer-Policy': 'no-referrer' });
      return res.end(html);
    }
    res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream', 'Cache-Control': 'no-cache', 'X-Content-Type-Options': 'nosniff' });
    fs.createReadStream(file).pipe(res);
  } catch (e) {
    json(res, 500, { error: e.message });
  }
});

const SIZE_OSC = { small: 5, medium: 5, large: 3, heavy: 1.5 };
function sanitizeOpts(b) {
  const o = {};
  if (b.preset && PRESETS[b.preset]) o.preset = b.preset;
  if (b.autoApply !== undefined) o.autoApply = !!b.autoApply;
  if (Array.isArray(b.axes) && b.axes.length) o.axes = b.axes.filter(a => AXES.includes(a));
  if (b.observeSeconds) o.observeSeconds = Math.max(15, Math.min(120, +b.observeSeconds));
  if (b.size && SIZE_OSC[b.size]) { o.size = b.size; o.oscSmax = SIZE_OSC[b.size]; }
  if (b.oscSmax) o.oscSmax = Math.max(1, Math.min(20, +b.oscSmax));
  if (b.allowRamp !== undefined) o.allowRamp = !!b.allowRamp;
  if (b.stepMax) o.stepMax = Math.max(1.1, Math.min(2, +b.stepMax));
  if (b.watchSeconds) o.watchSeconds = Math.max(2, Math.min(15, +b.watchSeconds));
  if (b.inFlightWrites !== undefined) o.inFlightWrites = b.inFlightWrites === true;
  return o;
}
function setOpts(b) {
  const o = sanitizeOpts(b);
  if (o.inFlightWrites !== undefined && o.inFlightWrites !== !!tuner.opts.inFlightWrites) tuner.log('tuner', o.inFlightWrites ? 'changes in flight turned ON (experimental, tested only in the simulator)' : 'changes in flight turned off');
  Object.assign(tuner.opts, o);
  return o;
}

server.on('error', e => { if (e.code === 'EADDRINUSE') { console.log('AP Tuner is already running on port ' + PORT); process.exit(0); } else { console.error(e); process.exit(1); } });
server.listen(PORT, '127.0.0.1', () => {
  console.log(`AP Tuner running at http://127.0.0.1:${PORT}${APP_MODE ? ' (app mode)' : ''}`);
  if (process.argv.includes('--open')) { require('child_process').exec(`start "" http://127.0.0.1:${PORT}`); }
});
