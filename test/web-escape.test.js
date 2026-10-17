// The web UI shows text from logs and from the vehicle only as text (2026-10-17 audit F-90). The real page scripts
// (web/app.js, flight.js, ground.js) run in a sandbox with a minimal fake DOM; they get snapshots, analyses and a
// log result in which every string carries an HTML payload, and no innerHTML they produce may contain it as markup.
// Also: no inline event handlers or inline scripts (the page runs under a Content-Security-Policy).
'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const WEB = path.join(__dirname, '..', 'web');
const P = '"\'><img src=x onerror=alert(1)>';
const html = [];           // every innerHTML written
const errors = [];         // anything the page scripts threw or logged as an error
let failed = 0;
const ok = (cond, what) => { if (cond) console.log('  ok    ' + what); else { failed++; console.log('  FAIL  ' + what); } };

// ---------------- a minimal DOM ----------------
const byId = new Map();
function el(tag = 'div', id = null) {
  const e = {
    tagName: tag.toUpperCase(), id, children: [], style: {}, dataset: {}, className: '', textContent: '', value: '', checked: false, disabled: false, options: [], clientWidth: 800, scrollTop: 0, scrollHeight: 0,
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    _html: '',
    get innerHTML() { return this._html; }, set innerHTML(v) { this._html = String(v); html.push(this._html); },
    appendChild(c) { this.children.push(c); return c; }, removeChild(c) { this.children = this.children.filter(x => x !== c); return c; },
    get firstChild() { return this.children[0] || null; }, get parentElement() { return el(); },
    querySelector(s) { return find(s); }, querySelectorAll() { return []; },
    addEventListener() {}, removeEventListener() {}, scrollIntoView() {}, closest() { return null; }, remove() {}, setAttribute() {}, click() {},
  };
  return e;
}
function find(sel) {
  if (sel === 'meta[name="aptuner-token"]') return { content: 'f'.repeat(48) };
  if (!byId.has(sel)) { const e = el('div', sel); if (sel === 'input[name=mode]:checked' || sel === 'input[name=fmode]:checked') e.value = 'advise'; byId.set(sel, e); }
  return byId.get(sel);
}
const listeners = {};
class EventSource { constructor() {} addEventListener(name, fn) { (listeners[name] = listeners[name] || []).push(fn); } }
class uPlot { constructor() {} setData() {} setSize() {} }
class ResizeObserver { constructor() {} observe() {} }
const document = { querySelector: find, querySelectorAll: () => [], getElementById: id => find('#' + id), createElement: t => el(t), addEventListener() {} };
const sandbox = {
  document, window: { addEventListener() {}, dispatchEvent() {} }, Event: class { constructor(n) { this.type = n; } }, EventSource, uPlot, ResizeObserver,
  fetch: async () => ({ ok: true, json: async () => ({}) }), setTimeout: () => 0, setInterval: () => 0, clearInterval() {}, clearTimeout() {},
  console: { log() {}, warn() {}, error: (...a) => errors.push(a.map(String).join(' ')) },
};
vm.createContext(sandbox);
for (const f of ['app.js', 'flight.js', 'ground.js']) {
  try { vm.runInContext(fs.readFileSync(path.join(WEB, f), 'utf8'), sandbox, { filename: f }); } catch (e) { errors.push(`${f}: ${e.stack}`); }
}
const emit = (name, data) => { for (const fn of listeners[name] || []) { try { fn({ data: JSON.stringify(data) }); } catch (e) { errors.push(`${name}: ${e.stack}`); } } };
const call = (expr, arg) => { try { sandbox.__arg = arg; vm.runInContext(expr + '(__arg)', sandbox); } catch (e) { errors.push(`${expr}: ${e.stack}`); } };

// ---------------- hostile data ----------------
const issue = { level: 'bad', text: P, fix: P };
const axisRes = axis => ({ axis, at: 1, issues: [issue], warnings: [P], error: null, current: { P: 0.1, I: 0.1, D: 0.003, FF: 0, angleP: 4.5 },
  diagnostics: { grade: { letter: 'B', score: 80, notes: [P], gradable: true }, trackingPct: 70, rmsErrorDegs: 5, oscRmsDegs: 1, noiseRmsDegs: 0.2, saturationPct: 0, srateMax: 1 },
  identification: { reason: P, fit: null }, ffEvents: { events: 3, ff: null, reason: P },
  proposal: { gains: { P: 0.12, I: 0.12, D: 0.004, FF: 0, angleP: 5 }, changes: [{ key: 'P', from: 0.1, to: 0.12, pct: 20 }], note: P, margins: { gmDb: 6, pmDeg: 40, ms: 1.8 }, overshootPct: 10 },
  dataInfo: { seconds: 30, fs: 50, rawRateHz: 50 } });
const slim = a => ({ issues: a.issues, grade: a.diagnostics.grade, diagnostics: a.diagnostics, proposal: { changes: a.proposal.changes, margins: a.proposal.margins, overshootPct: 10 }, measuredStep: null, error: P, current: a.current, warnings: [P] });
const series = { t: [1, 2], target: [0, 0], actual: [0, 0], P: [0, 0], I: [0, 0], D: [0, 0], FF: [0, 0], srate: [0, 0], dmod: [1, 1] };
const gains = { roll: { P: 0.1, I: 0.1, D: 0.003, FF: 0, angleP: 4.5 }, pitch: { P: 0.1, I: 0.1, D: 0.003, FF: 0, angleP: 4.5 }, yaw: { P: 0.2, I: 0.02, D: 0, FF: 0, angleP: 4.5 } };
function snapshot(flightPhase, groundPhase) {
  const checkup = { summary: { anyProposal: true, changes: 1, bad: 1, warn: 0, worst: 'B' }, axes: { roll: slim(axisRes('roll')), [P]: slim(axisRes('pitch')) }, gains, vehicleIssues: [issue], verdict: { ok: false, items: [{ ok: false, text: P }] }, vibe: null, hover: 0.4, notch: false, t: 5 };
  return {
    alive: true, connected: true, linkDesc: P, auto: { running: false, current: P, tried: [] }, prepared: true, family: 'copter', state: 'paused', info: P, stateSince: 0, axis: 'roll', round: 0,
    vehicle: { type: 2, typeName: 'MAV_TYPE_' + P, modeName: P, armed: true }, attitude: { roll: 1, pitch: 2 }, rates: { roll: { hz: 50 }, pitch: { hz: 50 }, yaw: { hz: 50 } },
    gains, originals: gains, previous: {}, opts: { observeSeconds: 30, inFlightWrites: false, axes: ['roll'] }, journal: [{ t: 1, name: P, from: 1, to: 2, reason: P }], messages: [],
    monitor: { active: true, axes: { roll: { state: P, srateMax: 1, samples: 10 } }, attitude: { active: false } }, vibe: { x: 1, y: 1, z: 1, clip: 0 },
    series: { roll: series, pitch: series, yaw: series },
    flight: { phase: flightPhase, info: P, pausedInfo: P, since: 0, opts: { axes: ['roll', P], observeSeconds: 30 }, mode: P, armed: true, modeOk: true, goodModes: P, inFlight: false,
      checkup, overall: checkup, filters: { done: true, result: 'esc', applied: { [P]: P }, status: { mode: P, freq: P } }, resumePhase: 'tuning',
      summary: { reason: P, axes: ['roll'], before: { roll: { grade: P, score: P } }, after: { roll: { grade: 'A', score: 90 } }, gainsStart: gains, gainsEnd: gains, writes: P, filters: { result: P }, notes: [{ text: P }] },
      notes: [{ text: P }], tunerState: 'observing', tunerInfo: P, tunerRound: 0, tunerAxis: P, writes: 3 },
    ground: { phase: groundPhase, info: P, since: 0, opts: { axes: ['roll', P], seconds: 30 }, mark: { tStart: 1, tEnd: 31, mode: P }, savedMark: null, mode: P, armed: false, modeOk: true, alive: true, linkDesc: P, usb: null, progress: { done: 1, total: 2 },
      result: { family: P, firmware: P, chosenAxes: ['roll'], axes: { roll: Object.assign(slim(axisRes('roll')), { dataInfo: { seconds: 30, rawRateHz: 400 } }) } },
      rows: [{ group: P, name: P, value: 1, from: 2, vehicle: 2 }], writes: [{ name: P, from: 1, to: 2, ok: false, error: P }, { name: P, from: 1, to: 2, ok: true, undoError: true }], files: {}, notes: [{ text: P }], error: P },
  };
}

// ---------------- feed it ----------------
for (const [fp, gp] of [['review', 'review'], ['tuning', 'done'], ['filters', 'error'], ['land', 'usb'], ['finished', 'captured'], ['paused', 'review'], ['preflight', 'idle'], ['checkup', 'flight']]) emit('snapshot', snapshot(fp, gp));
emit('analysis', axisRes('roll'));
emit('analysis', Object.assign(axisRes('pitch'), { error: P }));
emit('analysis', Object.assign(axisRes(P), {}));
emit('log', { t: 1, kind: P, text: P });
emit('param', { name: P, from: 1, to: 2, reason: P });
emit('connect', { trying: P });
call('renderLogResult', { file: 'C:\\logs\\' + P + '.bin', firmware: P, family: P, flightWindow: { tStart: 1, tEnd: 2 }, paramChanges: P, axes: { roll: axisRes('roll'), pitch: Object.assign(axisRes('pitch'), { error: P }) }, rows: [{ group: P, name: P, label: P, value: 1, from: 2 }],
  noise: { available: true, source: P, note: P, peaks: [{ f: P, prominence: P }], harmonics: { f0: P, members: [{ k: P }] }, rmsBand: { pre: P, postCurrent: P }, recommended: { params: { [P]: P }, gyroFilterHz: P, why: P, phaseLag10Hz: { current: P, proposed: P }, notes: [P], rmsBandPost: P, postPredicted: [1] }, f: [1], amp: [1], postCurrent: [1] } });
call('renderNoise', { available: false, reason: P });
call('noVehicleHtml', { tried: [{ label: P, result: P }], busyPorts: [P], missionPlanner: false });
call('$("#modalBody").innerHTML = noVehicleHtml', { tried: [{ label: P, result: P }], busyPorts: [P], missionPlanner: true });

// ---------------- verdicts ----------------
ok(errors.length === 0, 'the page scripts ran without errors' + (errors.length ? ':\n' + errors.slice(0, 5).join('\n') : ''));
ok(html.length > 40, `the renderers produced HTML (${html.length} innerHTML writes)`);
const raw = html.filter(h => h.includes('<img'));
ok(raw.length === 0, 'no payload rendered as markup' + (raw.length ? ': ' + raw[0].slice(0, 300) : ''));
ok(html.some(h => h.includes('&lt;img src=x onerror=alert(1)&gt;')), 'the payload is shown as text');
const markup = fs.readFileSync(path.join(WEB, 'index.html'), 'utf8');
ok(!/\son[a-z]+\s*=/i.test(markup.replace(/<script[\s\S]*?<\/script>/g, '')), 'index.html has no inline event handlers');
ok(!/<script(?![^>]*\ssrc=)[^>]*>/i.test(markup), 'index.html has no inline scripts');
for (const f of ['app.js', 'flight.js', 'ground.js']) ok(!/\son(click|load|error|change|input|submit|mouse\w+)\s*=\s*["']/i.test(fs.readFileSync(path.join(WEB, f), 'utf8')), `${f} writes no inline event handlers`);
console.log(failed ? `${failed} check(s) failed` : 'web escaping test passed');
process.exit(failed ? 1 : 0);
