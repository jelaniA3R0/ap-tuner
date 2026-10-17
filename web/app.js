/* AP Tuner web UI */
'use strict';
const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];
// every text that comes from a log, the vehicle or the server is escaped before it goes into HTML (F-90)
const ESC = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' };
const esc = v => (v === undefined || v === null) ? '' : String(v).replace(/[&<>"']/g, c => ESC[c]);
// the per-run token the server wrote into this page: every state-changing request carries it (F-05)
const TOKEN = (document.querySelector('meta[name="aptuner-token"]') || {}).content || '';
const api = async (p, body, opts = {}) => {
  const r = await fetch('/api/' + p, body !== undefined ? Object.assign({ method: 'POST', headers: { 'Content-Type': 'application/json', 'X-APTuner-Token': TOKEN }, body: JSON.stringify(body) }, opts) : opts);
  const j = await r.json(); if (!r.ok) throw new Error(j.error || r.statusText); return j;
};
const f3 = v => (v === undefined || v === null || isNaN(v)) ? '-' : (+(+v).toPrecision(4)).toString();
const fmtPct = p => p === null || p === undefined ? '' : (p > 0 ? '+' : '') + Math.round(p) + '%';
const num = (v, d) => (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '-';
function toast(msg, kind = '') { const d = document.createElement('div'); d.className = 'toast ' + kind; d.textContent = msg; $('#toasts').appendChild(d); setTimeout(() => d.remove(), 6000); }
const guard = fn => async (...a) => { try { await fn(...a); } catch (e) { toast(e.message, 'bad'); } };
function modal(title, html, { yes = 'OK', no = 'Cancel', danger = false } = {}) {
  return new Promise(res => {
    $('#modalTitle').textContent = title; $('#modalBody').innerHTML = html; $('#modalYes').textContent = yes; $('#modalNo').textContent = no;
    $('#modalYes').className = 'btn ' + (danger ? 'danger' : 'primary'); $('#modalNo').style.display = no ? '' : 'none';
    $('#modal').classList.add('open');
    const done = v => { $('#modal').classList.remove('open'); $('#modalYes').onclick = $('#modalNo').onclick = null; res(v); };
    $('#modalYes').onclick = () => done(true); $('#modalNo').onclick = () => done(false);
  });
}

// ---------- views & panels ----------
$$('nav.views button').forEach(b => b.onclick = () => { $$('nav.views button').forEach(x => x.classList.toggle('active', x === b)); $$('.view').forEach(v => v.classList.toggle('active', v.id === 'view-' + b.dataset.view)); window.dispatchEvent(new Event('resize')); });
// links that switch views (no inline handlers: the page runs under a Content-Security-Policy)
document.addEventListener('click', e => { const a = e.target && e.target.closest ? e.target.closest('[data-goview]') : null; if (!a) return; e.preventDefault(); const b = $(`nav.views button[data-view=${a.dataset.goview}]`); if (b) b.click(); if (a.dataset.gopanel) openPanel(a.dataset.gopanel, true); });
const PANELS = ['connect', 'vehicle', 'tune', 'notch', 'save'];
$$('.panel > header').forEach(h => h.onclick = () => h.parentElement.classList.toggle('open'));
$$('.step').forEach(s => s.onclick = () => { openPanel(s.dataset.step, true); });
function openPanel(name, scroll) { const p = $('#p-' + name); p.classList.add('open'); if (scroll) p.scrollIntoView({ behavior: 'smooth', block: 'start' }); }
let activeStep = 'connect';
function setActive(name) {
  if (activeStep === name) return;
  activeStep = name;
  PANELS.forEach(n => { const p = $('#p-' + n); p.classList.toggle('active', n === name); if (n === name) p.classList.add('open'); });
  $$('.step').forEach(s => s.classList.toggle('active', s.dataset.step === name));
}
function setDone(name, done) { $('#p-' + name).classList.toggle('done', !!done); $(`.step[data-step="${name}"]`).classList.toggle('done', !!done); }
function setStatus(name, text, cls = '') { const el = $('#st-' + name); el.textContent = text; el.className = 'status ' + cls; }

// ---------- charts ----------
const COLORS = ['#38bdf8', '#f97316', '#22c55e', '#facc15', '#a78bfa', '#ef4444', '#94a3b8'];
function mkChart(el, series, opts = {}) {
  const h = opts.height || 170, w = Math.max(300, el.clientWidth - 20);
  const u = new uPlot({ width: w, height: h, scales: { x: { time: false, distr: opts.logx ? 3 : 1 }, y: { distr: opts.logy ? 3 : 1 } }, axes: [{ stroke: '#93a0b4', grid: { stroke: '#223049' }, label: opts.xLabel, labelSize: 14 }, { stroke: '#93a0b4', grid: { stroke: '#223049' }, size: 54 }], cursor: { show: true }, legend: { show: true },
    series: [{ label: opts.xLabel || 't' }, ...series.map((s, i) => Object.assign({ label: s, stroke: COLORS[i % COLORS.length], width: 1.4 }, opts.dash && opts.dash[i] ? { dash: [4, 4] } : {}))] }, [[], ...series.map(() => [])], el);
  let lastW = w;
  new ResizeObserver(() => { const nw = Math.max(300, el.clientWidth - 20); if (Math.abs(nw - lastW) > 4) { lastW = nw; u.setSize({ width: nw, height: h }); } }).observe(el);
  return u;
}
const chAtt = mkChart($('#chAtt'), ['roll demanded', 'roll', 'pitch demanded', 'pitch', 'yaw demanded', 'yaw'], { height: 190, dash: [true, false, true, false, true, false] });
const chOsc = mkChart($('#chOsc'), ['SRate roll', 'SRate pitch', 'SRate yaw', 'Dmod'], { height: 190 });
function resampleTo(t, ts, vs) { if (!ts || !ts.length) return t.map(() => null); let j = 0; return t.map(x => { while (j < ts.length - 1 && ts[j + 1] <= x) j++; return vs[j]; }); }

// ---------- state from the server ----------
let snap = null; const analyses = {}; let prepared = false; let presetName = 'normal'; let sizeName = 'medium';
const es = new EventSource('/api/events');
es.addEventListener('snapshot', e => { snap = JSON.parse(e.data); render(); });
es.addEventListener('analysis', e => { const r = JSON.parse(e.data); analyses[r.axis] = r; renderAxisCard($('#axesPanel'), r, true); });
es.addEventListener('log', e => { const m = JSON.parse(e.data); addLine('#messages', `${new Date(m.t).toLocaleTimeString()}  ${m.kind}: ${m.text}`); });
es.addEventListener('param', e => { const p = JSON.parse(e.data); addLine('#journal', `${new Date().toLocaleTimeString()}  ${p.name}: ${f3(p.from)} → ${f3(p.to)}   (${p.reason})`); });
es.addEventListener('progress', e => { const p = JSON.parse(e.data); $('#prepText').textContent = `reading ${p.done}/${p.total}`; });
es.addEventListener('connect', e => { const p = JSON.parse(e.data); if (p.trying) { addTry(p.trying, 'busy', 'trying…'); } if (p.found) { markTry(p.found, 'ok', 'connected'); } });
function addLine(sel, text) { const el = $(sel); const d = document.createElement('div'); d.textContent = text; el.appendChild(d); while (el.children.length > 300) el.removeChild(el.firstChild); el.scrollTop = el.scrollHeight; }
function addTry(label, cls, txt) { const ul = $('#autoList'); $$('li .tick.busy', ul).forEach(t => { t.className = 'tick todo'; t.textContent = '–'; t.parentElement.querySelector('.r').textContent = 'no answer'; }); const li = document.createElement('li'); li.dataset.label = label; li.innerHTML = `<span class="tick ${esc(cls)}">${cls === 'ok' ? '✓' : '…'}</span><span style="flex:1">${esc(label)}</span><span class="r muted">${esc(txt)}</span>`; ul.appendChild(li); }
function markTry(label, cls, txt) { const li = $$('#autoList li').find(l => l.dataset.label === label); if (!li) return; const t = $('.tick', li); t.className = 'tick ' + cls; t.textContent = cls === 'ok' ? '✓' : '–'; $('.r', li).textContent = txt; }

// ---------- changes in flight: one switch (shown in the Flight and Advanced views), off by default ----------
async function setInFlight(on) {
  if (on && !(await modal('Allow changes in flight?', '<p><b>Experimental, tested only in the simulator.</b> It has never been tried on a real aircraft.</p><p class="muted">With this on, the app writes gains and the notch filter to the vehicle while it flies. It watches every change and tries to undo it if the vehicle starts to oscillate, but parameters are stored on the vehicle as they are written and an undo can fail (for example when the link drops). Only in calm air, at a safe height, with a hand on the mode switch, ready to land.</p>', { yes: 'Allow changes in flight', danger: true }))) { renderInFlight(); return; }
  await api('options', { inFlightWrites: !!on }); toast(on ? 'changes in flight allowed (experimental)' : 'changes in flight off: the app only measures while armed', on ? 'bad' : 'ok');
}
function renderInFlight() { const on = !!(snap && snap.opts && snap.opts.inFlightWrites); $$('.inflight-toggle').forEach(c => { c.checked = on; }); }
$$('.inflight-toggle').forEach(c => c.onchange = guard(async () => { await setInFlight(c.checked); }));

function monitorText(s) {
  const m = s.monitor; if (!m || !m.axes) return 'Oscillation monitor: not running';
  const parts = ['roll', 'pitch', 'yaw'].map(a => m.axes[a] ? `${a} ${m.axes[a].state}${m.axes[a].samples ? ' (slew rate ' + num(m.axes[a].srateMax, 1) + ')' : ''}` : '').filter(Boolean);
  return `Oscillation monitor ${m.active ? 'on (acts while a session runs)' : 'watching (no session)'}: ${parts.join(' · ')} · attitude check ${m.attitude && m.attitude.active ? 'on' : 'off (no attitude target from the vehicle)'}`;
}

function render() {
  const s = snap; if (!s) return;
  // connection chip
  const chip = $('#connChip'), txt = $('#connText');
  if (s.alive) { chip.className = 'chip on'; txt.textContent = `${s.vehicle.typeName.replace('MAV_TYPE_', '').toLowerCase()} · ${s.vehicle.modeName} · ${s.vehicle.armed ? 'ARMED' : 'disarmed'} · ${s.linkDesc}`; }
  else if (s.auto && s.auto.running) { chip.className = 'chip busy'; txt.textContent = 'looking for a vehicle: ' + (s.auto.current || ''); }
  else if (s.connected) { chip.className = 'chip busy'; txt.textContent = 'connected to ' + s.linkDesc + ', waiting for the vehicle'; }
  else { chip.className = 'chip'; txt.textContent = 'Not connected'; }
  $('#btnDisconnect').classList.toggle('hidden', !s.connected);
  $('#btnConnect').textContent = s.connected ? 'Reconnect' : 'Connect';
  prepared = !!s.prepared;
  renderInFlight();
  // steps
  setDone('connect', s.alive); setStatus('connect', s.alive ? 'connected' : s.connected ? 'waiting for heartbeat' : 'not connected', s.alive ? 'ok' : s.connected ? 'busy' : '');
  setDone('vehicle', prepared); setStatus('vehicle', prepared ? `${s.family} ready` : s.alive ? 'press Read vehicle settings' : 'waiting for connection', prepared ? 'ok' : s.alive ? 'warn' : '');
  const tuneCls = { observing: 'busy', analyzing: 'busy', applying: 'busy', ramping: 'busy', proposed: 'warn', paused: 'bad', done: 'ok' }[s.state] || '';
  setStatus('tune', s.state === 'idle' || s.state === 'ready' ? (prepared ? 'ready' : 'waiting') : s.state, tuneCls); setDone('tune', s.state === 'done');
  const changed = s.journal.some(j => !/GCS_PID_MASK|SMAX/.test(j.name));
  setStatus('save', changed ? `${s.journal.length} changes` : 'nothing changed yet', changed ? 'warn' : '');
  if (!s.alive) setActive('connect'); else if (!prepared) setActive('vehicle'); else if (['observing', 'analyzing', 'applying', 'ramping', 'proposed', 'paused'].includes(s.state)) setActive('tune'); else if (s.state === 'done') setActive('notch'); else if (activeStep === 'connect' || activeStep === 'vehicle') setActive('tune');
  // vehicle card
  if (s.vehicle) {
    const r = s.rates; $('#vehicleKv').innerHTML = kv([['type', s.vehicle.typeName.replace('MAV_TYPE_', '').toLowerCase() + (s.family ? ` (${s.family})` : '')], ['mode', `${s.vehicle.modeName} · ${s.vehicle.armed ? 'armed' : 'disarmed'}`], ['link', s.linkDesc], ['attitude', `roll ${num(s.attitude.roll, 1)}°, pitch ${num(s.attitude.pitch, 1)}°`], ['PID telemetry', `${r.roll.hz} Hz per axis` + (r.roll.hz < 20 && prepared ? ' (slow link: the measurement is less reliable)' : '')]]);
  }
  if (s.gains && prepared) $('#gainsNow').innerHTML = gainsTable(s.gains, s.originals);
  // tune banner
  const info = { idle: 'Press Start, then fly in Stabilize / AltHold (plane: FBWA) and give short, sharp stick taps every few seconds.', ready: 'Press Start, then fly and tap the sticks.', observing: 'Fly: short sharp stick taps on ' + (s.axis || '') + '. Collecting data…', analyzing: 'Computing…', proposed: 'Look at the proposal below and press Apply when you agree (or Resume to keep observing).', applying: 'Writing the new gains step by step and watching for oscillation. Keep flying calmly.', paused: 'Paused. Fix what the message says, then press Resume.', done: 'All chosen axes are tuned. Go to step 4 for the notch filter, or step 5 to save.', ramping: 'QuikTune-style ramp: raising a gain slowly until the oscillation detector trips, then backing off.' }[s.state] || '';
  $('#tuneState').textContent = (s.state === 'idle' ? 'Not started' : s.state.charAt(0).toUpperCase() + s.state.slice(1)) + (s.info ? ' · ' + s.info : '');
  $('#tuneInfo').textContent = info;
  const banner = $('#tuneBanner'); banner.className = 'banner ' + ({ paused: 'bad', proposed: 'ok', done: 'ok', applying: 'warn' }[s.state] || '');
  const prog = s.state === 'observing' ? Math.min(100, 100 * ((Date.now() - (s.stateSince || Date.now())) / 1000) / (s.opts.observeSeconds || 30)) : (s.state === 'done' ? 100 : 0);
  $('#tuneProg').style.width = prog + '%';
  const mon = $('#monitorLine'); if (mon) { mon.textContent = monitorText(s); const worst = s.monitor && s.monitor.axes ? Object.values(s.monitor.axes).map(a => a.state) : []; mon.className = 'small ' + (worst.includes('oscillating') ? 'bad' : worst.includes('unsettled') ? 'warn' : 'muted'); }
  // charts
  const R = s.series.roll, P = s.series.pitch, Y = s.series.yaw;
  const t = R.t.length ? R.t : P.t.length ? P.t : Y.t; const t0 = t.length ? t[t.length - 1] : 0; const rel = arr => arr.map(v => +(v - t0).toFixed(2));
  chAtt.setData([rel(t), resampleTo(t, R.t, R.target), resampleTo(t, R.t, R.actual), resampleTo(t, P.t, P.target), resampleTo(t, P.t, P.actual), resampleTo(t, Y.t, Y.target), resampleTo(t, Y.t, Y.actual)]);
  const A = s.series[s.axis] || R;
  chOsc.setData([rel(t), resampleTo(t, R.t, R.srate), resampleTo(t, P.t, P.srate), resampleTo(t, Y.t, Y.srate), resampleTo(t, A.t, A.dmod)]);
  // axis cards: refresh live gains + undo availability
  for (const ax of ['roll', 'pitch', 'yaw']) { const b = $(`#live-${ax} [data-undo]`); if (b) b.disabled = !(s.previous && s.previous[ax]); }
}
function kv(pairs) { return pairs.map(([k, v]) => `<b>${esc(k)}</b><span>${esc(v)}</span>`).join(''); }
function gainsTable(g, o) {
  const keys = ['P', 'I', 'D', 'FF', 'FLTT', 'FLTE', 'FLTD', 'SMAX', 'angleP', 'tconst'];
  return `<table class="gains"><tr><th>axis</th>${keys.map(k => `<th>${k}</th>`).join('')}</tr>` + ['roll', 'pitch', 'yaw'].map(a => g[a] ? `<tr><td>${a}</td>${keys.map(k => { const v = g[a][k], ov = o && o[a] ? o[a][k] : undefined; const ch = ov !== undefined && v !== undefined && Math.abs(ov - v) > 1e-6 * Math.max(1, Math.abs(ov)); return `<td class="${ch ? (v > ov ? 'up' : 'down') : ''}" title="${ch ? 'was ' + f3(ov) : ''}">${v === undefined ? '' : f3(v)}</td>`; }).join('')}</tr>` : '').join('') + '</table>';
}

// ---------- axis cards (live + log) ----------
function marginBar(label, value, unit, lo, hi, invert) {
  if (value === undefined || value === null || isNaN(value)) return `<span>${esc(label)}</span><div class="bar"></div><span class="v">-</span>`;
  const good = invert ? value <= lo : value >= hi, bad = invert ? value > hi : value < lo;
  const col = good ? 'var(--ok)' : bad ? 'var(--bad)' : 'var(--warn)';
  const span = invert ? Math.min(100, 100 * (value / (hi * 1.5))) : Math.min(100, 100 * value / (hi * 1.6));
  return `<span>${esc(label)}</span><div class="bar"><i style="width:${num(span, 1)}%;background:${col}"></i></div><span class="v" style="color:${col}">${num(+value, unit === '°' ? 0 : 1)}${esc(unit)}</span>`;
}
function renderAxisCard(panel, r, live) {
  const axis = ['roll', 'pitch', 'yaw'].includes(r.axis) ? r.axis : 'roll';
  const id = (live ? 'live-' : 'log-') + axis;
  let card = document.getElementById(id);
  if (!card) { card = document.createElement('div'); card.id = id; card.className = 'card axis'; panel.appendChild(card); }
  if (r.error) { card.innerHTML = `<div class="head"><h3>${axis}</h3></div><div class="bad">${esc(r.error)}</div>`; return; }
  const d = r.diagnostics || {}; const g = d.grade || {}; const cur = r.current || {}; const p = r.proposal; const idn = r.identification || {};
  const letter = /^[A-F]$/.test(g.letter || '') ? g.letter : '';
  const rows = ['P', 'I', 'D', 'FF', 'FLTT', 'FLTE', 'FLTD', 'IMAX', 'angleP', 'tconst'].filter(k => cur[k] !== undefined && !(k === 'FF' && !cur.FF && !(p && p.gains && p.gains.FF)) && !(k === 'IMAX' && !(p && p.gains && p.gains.IMAX !== undefined && Math.abs(p.gains.IMAX - cur.IMAX) > 1e-9)));
  const tbl = `<table class="gains"><tr><th>gain</th><th>now</th><th>proposed</th><th>change</th></tr>` + rows.map(k => { const a = cur[k], b = p && p.gains ? p.gains[k] : undefined; const ch = (b !== undefined && a !== undefined && Math.abs(b - a) > 1e-9) ? (a ? fmtPct((b / a - 1) * 100) : 'new') : ''; return `<tr class="${ch ? 'changed' : ''}"><td>${k}</td><td>${f3(a)}</td><td>${b !== undefined ? f3(b) : ''}</td><td class="${ch.startsWith('-') ? 'down' : ch ? 'up' : ''}">${ch}</td></tr>`; }).join('') + '</table>';
  const td = r.timeDomain;
  const modelTxt = idn.fit && idn.confidence === 'lower' ? `Lower-confidence model from ordinary hover data (no stick taps): ${num(idn.fit.K / 57.2958, 2)} rad/s² of rate change per unit of controller output, damping ${num(idn.fit.a, 2)} /s` + (td && td.uLimit ? `, the mixer gives at most ${esc(td.uLimit)} of output on this axis` : '') + (td ? `, steady trim ${esc(td.trimMean)} (${esc(td.trimMin)} to ${esc(td.trimMax)})` : '') + `. It replays ${td ? Math.round(Math.max(td.vaf, td.r2) * 100) : '?'}% of the measured rate; control power known to +-${td ? Math.round(td.bRelErr * 100) : '?'}%.` + (p && p.modelDesign ? ` Full model design: rate P ${f3(p.modelDesign.P)}, I ${f3(p.modelDesign.I)}, angle P ${f3(p.modelDesign.angleP)}${p.modelDesign.headroomNote ? ' (' + esc(p.modelDesign.headroomNote) + ')' : ''}; one round moves at most 2x towards it.` : '')
    : idn.fit ? `The airframe answers the controller with ${num(idn.fit.K, 0)} deg/s² per unit of output, damping ${num(idn.fit.a, 1)} /s` + (idn.assumed ? `, lag ${num(idn.assumed.tau * 1000, 0)} ms and delay ${num(idn.assumed.Td * 1000, 0)} ms` + (idn.assumed.calibrated ? ' (checked against the measured closed loop' + (idn.assumed.closedLoopMismatch ? `, ${num(idn.assumed.closedLoopMismatch.magPct, 0)}% / ${num(idn.assumed.closedLoopMismatch.phaseDeg, 0)}° off` : '') + ')' : ' (assumed)') : '') + `. Model confidence ${num(idn.fit.fitQuality * 100, 0)}%, valid up to ${num(idn.fCoh, 1)} Hz from ${esc(idn.nWin)} data windows.` : (idn.reason ? 'No model yet: ' + esc(idn.reason) + '.' : '');
  const m = p && p.margins ? `<div class="bars">${marginBar('gain margin', p.margins.gmDb, ' dB', 5, 7)}${marginBar('phase margin', p.margins.pmDeg, '°', 33, 40)}${marginBar('sensitivity', p.margins.ms, '', 2.0, 2.4, true)}${marginBar('overshoot', p.overshootPct, '%', 10, 15, true)}</div>` : '';
  const ms = r.measuredStep ? `measured response: rises in ${num(r.measuredStep.metrics.riseTime * 1000, 0)} ms, overshoots ${num(r.measuredStep.metrics.overshootPct, 0)}%, settles ${num(r.measuredStep.metrics.settlingTime * 1000, 0)} ms (${esc(r.measuredStep.count)} windows)` : '';
  const pm = p && p.predicted ? `predicted with the proposal: rises in ${num(p.predicted.proposedMetrics.riseTime * 1000, 0)} ms, overshoots ${num(p.predicted.proposedMetrics.overshootPct, 0)}% (now: ${num(p.predicted.currentMetrics.riseTime * 1000, 0)} ms / ${num(p.predicted.currentMetrics.overshootPct, 0)}%)` : '';
  const diag = `tracking ${d.trackingPct !== null && d.trackingPct !== undefined ? num(d.trackingPct, 0) + '%' : 'n/a'} · error ${num(d.rmsErrorDegs, 1)} deg/s · oscillation band ${num(d.oscRmsDegs, 1)} deg/s · noise ${num(d.noiseRmsDegs, 2)} ` + (d.dOverP !== undefined ? ` · D/P ${num(d.dOverP, 2)}` : '') + (d.saturationPct !== undefined ? ` · saturation ${num(d.saturationPct, 1)}%` : '') + (d.srateMax !== undefined ? ` · SRate max ${num(d.srateMax, 1)}` : '');
  card.innerHTML = `<div class="head"><div class="grade ${letter}">${letter || '?'}</div><div style="flex:1"><h3>${axis}</h3><div class="muted small">${r.dataInfo ? `${num(r.dataInfo.seconds, 0)} s at ${esc(r.dataInfo.fs)} Hz · ` : ''}${g.gradable === false ? 'not gradable' : 'score ' + esc(g.score ?? '-') + '/100'}</div></div><span class="pill ${p ? (p.partial || p.lowConfidence ? 'warn' : 'ok') : 'warn'}">${p ? (p.partial ? 'partial proposal' : p.lowConfidence ? 'lower-confidence proposal' : 'proposal ready') : 'no proposal'}</span></div>
    ${(r.issues || []).filter(i => i.level === 'bad').map(i => `<div class="bad small">✖ ${esc(i.text)}${i.fix ? ' <b>What to do:</b> ' + esc(i.fix) : ''}</div>`).join('')}
    <div class="muted small">${diag}</div>
    ${(g.notes || []).length ? '<ul class="notes">' + g.notes.map(n => `<li>${esc(n)}</li>`).join('') + '</ul>' : ''}
    <p class="explain">${modelTxt}</p>
    ${r.ffEvents ? `<p class="explain">Feed-forward from ${esc(r.ffEvents.events)} stick events: ${r.ffEvents.ff ? num(r.ffEvents.ff, 3) + ' (middle half ' + num(r.ffEvents.p25, 3) + '-' + num(r.ffEvents.p75, 3) + ')' : esc(r.ffEvents.reason)}.</p>` : ''}
    ${(r.warnings || []).map(w => `<div class="warn small">⚠ ${esc(w)}</div>`).join('')}
    ${p && p.note ? `<div class="warn small">⚠ ${esc(p.note)}</div>` : ''}
    ${tbl}${m}
    ${pm ? `<div class="muted small">${pm}</div>` : ''}${ms ? `<div class="muted small">${ms}</div>` : ''}
    <div class="minichart" id="${id}-step"></div>${idn.bode ? `<div class="minichart" id="${id}-bode"></div>` : ''}
    ${live ? `<div class="row" style="margin-top:10px">${p ? `<button class="btn primary" data-apply="${axis}">Apply ${axis} proposal</button>` : ''}<button class="btn ghost" data-undo="${axis}" ${snap && snap.previous && snap.previous[axis] ? '' : 'disabled'}>Undo last apply</button></div>` : ''}`;
  if (live && p) $('[data-apply]', card).onclick = guard(async () => {
    const list = p.changes.map(c => `<tr class="changed"><td>${esc(c.key)}</td><td>${f3(c.from)}</td><td>${f3(c.to)}</td><td>${fmtPct(c.pct)}</td></tr>`).join('');
    const armed = !!(snap && snap.vehicle && snap.vehicle.armed);
    if (armed && !(snap.opts && snap.opts.inFlightWrites)) { await modal('Changes in flight are off', '<p>The vehicle is armed. While it flies the app only measures: land and press Apply on the ground, or turn on <b>Allow changes in flight</b> (experimental, tested only in the simulator).</p>', { yes: 'OK', no: '' }); return; }
    const ok = await modal(`Apply the ${axis} proposal?`, `<p class="muted">${armed ? 'These parameters are written to the flying vehicle in small steps, each watched for oscillation. If the vehicle starts to oscillate the app pauses and tries to undo the change; that is a safety net, not a guarantee (experimental, tested only in the simulator).' : 'These parameters are written to the vehicle (disarmed).'} You can Undo afterwards.</p><table class="gains"><tr><th>gain</th><th>now</th><th>new</th><th>change</th></tr>${list}</table>${m}`, { yes: 'Apply now', danger: armed });
    if (!ok) return; await api('apply', { axis }); toast('applying ' + axis + ' gains in steps', 'ok');
  });
  if (live) $('[data-undo]', card).onclick = guard(async () => { if (await modal(`Undo the last ${axis} apply?`, '<p class="muted">The gains from before the last apply are written back.</p>', { yes: 'Undo' })) { await api('undo', { axis }); toast(axis + ' gains restored to the previous set', 'ok'); } });
  const stepEl = $(`#${id}-step`); const series = [], data = [];
  if (p && p.predicted) { data.push(p.predicted.t.map(v => +v.toFixed(3))); series.push('predicted: current gains'); data.push(p.predicted.current); series.push('predicted: proposed gains'); data.push(p.predicted.proposed); }
  if (r.measuredStep) { const t = r.measuredStep.t; if (!data.length) data.push(t.map(v => +v.toFixed(3))); series.push('measured'); data.push(resampleTo(data[0], t, r.measuredStep.y)); }
  if (data.length) { const u = mkChart(stepEl, series, { height: 160, xLabel: 'step response, seconds (1 = demanded rate)' }); u.setData(data); }
  if (idn.bode) { const el = $(`#${id}-bode`); const b = idn.bode; const u = mkChart(el, ['measured airframe response', 'model', 'coherence ×100'], { height: 160, xLabel: 'Hz (response of the airframe to the controller output)', logx: true, logy: true }); u.setData([b.f, b.magMeas.map(v => v > 0 ? v : null), b.magFit, b.coh.map(v => v * 100 || null)]); }
}

// ---------- connect ----------
// what to tell the user when autoconnect finds nothing (Mission Planner owning the radio is the usual cause)
function noVehicleHtml(r) {
  const mirror = '<ol class="notes"><li>In Mission Planner press <b>Ctrl+F</b>, then click <b>Mavlink</b> (left column).</li><li>Pick <b>TCP Host - 14550</b>, tick <b>Write access</b>, press <b>Connect</b>, then <b>OK</b> on the port box.</li><li>Press <b>Connect</b> here again.</li></ol>';
  const busy = (r.busyPorts || []).map(esc);
  const tried = `<p class="muted small">Tried: ${(r.tried || []).map(t => esc(t.label) + (t.result && t.result !== 'no heartbeat' ? ' (' + esc(t.result) + ')' : '')).join(', ')}</p>`;
  if (r.missionPlanner || busy.length) {
    return `<p><b>${r.missionPlanner ? 'Mission Planner is open' : 'Another program is using ' + busy.join(', ')}</b>${busy.length ? ' and is using the telemetry / USB port (' + busy.join(', ') + ')' : ''}. Only one program can use that port, so AP Tuner has to get the data through Mission Planner's mirror:</p>${mirror}<p class="muted small">Or close Mission Planner and press Connect: AP Tuner then opens the radio itself.</p>${tried}`;
  }
  return `<p>Nothing answered on the usual connections. Check:</p><ul class="notes"><li>Telemetry radio or USB cable plugged into this PC and the autopilot powered (the radio link LED solid).</li><li>Using Mission Planner? Open it, connect to the vehicle, then turn on its mirror:</li></ul>${mirror}<ul class="notes"><li>Simulator: run <code>sitl\\run-sitl.cmd</code> and <code>sitl\\fly test quad.cmd</code>.</li><li>Bluetooth links are not tried automatically: use Connection options (manual).</li></ul>${tried}`;
}

$('#btnConnect').onclick = $('#btnAuto').onclick = guard(async () => {
  $('#autoList').innerHTML = ''; $('#autoText').textContent = 'looking…'; $('#btnAuto').disabled = true;
  try { const r = await api('autoconnect', {}); if (r.ok) { $('#autoText').textContent = 'connected: ' + r.found.label; toast('Connected: ' + r.found.label, 'ok'); setTimeout(() => openPanel('vehicle', true), 600); } else { $('#autoText').textContent = 'no vehicle found'; $$('#autoList li .tick.busy').forEach(t => { t.className = 'tick todo'; t.textContent = '–'; }); await modal('No vehicle found', noVehicleHtml(r), { yes: 'OK', no: '' }); } }
  finally { $('#btnAuto').disabled = false; }
});
$('#btnDisconnect').onclick = guard(async () => { await api('disconnect', {}); });
$('#btnPorts').onclick = guard(async () => { const l = await api('ports'); $('#serial').innerHTML = l.length ? l.map(p => `<option value="${esc(p.port)}">${esc(p.name)}</option>`).join('') : '<option value="">no serial ports</option>'; });
$('#btnManual').onclick = guard(async () => { const kind = $('#kind').value; await api('connect', { kind, host: $('#host').value, port: kind === 'serial' ? $('#serial').value : $('#port').value, baud: $('#baud').value }); toast('connecting…'); });
$('#kind').onchange = () => { if ($('#kind').value === 'serial' && $('#serial').options.length <= 1) $('#btnPorts').click(); };

// ---------- vehicle ----------
$$('#sizeSeg button').forEach(b => b.onclick = guard(async () => { $$('#sizeSeg button').forEach(x => x.classList.toggle('active', x === b)); sizeName = b.dataset.size; await api('options', { size: sizeName }); }));
$('#btnPrepare').onclick = guard(async () => {
  $('#prepText').textContent = 'reading…';
  let r = await api('prepare', { confirm: false });
  if (r.needsConfirm) {
    const ok = await modal('Two settings need to change first', `<p class="muted">To tune live the app needs these housekeeping values. They do not change how the vehicle flies.</p><ul class="notes">${r.writes.map(w => `<li><b>${esc(w.name)}</b>: ${f3(w.from)} → ${f3(w.to)}<br><span class="muted">${esc(w.why)}</span></li>`).join('')}</ul>`, { yes: 'Set them and continue' });
    if (!ok) { $('#prepText').textContent = 'cancelled (settings unchanged)'; return; }
    r = await api('prepare', { confirm: true });
  }
  $('#prepText').textContent = `done: ${r.family}, loop ${r.loopHz} Hz`;
  $('#checkList').innerHTML = r.checks.map(c => `<li><span class="tick ${c.ok ? '' : 'warn'}">${c.ok ? '✓' : '!'}</span><div><b>${esc(c.title)}</b><div class="muted small">${esc(c.detail)}</div></div></li>`).join('');
  toast('Vehicle read: ' + r.family, 'ok'); setTimeout(() => openPanel('tune', true), 500);
});

// ---------- tune ----------
const PRESET_TXT = { conservative: 'Smooth: wider safety margins, gentler response. For large, flexible or payload aircraft.', normal: 'Normal: the margins ArduPilot\'s own tuners aim for.', aggressive: 'Sporty: tighter margins, faster response. For small agile aircraft only.' };
$$('#presetSeg button').forEach(b => b.onclick = guard(async () => { $$('#presetSeg button').forEach(x => x.classList.toggle('active', x === b)); presetName = b.dataset.preset; $('#presetExplain').textContent = PRESET_TXT[presetName]; await api('options', { preset: presetName }); }));
const opts = () => ({ preset: presetName, size: sizeName, autoApply: $('input[name=mode]:checked').value === 'auto', axes: $$('.ax:checked').map(x => x.value) });
$$('input[name=mode]').forEach(r => r.onchange = guard(async () => { $('#swAuto').classList.toggle('on', $('input[name=mode]:checked').value === 'auto'); $('#swAdvise').classList.toggle('on', false); await api('options', opts()); }));
$$('.ax').forEach(c => c.onchange = guard(async () => api('options', opts())));
$('#btnStart').onclick = guard(async () => {
  if (!prepared) { toast('Read the vehicle first (step 2)', 'bad'); openPanel('vehicle', true); return; }
  if (opts().autoApply && !(await modal('Auto-apply is on', '<p>The app writes gains to the vehicle by itself: in steps of at most 1.5×, capped at 2× per round. While the vehicle is armed this needs <b>Allow changes in flight</b> (experimental, tested only in the simulator); every change is then watched and the app tries to undo it if the vehicle starts to oscillate. Keep a hand on the mode switch.</p>', { yes: 'Start with auto-apply', danger: true }))) return;
  await api('start', opts()); toast('Session started: fly and tap the sticks', 'ok');
});
$('#btnPause').onclick = guard(async () => api('pause', {}));
$('#btnResume').onclick = guard(async () => { await api('options', opts()); await api('resume', {}); });
$('#btnStop').onclick = guard(async () => api('stop', {}));
$('#btnRestore').onclick = guard(async () => { if (await modal('Restore the original gains?', '<p>All rate, filter and angle gains go back to what the vehicle had when you connected (in steps of at most 1.5×). While the vehicle is armed this needs Allow changes in flight.</p>', { yes: 'Restore', danger: true })) { await api('restore', {}); toast('original gains restored', 'ok'); } });

// ---------- notch ----------
let fftTimer = null;
$('#btnNotchStatus').onclick = guard(async () => {
  $('#notchText').textContent = 'checking…';
  const st = await api('notch/status');
  $('#notchText').textContent = '';
  const cur = st.current;
  const curTxt = st.notchActive ? `Notch is on: mode ${esc(cur.INS_HNTCH_MODE)} (${['fixed', 'throttle', 'RPM sensor', 'ESC telemetry', 'in-flight FFT', 'RPM sensor 2'][cur.INS_HNTCH_MODE] || '?'}), ${esc(cur.INS_HNTCH_FREQ)} Hz, bandwidth ${esc(cur.INS_HNTCH_BW)} Hz, harmonics ${esc(cur.INS_HNTCH_HMNCS)}` : 'No harmonic notch is configured yet.';
  $('#notchCards').innerHTML = `<div class="card"><h3>Now on the vehicle</h3><p>${curTxt}</p><p class="muted small">Gyro low-pass INS_GYRO_FILTER = ${esc(cur.INS_GYRO_FILTER)} Hz · FFT ${st.fftCapable ? (st.fftEnabled ? 'enabled' : 'available, off') : 'not available in this firmware'} · ESC RPM telemetry ${st.escTelemetry ? 'yes (' + Math.round(st.escRpm) + ' rpm)' : 'no'}</p>${st.note ? `<p class="warn small">${esc(st.note)}</p>` : ''}</div>` +
    `<div class="card"><h3>Options</h3><ul class="list">${st.options.map(o => `<li><span class="tick ${o.id === 'log' ? 'todo' : ''}">${o.id === 'log' ? '…' : '✓'}</span><div style="flex:1"><b>${esc(o.title)}</b><div class="muted small">${esc(o.detail)}</div>${o.id === 'esc' ? `<button class="btn primary" style="margin-top:6px" data-esc>Set notch to ESC RPM tracking</button>` : ''}${o.id === 'fft' ? `<button class="btn" style="margin-top:6px" data-fft>Open the FFT steps</button>` : ''}${o.id === 'log' ? `<button class="btn ghost" style="margin-top:6px" data-log>Go to Log file</button>` : ''}</div></li>`).join('')}</ul></div>`;
  const esc_ = $('[data-esc]'); if (esc_) esc_.onclick = guard(async () => { const armed = !!(snap && snap.vehicle && snap.vehicle.armed); if (await modal('Set the notch to ESC RPM tracking?', `<p class="muted">Parameters to write:</p><ul class="notes">${Object.entries(st.escProposal).map(([k, v]) => `<li>${esc(k)} = ${esc(v)}</li>`).join('')}</ul>${armed ? '<p class="warn small">The vehicle is armed: this needs Allow changes in flight. Every axis is then watched for 10 s and the notch is switched back if the vehicle starts to oscillate (experimental).</p>' : ''}`, { yes: 'Write them', danger: armed })) { await api('notch/esc', {}); toast('notch set to ESC RPM tracking', 'ok'); $('#btnNotchStatus').click(); } });
  const fft = $('[data-fft]'); if (fft) fft.onclick = () => { $('#fftFlow').style.display = ''; $('#fftFlow').scrollIntoView({ behavior: 'smooth' }); };
  const lg = $('[data-log]'); if (lg) lg.onclick = () => $('nav.views button[data-view=log]').click();
  setStatus('notch', st.notchActive ? 'notch on' : 'not configured', st.notchActive ? 'ok' : 'warn');
});
$('#btnFftEnable').onclick = guard(async () => { await api('notch/fft/enable', {}); toast('FFT_ENABLE set to 1: reboot the autopilot (disarmed) before flying', 'ok'); });
$('#btnReboot').onclick = guard(async () => { if (await modal('Reboot the autopilot?', '<p>Only while disarmed, on the ground.</p>', { yes: 'Reboot', danger: true })) { await api('notch/reboot', {}); toast('reboot sent; reconnect in ~20 s'); } });
$('#btnFftStart').onclick = guard(async () => { await api('notch/fft/start', {}); let s = 30; $('#fftTimer').textContent = s + ' s'; clearInterval(fftTimer); fftTimer = setInterval(() => { s--; $('#fftTimer').textContent = (s > 0 ? s + ' s' : 'press Stop'); if (s <= 0) clearInterval(fftTimer); }, 1000); toast('measuring: hold a steady hover', 'ok'); });
$('#btnFftStop').onclick = guard(async () => { clearInterval(fftTimer); $('#fftTimer').textContent = ''; const r = await api('notch/fft/stop', {}); const ch = Object.entries(r.changed || {}); $('#fftResult').innerHTML = (r.ok ? '<b class="ok">The autopilot wrote the notch:</b> ' : r.held === false ? '<b class="bad">The notch the autopilot wrote made the vehicle oscillate and was switched back.</b> ' : '<b class="warn">No notch written</b> (needs a steady hover and clear motor noise). ') + (ch.length ? '<ul class="notes">' + ch.map(([k, c]) => `<li>${esc(k)}: ${f3(c.from)} → ${f3(c.to)}</li>`).join('') + '</ul>' : '') + (r.messages.length ? '<div class="muted small">' + r.messages.map(m => esc(m.text)).join('<br>') + '</div>' : ''); if (r.ok) { toast('notch configured by the autopilot', 'ok'); setStatus('notch', 'notch on', 'ok'); } });

// ---------- save ----------
async function refreshRows() { const r = await api('live/rows'); if (!r.rows.length) { $('#liveRows').innerHTML = '<p class="muted">Read the vehicle first.</p>'; return; } $('#liveRows').innerHTML = rowsTable(r.rows); }
function rowsTable(rows) { let group = null; let html = '<table class="gains"><tr><th>parameter</th><th>value</th><th>was</th><th>change</th></tr>'; for (const r of rows) { if (r.group !== group) { group = r.group; html += `<tr><td colspan="4" style="text-align:left;color:var(--accent);font-weight:600;padding-top:10px">${esc(group)}</td></tr>`; } const ch = r.from !== undefined && Math.abs(r.from - r.value) > 1e-6 * Math.max(1, Math.abs(r.from)); html += `<tr class="${ch ? 'changed' : ''}"><td>${esc(r.name)}<span class="muted small"> ${r.label && r.label !== r.name ? esc(r.label) : ''}</span></td><td>${f3(r.value)}</td><td>${r.from !== undefined ? f3(r.from) : ''}</td><td class="${ch ? (r.value > r.from ? 'up' : 'down') : ''}">${ch && r.from ? fmtPct((r.value / r.from - 1) * 100) : ''}</td></tr>`; } return html + '</table>'; }
$('#btnRefreshRows').onclick = guard(refreshRows);
$('#p-save > header').addEventListener('click', () => { if ($('#p-save').classList.contains('open')) refreshRows().catch(() => {}); });

// ---------- log file ----------
const drop = $('#drop');
drop.onclick = () => $('#file').click();
drop.ondragover = e => { e.preventDefault(); drop.classList.add('over'); }; drop.ondragleave = () => drop.classList.remove('over');
drop.ondrop = e => { e.preventDefault(); drop.classList.remove('over'); const f = e.dataTransfer.files[0]; if (f) uploadLog(f); };
$('#file').onchange = e => { const f = e.target.files[0]; if (f) uploadLog(f); };
const uploadLog = guard(async (f) => {
  if (!/\.bin$/i.test(f.name)) { toast('Please choose a .bin log', 'bad'); return; }
  $('#logBusy').textContent = `uploading ${f.name} (${(f.size / 1048576).toFixed(0)} MB)…`; $('#logResult').innerHTML = '';
  const r = await fetch(`/api/log/upload?name=${encodeURIComponent(f.name)}&preset=${encodeURIComponent($('#logPreset').value)}&family=${encodeURIComponent($('#logFamily').value)}`, { method: 'POST', body: f, headers: { 'X-APTuner-Token': TOKEN } });
  const j = await r.json(); if (!r.ok) throw new Error(j.error || r.statusText);
  $('#logBusy').textContent = ''; renderLogResult(j);
});
$('#btnLogBrowse').onclick = guard(async () => { const l = await api('log/list'); $('#logList').innerHTML = l.length ? l.map(x => `<div data-p="${esc(x.path)}"><span>${esc(x.path)}</span><span class="muted">${(x.size / 1048576).toFixed(0)} MB · ${new Date(x.mtime).toLocaleDateString()}</span></div>`).join('') : '<div class="muted">no .bin logs found on D:\\, Documents\\Mission Planner\\logs, Desktop or Downloads</div>'; $$('#logList div[data-p]').forEach(d => d.onclick = () => { $('#logPath').value = d.dataset.p; $('#btnLogAnalyze').click(); }); });
$('#btnLogAnalyze').onclick = guard(async () => {
  const p = $('#logPath').value.trim(); if (!p) { toast('Choose a log first', 'bad'); return; }
  $('#logBusy').textContent = 'analysing…'; $('#logResult').innerHTML = '';
  const r = await api('log/analyze', { path: p, preset: $('#logPreset').value, family: $('#logFamily').value || null });
  $('#logBusy').textContent = ''; renderLogResult(r);
});
function renderLogResult(r) {
  const gl = a => { const l = r.axes[a] && r.axes[a].diagnostics && r.axes[a].diagnostics.grade ? r.axes[a].diagnostics.grade.letter : null; return /^[A-F]$/.test(l || '') ? l : null; };
  const grades = ['roll', 'pitch', 'yaw'].map(a => gl(a) ? `${a} <span class="grade ${gl(a)}" style="width:28px;height:28px;font-size:14px;display:inline-grid;vertical-align:middle">${gl(a)}</span>` : '').filter(Boolean).join(' &nbsp; ');
  const nProp = Object.values(r.axes).filter(a => a && a.proposal).length;
  $('#logResult').innerHTML = `<div class="banner ok"><div class="ico">📄</div><div style="flex:1"><b>${esc(String(r.file || '').split(/[\\/]/).pop())}</b><span>${esc(r.firmware || '')} · ${esc(r.family)} · flight ${num(r.flightWindow && r.flightWindow.tStart, 0)}-${num(r.flightWindow && r.flightWindow.tEnd, 0)} s · ${esc(r.paramChanges)} parameter changes during the log · grades: ${grades}</span></div><a class="btn primary lg" href="/api/log/param" download>Download parameter file</a></div>
    ${nProp ? '' : '<div class="banner warn"><div class="ico">⚠</div><div><b>No gain proposal from this log</b><span>Usually because there was no stick excitation, the data rate is too low, or the vehicle only hovered. The report still shows what the flight looked like.</span></div></div>'}
    <div class="cards cols-auto" id="logAxes"></div>
    <div id="logNoise" class="card" style="margin-top:12px"></div>
    <div class="card" style="margin-top:12px"><h3>Parameter file contents</h3><p class="muted small">Load in Mission Planner: CONFIG → Full Parameter List → Load from file → Write Params. Fly carefully afterwards, ready to switch back to your previous parameters.</p>${r.rows && r.rows.length ? rowsTable(r.rows) : '<p class="muted">nothing to write</p>'}</div>`;
  for (const a of ['roll', 'pitch', 'yaw']) if (r.axes[a]) renderAxisCard($('#logAxes'), Object.assign({}, r.axes[a], { axis: a }), false);
  renderNoise(r.noise);
}
function renderNoise(nz) {
  const el = $('#logNoise');
  if (!nz || !nz.available) { el.innerHTML = `<h3>Gyro noise and notch filter</h3><div class="warn small">${nz ? esc(nz.reason) : ''}</div>`; return; }
  const rec = nz.recommended;
  el.innerHTML = `<h3>Gyro noise and notch filter</h3><div class="muted small">${esc(nz.source)}${nz.note ? ' · ' + esc(nz.note) : ''}</div>
    <p class="explain">Peaks: ${(nz.peaks || []).map(p => `${esc(p.f)} Hz (×${esc(p.prominence)})`).join(', ') || 'none'}${nz.harmonics ? ` · motor fundamental ${esc(nz.harmonics.f0)} Hz with harmonics ` + nz.harmonics.members.map(m => esc(m.k) + '×').join(', ') : ''}. Control-band (5-40 Hz) gyro noise: ${esc(nz.rmsBand.pre)} deg/s measured${nz.rmsBand.postCurrent !== null ? ', ' + esc(nz.rmsBand.postCurrent) + ' after the current filters' : ''}${rec ? ', <b>' + esc(rec.rmsBandPost) + ' predicted with the recommended notch</b>' : ''}.</p>
    ${rec ? `<div class="banner"><div class="ico">🎯</div><div><b>Recommended: ${Object.entries(rec.params).map(([k, v]) => esc(k.replace('INS_HNTCH_', '')) + ' ' + esc(v)).join(' · ')} · INS_GYRO_FILTER ${esc(rec.gyroFilterHz)}</b><span>${esc(rec.why)}. Phase lag at 10 Hz: ${esc(rec.phaseLag10Hz.current)}° now → ${esc(rec.phaseLag10Hz.proposed)}° with the notch.</span></div></div><ul class="notes">${rec.notes.map(n => `<li>${esc(n)}</li>`).join('')}</ul>` : `<p class="muted">${esc(nz.recommendation || '')}</p>`}
    <div class="minichart" id="noiseChart"></div>`;
  const ce = $('#noiseChart'); const series = ['gyro (measured)']; const data = [nz.f, nz.amp];
  if (nz.postCurrent) { series.push('after current filters'); data.push(nz.postCurrent); }
  if (rec) { series.push('predicted with recommended notch'); data.push(rec.postPredicted); }
  const vals = data.slice(1).flat().filter(v => v > 0); const span = vals.length ? Math.max(...vals) / Math.max(1e-6, Math.min(...vals)) : 1;
  const u = mkChart(ce, series, { height: 260, xLabel: 'Hz', logy: span > 100 }); u.setData(data.map((a, i) => i === 0 ? a : a.map(v => (v >= 0 ? v : null))));
}

// initial: open help texts collapsed, show connect panel
setActive('connect');
