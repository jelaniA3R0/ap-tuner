/* AP Tuner: the "Telemetry + USB" screen (fly on telemetry, write over USB after landing). Uses helpers from app.js. */
'use strict';
const G_ORDER = ['connect', 'fly', 'land', 'usb', 'review', 'write'];
const G_OF = { idle: 'connect', flight: 'fly', capture: 'fly', captured: 'land', usb: 'usb', download: 'usb', review: 'review', writing: 'write', done: 'write', stopped: 'connect', error: null };
const G_TITLE = { idle: 'Telemetry + USB', flight: 'Take off and hover', capture: 'Capturing 30 s', captured: 'Captured: land now', usb: 'Plug in the USB cable', download: 'Reading the flight log', review: 'What the app found', writing: 'Writing', done: 'Written', stopped: 'Stopped', error: 'Something went wrong' };
const gAxes = () => $$('.gax:checked').map(x => x.value);
let gLastPhase = null, gLastKey = '';

const gStart = guard(async () => {
  if (!gAxes().length) { toast('Choose at least one axis', 'bad'); return; }
  if (!snap || !snap.alive) {
    toast('Connecting over telemetry first…');
    const r = await api('autoconnect', {});
    if (!r.ok) { await modal('No telemetry found', noVehicleHtml(r), { yes: 'OK', no: '' }); return; }
  }
  await api('ground/start', { axes: gAxes(), preset: presetName, seconds: 30 });
  toast('Started: take off and hover', 'ok');
});
const gUsb = (useSaved) => guard(async () => { await api('ground/usb', { useSaved }); toast('Plug the USB cable into the autopilot', 'ok'); });
const gWrite = guard(async () => {
  const g = snap.ground; const names = $$('.grow:checked').map(x => x.value);
  if (!names.length) { toast('Tick at least one change', 'bad'); return; }
  const rows = g.rows.filter(r => names.includes(r.name)).map(r => `<tr class="changed"><td>${esc(r.name)}</td><td>${f3(r.vehicle ?? r.from)}</td><td>${f3(r.value)}</td></tr>`).join('');
  const ok = await modal('Write these values to the autopilot?', `<p class="muted">The vehicle must stay disarmed. The current values are saved to an UNDO file first, and every value is read back after writing.</p><table class="gains"><tr><th>parameter</th><th>now</th><th>new</th></tr>${rows}</table>`, { yes: 'Write to the autopilot' });
  if (!ok) return;
  const r = await api('ground/write', { names });
  const bad = r.results.filter(x => !x.ok).length;
  toast(bad ? `${bad} value(s) need attention` : 'All values written and read back', bad ? 'bad' : 'ok');
});
const gUndo = guard(async () => { if (await modal('Write the previous values back?', '<p>Every value this session changed goes back to what it was before.</p>', { yes: 'Undo' })) { await api('ground/undo', {}); toast('Previous values written back', 'ok'); } });
const gStop = guard(async () => { await api('ground/stop', {}); });

function gButtons(g) {
  const box = $('#gbtns'); box.innerHTML = '';
  const add = (label, cls, fn) => box.appendChild(fbtn(label, cls, fn));
  switch (g.phase) {
    case 'idle': case 'stopped': case 'error': case 'done':
      add(g.phase === 'done' ? 'Start another flight' : 'Start', 'primary lg', gStart);
      if (g.phase === 'done' && g.writes.some(w => w.ok && !w.same && !w.undone)) add('Undo (write the old values back)', '', gUndo);
      if (g.savedMark && g.phase !== 'done') add(`Landed already: use the 30 s marked at ${new Date(g.savedMark.savedAt).toLocaleTimeString()}`, '', gUsb(true));
      if (g.phase !== 'done') add('No telemetry: go straight to USB', 'ghost', gUsb(false));
      break;
    case 'flight': add('No telemetry: go straight to USB', 'ghost', gUsb(false)); add('Stop', 'ghost', gStop); break;
    case 'capture': add('Stop', 'ghost', gStop); break;
    case 'captured': add('Landed: go to USB', 'primary', gUsb(false)); add('Capture again', '', guard(() => api('ground/again', {}))); add('Stop', 'ghost', gStop); break;
    case 'usb': case 'download': add('Stop', 'ghost', gStop); break;
    case 'review': if (g.rows.length) add('Write to the autopilot', 'good lg', gWrite); add('Stop', 'ghost', gStop); break;
    default: add('Stop', 'ghost', gStop);
  }
}

function gAxisCard(ax, a) {
  if (!a) return '';
  const p = a.proposal; const gr = a.grade || {};
  const rows = p ? p.changes.map(c => `<tr class="changed"><td>${esc(c.key)}</td><td>${f3(c.from)}</td><td>${f3(c.to)}</td><td class="${(c.pct || 0) < 0 ? 'down' : 'up'}">${fmtPct(c.pct)}</td></tr>`).join('') : '';
  const bars = p && p.margins ? `<div class="bars">${marginBar('gain margin', p.margins.gmDb, ' dB', 4, 6)}${marginBar('phase margin', p.margins.pmDeg, '°', 25, 35)}${marginBar('overshoot', p.overshootPct, '%', 12, 20, true)}</div>` : '';
  const di = a.dataInfo ? `<div class="muted small">${num(a.dataInfo.seconds, 0)} s of log data at ${num(a.dataInfo.rawRateHz, 0)} Hz</div>` : '';
  return `<div class="card axis"><div class="head"><div class="grade ${/^[A-F]$/.test(gr.letter || '') ? gr.letter : ''}">${/^[A-F]$/.test(gr.letter || '') ? gr.letter : '?'}</div><div style="flex:1"><h3>${esc(ax)}</h3>${di}</div><span class="pill ${p ? 'ok' : 'warn'}">${p ? (p.changes.length ? p.changes.length + ' change(s)' : 'no change needed') : 'no proposal'}</span></div>
    ${issueList(a.issues || [])}${a.error ? `<div class="bad small">${esc(a.error)}</div>` : ''}${(a.warnings || []).map(w => `<div class="warn small">⚠ ${esc(w)}</div>`).join('')}
    ${rows ? `<table class="gains"><tr><th>gain</th><th>in the log</th><th>new</th><th>change</th></tr>${rows}</table>${bars}` : ''}</div>`;
}
function gRowsTable(g) {
  let group = null; let html = '<table class="gains"><tr><th></th><th>parameter</th><th>on the autopilot now</th><th>new</th></tr>';
  for (const r of g.rows) {
    if (r.group !== group) { group = r.group; html += `<tr><td colspan="4" class="muted small" style="padding-top:8px">${esc(group)}</td></tr>`; }
    html += `<tr class="changed"><td><input type="checkbox" class="grow" value="${esc(r.name)}" checked></td><td>${esc(r.name)}</td><td>${f3(r.vehicle ?? r.from)}</td><td>${f3(r.value)}</td></tr>`;
  }
  return html + '</table>';
}
const fileLinks = g => ['undo', 'param', 'bin'].filter(k => g.files && g.files[k]).map(k => `<a class="btn ghost" href="/api/ground/file?which=${k}" download>${k === 'undo' ? 'UNDO file' : k === 'param' ? 'Written values (.param)' : 'Log slice (.bin)'}</a>`).join('');

function gBody(g) {
  const key = g.phase + ':' + (g.writes || []).length + ':' + (g.result ? g.result.family : '') + ':' + (g.error || '');
  if (key === gLastKey && !['usb', 'download'].includes(g.phase)) return; gLastKey = key;
  const el = $('#gbody');
  switch (g.phase) {
    case 'idle': case 'stopped': el.innerHTML = `<div class="steps6">
      <div class="card"><b>1 · Connect over telemetry</b><span>Mission Planner connected over the radio as usual, with its mirror on (Ctrl+F → Mavlink → TCP Host 14550 → Write access → Connect). Press Start.</span></div>
      <div class="card"><b>2 · Fly and capture 30 s</b><span>Hover in QStabilize / QHover / QLoiter (copter: Stabilize / AltHold / Loiter). The 30 s capture starts by itself: short, sharp stick taps. Nothing is written.</span></div>
      <div class="card"><b>3 · Land</b><span>Land and disarm. The telemetry has only marked the time; the real data is in the autopilot's own log.</span></div>
      <div class="card"><b>4 · Plug in USB</b><span>Connect the USB cable to the autopilot. The app finds it by itself (Mission Planner can stay on the radio).</span></div>
      <div class="card"><b>5 · Download and analyse</b><span>The app pulls only the marked 30 s from the log at full rate (hundreds of samples a second) and works out the issues and the gains.</span></div>
      <div class="card"><b>6 · Review and write</b><span>You tick the changes and press Write. The old values are saved to an UNDO file first; each value is read back.</span></div></div>`; break;
    case 'flight': case 'capture': case 'captured': el.innerHTML = `<div class="card"><h3>${g.phase === 'capture' ? 'Keep tapping the sticks' : g.phase === 'captured' ? 'Captured' : 'Before the capture'}</h3><ul class="vt">
      <li>${ICON.ok}<div>Calm air, safe height, hand on the mode switch. Nothing is written in the air.</div></li>
      <li>${ICON.ok}<div>During the 30 s: short, sharp taps on ${esc(g.opts.axes.join(', '))}, hovering steady in between.</div></li>
      ${g.mark ? `<li>${ICON.ok}<div>Marked: ${num(g.mark.tEnd - g.mark.tStart, 0)} s in ${esc(g.mark.mode)}. Land and disarm when you are ready.</div></li>` : ''}</ul></div>`; break;
    case 'usb': case 'download': {
      const p = g.progress; const pct = p && p.total ? Math.round(100 * p.done / p.total) : 0;
      el.innerHTML = `<div class="card"><h3>${g.phase === 'usb' ? 'Plug the USB cable into the autopilot' : 'Reading the log over USB'}</h3><p class="muted">${esc(g.info)}</p>${g.phase === 'download' && p ? `<div class="progress" style="margin-top:8px"><i style="width:${pct}%"></i></div>` : ''}
        <ul class="notes">${g.notes.map(n => `<li>${esc(n.text)}</li>`).join('')}</ul></div>`; break;
    }
    case 'review': case 'writing': case 'done': {
      const r = g.result; if (!r) { el.innerHTML = ''; break; }
      let html = `<div class="banner ok"><div class="ico">📄</div><div style="flex:1"><b>${r.family === 'vtol' ? 'QuadPlane hover (Q_A_*)' : esc(r.family)} · ${esc(r.firmware || '')}</b><span>${g.notes.map(n => esc(n.text)).join(' · ')}</span></div><div class="row">${fileLinks(g)}</div></div>`;
      html += `<div class="cards cols-auto" style="margin-top:12px">${(r.chosenAxes || []).map(ax => gAxisCard(ax, r.axes[ax])).join('')}</div>`;
      if (g.phase === 'review') html += g.rows.length ? `<div class="card" style="margin-top:12px"><h3>Changes to write</h3><p class="muted small">Untick anything you do not want. These are one round of changes: fly again afterwards and repeat until the app says no change is needed.</p>${gRowsTable(g)}</div>` : `<div class="banner warn" style="margin-top:12px"><div class="ico">⚠</div><div><b>Nothing to write</b><span>Either the gains are already right, or there was too little stick input in the 30 s. See the cards above.</span></div></div>`;
      if (g.phase === 'done') html += `<div class="card" style="margin-top:12px"><h3>Written</h3><table class="gains"><tr><th>parameter</th><th>was</th><th>now</th><th></th></tr>${g.writes.map(w => `<tr class="${w.ok ? 'changed' : ''}"><td>${esc(w.name)}</td><td>${f3(w.from)}</td><td>${f3(w.to)}</td><td>${w.ok ? (w.same ? 'already set' : (w.undone ? 'undone' : (w.undoError ? '<span class="bad">undo failed</span>' : '✓ read back'))) : '<span class="bad">' + esc(w.error || 'failed') + '</span>'}</td></tr>`).join('')}</table><p class="muted small">Unplug the USB and fly again to check (run another capture). The UNDO file loads in Mission Planner: CONFIG → Full Parameter List → Load from file → Write Params.</p></div>`;
      el.innerHTML = html; break;
    }
    case 'error': el.innerHTML = `<div class="banner bad"><div class="ico">✖</div><div><b>${esc(g.error || 'error')}</b><span>${g.notes.map(n => esc(n.text)).join(' · ')}</span></div></div>`; break;
    default: el.innerHTML = '';
  }
}

function renderGround(s) {
  const g = s.ground; if (!g) return;
  const cur = G_OF[g.phase]; const ci = cur ? G_ORDER.indexOf(cur) : -1; const running = !['idle', 'stopped'].includes(g.phase);
  $$('#grail .ph').forEach(p => { const i = G_ORDER.indexOf(p.dataset.ph); p.classList.toggle('active', running ? i === ci : (p.dataset.ph === 'connect')); p.classList.toggle('done', running && (i < ci || (g.phase === 'done' && i <= ci))); p.classList.toggle('bad', g.phase === 'error'); });
  $('#gphase').textContent = g.phase === 'idle' ? (s.alive ? 'Connected: ' + (s.linkDesc || '') : 'Not connected') : g.phase;
  $('#gtitle').textContent = G_TITLE[g.phase] || g.phase;
  $('#ginfo').textContent = g.phase === 'idle' ? 'For a vehicle on a telemetry radio: the radio only marks the 30 s you fly; the gains come from the autopilot\'s full-rate log over USB after landing.' : g.info;
  const chips = [];
  if (s.alive) { chips.push(`<span class="chip good"><span class="dot"></span>${esc(g.linkDesc || '')}</span>`); chips.push(`<span class="chip ${g.modeOk ? 'good' : ''}"><span class="dot"></span>${esc(g.mode || '?')}${g.modeOk ? ' ✓' : ''}</span>`); chips.push(`<span class="chip ${g.armed ? 'good' : ''}"><span class="dot"></span>${g.armed ? 'armed' : 'disarmed'}</span>`); }
  else chips.push('<span class="chip"><span class="dot"></span>no link</span>');
  if (g.mark) chips.push(`<span class="chip good"><span class="dot"></span>30 s marked</span>`);
  $('#gchips').innerHTML = chips.join('');
  const el = (Date.now() - (g.since || Date.now())) / 1000;
  let prog = 0; if (g.phase === 'capture') prog = Math.min(100, 100 * el / (g.opts.seconds || 30)); else if (g.phase === 'download' && g.progress && g.progress.total) prog = 100 * g.progress.done / g.progress.total; else if (g.phase === 'done') prog = 100;
  $('#gprog').style.width = prog + '%';
  $('#gsetup').style.display = ['idle', 'stopped', 'done', 'error'].includes(g.phase) ? '' : 'none';
  const bk = g.phase + ':' + !!g.savedMark + ':' + (g.writes || []).length;
  if (bk !== gLastPhase) { gButtons(g); gLastPhase = bk; }
  gBody(g);
}
es.addEventListener('snapshot', () => { try { renderGround(snap); } catch (e) { console.error(e); } });
es.addEventListener('ground', e => { const p = JSON.parse(e.data); if (p.phase === 'captured') toast('30 s captured: land and disarm', 'ok'); if (p.phase === 'usb') toast('Plug the USB cable into the autopilot', 'ok'); if (p.phase === 'review') toast('Log analysed: check the proposed changes', 'ok'); });
