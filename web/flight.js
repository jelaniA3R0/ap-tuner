/* AP Tuner: the guided flight session screen (uses helpers and state from app.js) */
'use strict';
let fAuto = false;
const fAxes = () => $$('.fax:checked').map(x => x.value);
const PH_ORDER = ['setup', 'checkup', 'tuning', 'filters', 'overall', 'land'];
const PH_OF = { idle: 'setup', preflight: 'checkup', checkup: 'checkup', review: 'checkup', tuning: 'tuning', filters: 'filters', recheck: 'filters', overall: 'overall', land: 'land', finished: 'land', stopped: 'setup', paused: null };
const TITLE = { idle: 'Flight check', preflight: 'Take off', checkup: 'Check-up', review: 'What the app found', tuning: 'Tuning in flight', filters: 'Notch filter', recheck: 'Re-check with the filter', overall: 'Overall check', land: 'Land', finished: 'Done', stopped: 'Stopped', paused: 'Paused' };
const chFlight = mkChart($('#chFlight'), ['roll demanded', 'roll', 'pitch demanded', 'pitch'], { height: 180, dash: [true, false, true, false] });
const ICON = { bad: '<span class="tick" style="background:#3b0f0f;color:#fca5a5">✖</span>', warn: '<span class="tick warn">!</span>', ok: '<span class="tick">✓</span>' };
const AXIS = a => ['roll', 'pitch', 'yaw'].includes(a) ? a : esc(a);
const LETTER = l => /^[A-F]$/.test(l || '') ? l : '';
let lastPhase = null, lastBodyKey = '';

// setup controls (shared with the Advanced view through presetName / sizeName)
$$('#fpresetSeg button').forEach(b => b.onclick = guard(async () => { $$('#fpresetSeg button').forEach(x => x.classList.toggle('active', x === b)); presetName = b.dataset.preset; $('#fpresetExplain').textContent = PRESET_TXT[presetName]; $$('#presetSeg button').forEach(x => x.classList.toggle('active', x.dataset.preset === presetName)); await api('options', { preset: presetName }); }));
$$('#fsizeSeg button').forEach(b => b.onclick = guard(async () => { $$('#fsizeSeg button').forEach(x => x.classList.toggle('active', x === b)); sizeName = b.dataset.size; $$('#sizeSeg button').forEach(x => x.classList.toggle('active', x.dataset.size === sizeName)); await api('options', { size: sizeName }); }));
$$('input[name=fmode]').forEach(r => r.onchange = () => { fAuto = $('input[name=fmode]:checked').value === 'auto'; $('#fswAuto').classList.toggle('on', fAuto); });
const inFlightOn = () => !!(snap && snap.opts && snap.opts.inFlightWrites);

async function waitPrepared(ms) { const t0 = Date.now(); while (Date.now() - t0 < ms) { if (prepared) return true; await new Promise(r => setTimeout(r, 200)); } return prepared; }
const startFlight = guard(async () => {
  if (!snap || !snap.alive) { toast('Connect to the vehicle first', 'bad'); $('#btnAuto').click(); return; }
  if (!prepared) { await $('#btnPrepare').onclick(); if (!(await waitPrepared(4000))) return; }
  if (!fAxes().length) { toast('Choose at least one axis', 'bad'); return; }
  if (fAuto && !inFlightOn()) { toast('"Apply by itself" writes in flight: turn on "Allow changes in flight" first, or choose "Show me first"', 'bad'); return; }
  if (fAuto && !(await modal('Apply by itself', '<p>After the 30 s check-up the app writes the proposed gains to the flying vehicle without asking: small steps, each watched for oscillation; if the vehicle starts to oscillate the app pauses and tries to undo the change. Experimental, tested only in the simulator. Keep a hand on the mode switch.</p>', { yes: 'Start in this mode', danger: true }))) return;
  await api('flight/start', { axes: fAxes(), preset: presetName, size: sizeName, auto: fAuto, observeSeconds: 30 });
  toast('Flight session started: take off and hover', 'ok');
});
const applyInFlight = guard(async () => {
  const f = snap.flight; const rows = [];
  for (const [ax, a] of Object.entries(f.checkup.axes)) if (a.proposal) for (const c of a.proposal.changes) rows.push(`<tr class="changed"><td>${AXIS(ax)} ${esc(c.key)}</td><td>${f3(c.from)}</td><td>${f3(c.to)}</td><td>${fmtPct(c.pct)}</td></tr>`);
  const ok = await modal('Apply these changes in flight?', `<p class="muted">Written to the vehicle while it flies, in steps of at most 1.5×. Every change is watched for oscillation on every axis it affects; if the vehicle starts to oscillate the app pauses and tries to undo it. That is a safety net, not a guarantee: experimental, tested only in the simulator. The app then re-checks and continues until the gains settle.</p><table class="gains"><tr><th>gain</th><th>now</th><th>new</th><th>change</th></tr>${rows.join('')}</table>`, { yes: 'Apply in flight', danger: true });
  if (!ok) return; await api('flight/continue', {}); toast('applying in flight', 'ok');
});
const fbtn = (label, cls, fn) => { const b = document.createElement('button'); b.className = 'btn ' + cls; b.textContent = label; b.onclick = fn; return b; };
function buttons(f) {
  const box = $('#fbtns'); box.innerHTML = '';
  const stop = fbtn('Stop session', 'ghost', guard(async () => { if (await modal('Stop the session?', '<p>The gains stay as they are now. You can restore the originals in the Advanced view.</p>', { yes: 'Stop' })) await api('flight/stop', {}); }));
  switch (f.phase) {
    case 'idle': case 'finished': case 'stopped': box.appendChild(fbtn(f.phase === 'idle' ? 'Start flight check' : 'Start another flight', 'primary lg', startFlight)); break;
    case 'review': {
      if (f.checkup && f.checkup.summary.anyProposal && f.checkup.summary.changes) {
        if (f.inFlight) box.appendChild(fbtn('Apply in flight', 'good lg', applyInFlight));
        else box.appendChild(fbtn('Finish: apply on the ground', 'primary', guard(async () => { await api('flight/continue', {}); toast('Land, then apply the proposals on the ground (Advanced view) or use the Telemetry + USB tab', 'ok'); })));
      } else if (f.checkup && f.checkup.summary.anyProposal) box.appendChild(fbtn('Continue (nothing to change)', 'primary', guard(() => api('flight/continue', {}))));
      box.appendChild(fbtn('Repeat the check-up', '', guard(() => api('flight/repeat', {})))); box.appendChild(stop); break;
    }
    case 'tuning': case 'recheck':
      if (f.tunerState === 'paused') { box.appendChild(fbtn('Resume', 'primary', guard(() => api('flight/resume', {})))); box.appendChild(fbtn('Finish now', '', guard(() => api('flight/continue', {})))); }
      else box.appendChild(fbtn('Pause', '', guard(() => api('flight/pause', {}))));
      box.appendChild(stop); break;
    case 'filters': box.appendChild(fbtn('Skip the filter step', '', guard(() => api('flight/skip', {})))); box.appendChild(stop); break;
    case 'land': box.appendChild(fbtn('Finish without landing', '', guard(() => api('flight/continue', {})))); box.appendChild(stop); break;
    case 'paused': box.appendChild(fbtn('Resume', 'primary', guard(() => api('flight/resume', {})))); box.appendChild(fbtn('Finish now', '', guard(() => api('flight/continue', {})))); box.appendChild(stop); break;
    default: box.appendChild(stop);
  }
}
function issueList(items) { return `<ul class="issues">${items.map(i => `<li>${ICON[i.level] || ICON.warn}<div>${esc(i.text)}${i.fix ? `<span class="fix">${esc(i.fix)}</span>` : ''}</div></li>`).join('')}</ul>`; }
function axisCardF(ax, a, withProposal) {
  const g = a.grade || {}; const p = a.proposal;
  const rows = p ? p.changes.map(c => `<tr class="changed"><td>${esc(c.key)}</td><td>${f3(c.from)}</td><td>${f3(c.to)}</td><td class="${(c.pct || 0) < 0 ? 'down' : 'up'}">${fmtPct(c.pct)}</td></tr>`).join('') : '';
  const bars = p && p.margins ? `<div class="bars">${marginBar('gain margin', p.margins.gmDb, ' dB', 4, 6)}${marginBar('phase margin', p.margins.pmDeg, '°', 25, 35)}${marginBar('overshoot', p.overshootPct, '%', 12, 20, true)}</div>` : '';
  const pred = p && p.predicted ? `<div class="muted small">predicted step: ${num(p.predicted.current.riseTime * 1000, 0)} ms / ${num(p.predicted.current.overshootPct, 0)}% now → ${num(p.predicted.proposed.riseTime * 1000, 0)} ms / ${num(p.predicted.proposed.overshootPct, 0)}% with the changes</div>` : '';
  return `<div class="card axis"><div class="head"><div class="grade ${LETTER(g.letter)}">${LETTER(g.letter) || '?'}</div><div style="flex:1"><h3>${AXIS(ax)}</h3><div class="muted small">score ${esc(g.score ?? '-')}/100${a.measuredStep ? ` · measured step: ${num(a.measuredStep.riseTime * 1000, 0)} ms, ${num(a.measuredStep.overshootPct, 0)}% overshoot` : ''}</div></div>${withProposal ? `<span class="pill ${p ? 'ok' : 'warn'}">${p ? (p.changes.length ? p.changes.length + ' change(s)' : 'no change needed') : 'no proposal'}</span>` : ''}</div>
    ${issueList(a.issues || [])}${a.error ? `<div class="bad small">${esc(a.error)}</div>` : ''}${(a.warnings || []).map(w => `<div class="warn small">⚠ ${esc(w)}</div>`).join('')}
    ${withProposal && p && rows ? `<h4 style="margin-top:8px">Proposed changes</h4><table class="gains"><tr><th>gain</th><th>now</th><th>new</th><th>change</th></tr>${rows}</table>${bars}${pred}` : ''}</div>`;
}
function cmpTable(s) {
  const m = (v, d = 0, suf = '') => v === null || v === undefined || isNaN(v) ? '-' : (+v).toFixed(d) + suf;
  const rows = [];
  for (const ax of s.axes) {
    const b = s.before && s.before[ax], a = s.after && s.after[ax]; if (!b && !a) continue;
    const line = (label, vb, va, d, suf, lowerBetter) => { const cls = (vb !== null && vb !== undefined && va !== null && va !== undefined && !isNaN(vb) && !isNaN(va)) ? ((lowerBetter ? va < vb : va > vb) ? 'better' : (va === vb ? '' : 'worse')) : ''; rows.push(`<tr><td>${AXIS(ax)} ${label}</td><td>${m(vb, d, suf)}</td><td class="${cls}">${m(va, d, suf)}</td></tr>`); };
    rows.push(`<tr><td>${AXIS(ax)} grade</td><td>${b && b.grade ? LETTER(b.grade) + ' (' + esc(b.score) + ')' : '-'}</td><td>${a && a.grade ? LETTER(a.grade) + ' (' + esc(a.score) + ')' : '-'}</td></tr>`);
    line('follows the demand', b && b.trackingPct, a && a.trackingPct, 0, '%', false);
    line('rate error', b && b.rmsErrorDegs, a && a.rmsErrorDegs, 1, ' deg/s', true);
    line('oscillation band', b && b.oscRmsDegs, a && a.oscRmsDegs, 1, ' deg/s', true);
    if ((b && b.step) || (a && a.step)) { line('step rise', b && b.step && b.step.riseTime * 1000, a && a.step && a.step.riseTime * 1000, 0, ' ms', true); line('step overshoot', b && b.step && b.step.overshootPct, a && a.step && a.step.overshootPct, 0, '%', true); }
  }
  return `<table class="gains cmp"><tr><th>measured</th><th>check-up</th><th>final check</th></tr>${rows.join('')}</table>`;
}
function body(f, s) {
  const key = f.phase + ':' + (f.checkup ? f.checkup.t : 0) + ':' + (f.overall ? f.overall.t : 0) + ':' + (f.filters ? f.filters.result || 'run' : '') + ':' + f.writes + ':' + f.tunerState + ':' + f.inFlight + ':' + f.armed + ':' + (f.pausedInfo || '');
  if (key === lastBodyKey) return; lastBodyKey = key;
  const el = $('#fbody');
  const modes = esc(f.goodModes || 'Stabilize, AltHold or PosHold');
  switch (f.phase) {
    case 'idle': case 'stopped': el.innerHTML = `<div class="steps6">
      <div class="card"><b>1 · Connect and read</b><span>Press Start. The app finds the vehicle, reads its tuning settings and checks the setup (hover throttle, filters, notch).</span></div>
      <div class="card"><b>2 · Check-up</b><span>Take off and hover in ${modes}. For 30 s you tap the sticks; nothing is written. You get the issues per axis and the exact changes proposed.</span></div>
      <div class="card"><b>3 · Tune in flight</b><span>Only with "Allow changes in flight" (experimental): press Apply and the gains go to the vehicle in small steps, each watched for oscillation. Otherwise land and write the changes on the ground.</span></div>
      <div class="card"><b>4 · Filters</b><span>With changes in flight allowed, the harmonic notch is set through the vehicle (ESC RPM or the autopilot's FFT) and watched, then the gains are checked once more with the filter active.</span></div>
      <div class="card"><b>5 · Overall check</b><span>30 s of normal flying: grades per axis, vibration, hover throttle, notch.</span></div>
      <div class="card"><b>6 · Land and save</b><span>Land and disarm. You get before-vs-after numbers, the parameter file and the journal of every write.</span></div></div>`; break;
    case 'preflight': el.innerHTML = `<div class="card"><h3>Before you take off</h3><ul class="vt"><li>${ICON.ok}<div>Fly in <b>${modes}</b>. Calm air, safe height, a vehicle that already hovers.</div></li><li>${ICON.ok}<div>Keep a hand on the mode switch. Nothing is written during the check-up.</div></li><li>${f.inFlight ? ICON.warn : ICON.ok}<div>${f.inFlight ? 'Changes in flight are allowed (experimental, tested only in the simulator).' : 'Changes in flight are off: the app only measures in the air.'}</div></li><li>${ICON.ok}<div>Once you hover, the 30 s check-up starts by itself.</div></li></ul></div>`; break;
    case 'checkup': case 'overall': el.innerHTML = `<div class="card"><h3>${f.phase === 'checkup' ? 'Measuring how it flies now' : 'Final check with the new gains'}</h3><p class="muted">Short, sharp stick taps on ${esc(f.opts.axes.join(' and '))}, a few per second-or-two, hovering in between. The more input, the better the measurement.</p></div>`; break;
    case 'review': {
      const c = f.checkup; if (!c) { el.innerHTML = ''; break; }
      const veh = c.vehicleIssues && c.vehicleIssues.length ? `<div class="card" style="margin-bottom:12px"><h3>Vehicle setup</h3>${issueList(c.vehicleIssues)}</div>` : '';
      const off = !f.inFlight && c.summary.changes ? `<div class="banner warn" style="margin-bottom:12px"><div class="ico">✋</div><div><b>Changes in flight are off</b><span>Nothing is written while the vehicle flies. Press Finish, land, then apply these proposals on the ground (Advanced view, Apply per axis) or use the Telemetry + USB tab. To apply them in the air, turn on "Allow changes in flight" (experimental, tested only in the simulator).</span></div></div>` : '';
      el.innerHTML = off + veh + `<div class="cards cols-auto">${Object.entries(c.axes).map(([ax, a]) => axisCardF(ax, a, true)).join('')}</div>`; break;
    }
    case 'tuning': case 'recheck': {
      const lines = (s.journal || []).filter(j => !/GCS_PID_MASK/.test(j.name)).slice(-12).map(j => `<div>${new Date(j.t).toLocaleTimeString()}  ${esc(j.name)}: ${f3(j.from)} → ${f3(j.to)}</div>`).join('');
      el.innerHTML = `<div class="cards cols-2"><div class="card"><h3>${f.phase === 'tuning' ? 'Applying and re-checking' : 'Re-check with the filter active'}</h3><p class="muted">${esc(f.tunerInfo || '')}</p><p class="explain">Axis ${AXIS(f.tunerAxis || '')}, round ${(f.tunerRound || 0) + 1} of 4. Each round: write in steps ≤ 1.5× (each watched), observe 30 s, re-measure; done when the gains settle.</p></div><div class="card"><h3>Written so far (${esc(f.writes)})</h3><div class="scroll">${lines || '<div class="muted">nothing yet</div>'}</div></div></div>`; break;
    }
    case 'filters': {
      const r = f.filters || {}; const st = r.status || {};
      const txt = !r.done ? 'Checking what the vehicle offers: ESC RPM telemetry, the autopilot FFT, or neither. A change is watched on every axis for 10 s.' : r.result === 'esc' ? 'The notch now tracks the ESC RPM: ' + Object.entries(r.applied || {}).map(([k, v]) => esc(k.replace('INS_HNTCH_', '')) + ' ' + esc(v)).join(', ') : r.result === 'fft' ? 'The autopilot measured its motor noise and wrote the notch.' : r.result === 'kept' ? `A notch was already configured (mode ${esc(st.mode)}, ${esc(st.freq)} Hz) and is kept.` : r.result === 'skipped' ? 'Skipped.' : r.result === 'off' ? 'Changes in flight are off: the notch was not changed.' : r.result === 'reverted' ? 'The notch change made the vehicle oscillate and was switched back.' : r.result === 'ground' ? 'This notch changes values that take effect only after a reboot: after landing set it in the Notch panel (disarmed), then reboot.' : 'No in-flight route for the notch on this vehicle: set it from a log after landing (Log file tab).';
      el.innerHTML = `<div class="card"><h3>Harmonic notch filter</h3><p>${txt}</p><p class="muted small">ESC RPM ${st.esc ? 'yes' : 'no'} · autopilot FFT ${st.fft ? (st.fftEnabled ? 'on' : 'available, off') : 'no'} · notch before this step ${st.notchActive ? 'on' : 'off'}</p></div>`; break;
    }
    case 'land': case 'finished': {
      const o = f.overall; const sm = f.summary;
      let html = '';
      if (o && o.verdict) html += `<div class="card"><div class="verdict ${o.verdict.ok ? 'ok' : 'warn'}">${o.verdict.ok ? 'Good: this vehicle is tuned' : 'Flyable, with points to look at'}</div><ul class="vt">${o.verdict.items.map(i => `<li>${i.ok ? ICON.ok : ICON.warn}<div>${esc(i.text)}</div></li>`).join('')}</ul>${f.phase === 'land' ? '<p class="muted">Land and disarm: the summary appears when the motors stop.</p>' : ''}</div>`;
      if (o) html += `<div class="cards cols-auto" style="margin-top:12px">${Object.entries(o.axes).map(([ax, a]) => axisCardF(ax, a, false)).join('')}</div>`;
      if (sm) html += `<div class="card" style="margin-top:12px"><div class="row between"><h3>Before and after</h3><div class="row"><a class="btn primary" href="/api/live/param" download>Download parameter file</a><button class="btn ghost" data-goview="live" data-gopanel="save">Journal</button></div></div>${cmpTable(sm)}<h4 style="margin-top:10px">Gains</h4>${gainsTable(sm.gainsEnd, sm.gainsStart)}<p class="muted small">${esc(sm.writes)} parameter writes this session · ${esc(sm.reason)}${sm.filters && sm.filters.result ? ' · notch: ' + esc(sm.filters.result) : ''}</p>${sm.notes && sm.notes.length ? '<ul class="notes">' + sm.notes.map(n => `<li>${esc(n.text)}</li>`).join('') + '</ul>' : ''}</div>`;
      el.innerHTML = html; break;
    }
    case 'paused': el.innerHTML = `<div class="banner bad"><div class="ico">⏸</div><div><b>Paused</b><span>${esc(f.pausedInfo || '')}</span></div></div>`; break;
    default: el.innerHTML = '';
  }
}
function renderFlight(s) {
  const f = s.flight; if (!f) return;
  const setupDone = s.alive && s.prepared;
  // rail
  const cur = PH_OF[f.phase] === undefined ? null : PH_OF[f.phase]; const curIdx = cur ? PH_ORDER.indexOf(cur) : (f.phase === 'paused' ? PH_ORDER.indexOf(PH_OF[f.resumePhase] || 'checkup') : -1);
  $$('#frail .ph').forEach(p => { const i = PH_ORDER.indexOf(p.dataset.ph); const running = !['idle', 'stopped', 'finished'].includes(f.phase); p.classList.toggle('active', running ? i === curIdx : (p.dataset.ph === 'setup' && !setupDone)); p.classList.toggle('done', (p.dataset.ph === 'setup' && setupDone) || (running && i < curIdx) || (f.phase === 'finished' && i <= PH_ORDER.length - 1 && i > 0)); p.classList.toggle('bad', f.phase === 'paused' && i === curIdx); });
  // hero
  $('#fphase').textContent = f.phase === 'idle' ? (setupDone ? 'Ready' : s.alive ? 'Connected' : 'Not connected') : f.phase.replace('recheck', 're-check');
  $('#ftitle').textContent = TITLE[f.phase] || f.phase;
  $('#finfo').textContent = f.phase === 'idle' ? (setupDone ? `Vehicle read (${s.family}). Press Start, then take off and hover in ${f.goodModes}.` : s.alive ? 'Press Start: the app reads the vehicle, then you take off.' : 'Connect, then press Start: the app reads the vehicle and leads you through one flight.') : (f.phase === 'paused' ? (f.pausedInfo || 'paused') : f.info);
  const chips = [];
  if (s.alive) { chips.push(`<span class="chip ${f.modeOk ? 'good' : (f.armed ? 'warn' : '')}"><span class="dot"></span>${esc(f.mode || '?')}${f.modeOk ? ' ✓' : ''}</span>`); chips.push(`<span class="chip ${f.armed ? 'good' : ''}"><span class="dot"></span>${f.armed ? 'armed' : 'disarmed'}</span>`); chips.push(`<span class="chip ${f.inFlight ? 'warn' : ''}"><span class="dot"></span>${f.inFlight ? 'changes in flight ON' : 'changes in flight off'}</span>`); if (s.rates && s.rates.roll) chips.push(`<span class="chip"><span class="dot"></span>PID data ${esc(s.rates.roll.hz)} Hz</span>`); if (s.vibe) chips.push(`<span class="chip ${Math.max(s.vibe.x, s.vibe.y, s.vibe.z) < 30 ? 'good' : 'warn'}"><span class="dot"></span>vibe ${num(Math.max(s.vibe.x, s.vibe.y, s.vibe.z), 0)}</span>`); }
  $('#fchips').innerHTML = chips.join('');
  const el = (Date.now() - (f.since || Date.now())) / 1000; const w = f.opts.observeSeconds || 30;
  let prog = 0; if (f.phase === 'checkup' || f.phase === 'overall') prog = Math.min(100, 100 * el / w); else if (f.phase === 'tuning' || f.phase === 'recheck') { const ax = f.opts.axes.indexOf(f.tunerAxis); prog = 100 * (Math.max(0, ax) + Math.min(1, (f.tunerRound || 0) / 4)) / f.opts.axes.length; } else if (f.phase === 'finished') prog = 100;
  $('#fprog').style.width = prog + '%';
  $('#fsetup').style.display = ['idle', 'stopped', 'finished'].includes(f.phase) ? '' : 'none';
  const bk = f.phase + ':' + f.tunerState + ':' + f.inFlight + ':' + f.armed;
  if (bk !== lastPhase) { buttons(f); lastPhase = bk; }
  body(f, s);
  // chart
  if ($('#view-flight').classList.contains('active')) { const R = s.series.roll, P = s.series.pitch; const t = R.t.length ? R.t : P.t; const t0 = t.length ? t[t.length - 1] : 0; const rel = arr => arr.map(v => +(v - t0).toFixed(2)); chFlight.setData([rel(t), resampleTo(t, R.t, R.target), resampleTo(t, R.t, R.actual), resampleTo(t, P.t, P.target), resampleTo(t, P.t, P.actual)]); }
}
es.addEventListener('snapshot', () => { try { renderFlight(snap); } catch (e) { console.error(e); } });
es.addEventListener('flight', e => { const p = JSON.parse(e.data); if (p.phase === 'review') toast('Check-up done: see the issues and the proposed changes', 'ok'); if (p.phase === 'land') toast('Final check done: land when you are ready', 'ok'); if (p.phase === 'finished') toast('Session finished: summary and parameter file are ready', 'ok'); if (p.phase === 'paused' && /oscillation/.test(p.info || '')) toast('Oscillation: ' + p.info.replace(/^oscillation: /, ''), 'bad'); });
