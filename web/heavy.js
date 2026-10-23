/* AP Tuner web UI: Model tune (heavy and slow drones). Connect through Mission Planner (or drop logs), record, judge,
   then apply: exact values, a parameter file built from the values shown, or a verified write to the drone. */
'use strict';
(() => {
  const AX = ['yaw', 'pitch', 'roll'];
  let chosen = [];          // log paths to analyse together
  let result = null;        // the latest analysis
  let st = null;            // the latest /heavy/state
  // the result tab to show; a link may name it (#overview, #model, #apply, #details)
  let tab = ['overview', 'model', 'apply', 'details'].includes(location.hash.slice(1)) ? location.hash.slice(1) : 'overview';
  let chartsDrawn = false;
  const evalTimers = {};

  let opBusy = null;   // a write, reboot, upload or analysis started from this page
  let liveBusy = null; // the live recorder reading or analysing
  const showBusy = () => { const t = opBusy || liveBusy; $('#hvBusy').classList.toggle('hidden', !t); if (t) $('#hvBusyText').textContent = t; };
  const busy = (on, text) => { opBusy = on ? (text || 'working…') : null; showBusy(); };
  const lvlIcon = l => l === 'bad' ? '✖' : l === 'warn' ? '⚠' : '✓';
  const rng = (a, d = 0) => Array.isArray(a) ? `${num(a[0], d)}–${num(a[1], d)}` : '–';
  const pmTxt = v => (typeof v === 'number' ? Math.round(v) + '°' : '–');
  // a float32 value as the autopilot holds it, printed with the digits float32 carries
  const f32 = v => (v === null || v === undefined || isNaN(v)) ? '–' : String(+(+v).toPrecision(7));
  const download = (name, text) => { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' })); a.download = name; document.body.appendChild(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000); };

  // ======================================================================== the drone panel
  const STREAMS = [['HEARTBEAT', 'heartbeat', 0.5, 0.5], ['ATTITUDE', 'attitude', 4, 1], ['PID_TUNING', 'PID data (rate, P, I, D)', 6, 1], ['SERVO_OUTPUT_RAW', 'motor outputs', 3, 0.5], ['RC_CHANNELS', 'sticks', 2, 0.5], ['VFR_HUD', 'throttle / altitude', 1, 0.2]];
  function renderDrone(s) {
    const alive = s && s.alive, v = (s && s.vehicle) || {}, live = (s && s.live) || {};
    $('#mtRing').className = 'mt-ring' + (alive ? ' on' : s && s.connected ? ' warn' : '');
    $('#mtConnTitle').textContent = alive ? `${(v.typeName || 'vehicle').replace('MAV_TYPE_', '').replace(/_/g, ' ').toLowerCase()} · system id ${s.identity ? s.identity.sysid : '?'}` : s && s.connected ? 'Connected, waiting for the drone…' : 'Not connected';
    $('#mtConnSub').textContent = alive ? (live.link || 'connected') : 'Press Connect: Mission Planner is found by itself';
    $('#hvConnect').textContent = alive ? 'Reconnect' : 'Connect';
    $('#hvConnect').className = 'btn ' + (alive ? 'ghost' : 'primary');
    $('#mtChips').innerHTML = alive ? [v.armed ? '<span class="pill bad">ARMED</span>' : '<span class="pill ok">disarmed</span>', v.modeName ? `<span class="pill">${esc(v.modeName)}</span>` : '', s.identity && s.identity.fw ? `<span class="pill">ArduPilot ${esc(s.identity.fw)}</span>` : ''].join('') : '';
    const a = live.attitude || {};
    const has = alive && typeof a.roll === 'number';
    $('#mtRoll').textContent = has ? a.roll.toFixed(1) + '°' : '–';
    $('#mtPitch').textContent = has ? a.pitch.toFixed(1) + '°' : '–';
    $('#mtYaw').textContent = has ? a.yaw.toFixed(0) + '°' : '–';
    $('#mtHorizonG').setAttribute('transform', has ? `rotate(${-a.roll}) translate(0 ${Math.max(-50, Math.min(50, a.pitch * 2))})` : '');
    const rates = live.rates || {};
    $('#mtHealth').innerHTML = alive ? STREAMS.map(([k, label, good, some]) => {
      const r = rates[k] || 0; const cls = r >= good ? 'ok' : r >= some ? 'warn' : 'bad';
      return `<div><i class="${cls}"></i><span>${esc(label)}</span><em>${r ? r.toFixed(1) + ' /s' : 'none'}</em></div>`;
    }).join('') + (!(rates.PID_TUNING > 0) ? '<div class="small warn" style="margin-top:4px">No PID data: in Mission Planner set GCS_PID_MASK = 7 (Full Parameter List → Write Params), then Start listening.</div>' : '') : '<div class="muted small">connect to see the data the drone sends</div>';
    // recording
    const state = live.state || 'idle';
    const dot = { waiting: 'wait', recording: 'rec', analysing: 'busy', reading: 'busy' }[state] || '';
    $('#mtRecDot').className = 'mt-recdot ' + dot;
    $('#mtRecTitle').textContent = { idle: 'Not listening', reading: 'Reading the drone\'s settings…', waiting: 'Listening: arm and fly', recording: 'Recording the flight', analysing: 'Landed: analysing…', done: 'Analysed' }[state] || state;
    $('#hvLive').textContent = state === 'recording' ? `PID data: roll ${live.pidSeconds ? live.pidSeconds.roll : 0} s · pitch ${live.pidSeconds ? live.pidSeconds.pitch : 0} s · yaw ${live.pidSeconds ? live.pidSeconds.yaw : 0} s` : (live.info || '');
    const secs = state === 'recording' ? (live.recordedS || 0) : 0;
    $('#mtTimer').textContent = `${Math.floor(secs / 60)}:${String(Math.floor(secs % 60)).padStart(2, '0')}`;
    $('#hvNow').classList.toggle('hidden', state !== 'recording');
    $('#hvStop').classList.toggle('hidden', !live.listening);
    $('#hvListen').classList.toggle('hidden', !!live.listening);
    $('#hvListen').disabled = !alive;
    liveBusy = state === 'analysing' || state === 'reading' ? (state === 'reading' ? 'reading the drone\'s settings…' : 'landed: building the model from the recorded flight…') : null;
    showBusy();
    $('#hvWarn').innerHTML = (live.warnings || []).map(w => `<div class="small warn" style="margin-top:8px">⚠ ${esc(w)}</div>`).join('');
    // the stepper
    const done = { connect: alive || !!result, record: !!result, judge: !!result, apply: false };
    const now = !alive && !result ? 'connect' : !result ? 'record' : 'apply';
    $$('#mtSteps .mt-step').forEach(e => { e.classList.toggle('done', done[e.dataset.s]); e.classList.toggle('now', e.dataset.s === now); });
    // the write buttons follow the connection
    $$('[data-write]').forEach(b => { const why = writeBlocked(); b.disabled = !!why; b.title = why || 'write these exact values to the drone (verified, with UNDO)'; });
    $$('[data-writewhy]').forEach(e => { e.textContent = writeBlocked() || ''; });
  }
  function writeBlocked() {
    if (!st || !st.alive) return 'connect to the drone to write from the app';
    if (st.vehicle && st.vehicle.armed) return 'the drone is armed: values are written only on the ground';
    if (result && result.vehicle && st.identity && result.vehicle.sysid !== st.identity.sysid) return `the connected drone is system id ${st.identity.sysid}; the analysed flights are from ${result.vehicle.sysid}`;
    return '';
  }
  const poll = () => { if (!$('#view-model').classList.contains('active')) return; api('heavy/state').then(s => { st = s; renderDrone(s); }).catch(() => {}); };
  setInterval(poll, 1000); poll();
  es.addEventListener('heavy', e => { if (st) { st.live = JSON.parse(e.data); renderDrone(st); } });
  es.addEventListener('heavyProgress', e => { const p = JSON.parse(e.data); $('#hvLive').textContent = `reading the drone's settings ${p.done}/${p.total}…`; });
  es.addEventListener('heavyResult', guard(async () => { render(await api('heavy/result')); toast('Flight analysed', 'ok'); }));

  // ---- connect / record
  $('#hvConnect').onclick = () => $('#btnConnect').click();
  $('#hvListen').onclick = guard(async () => { await api('heavy/live/start', {}); toast('Listening: arm and fly', 'ok'); poll(); });
  $('#hvNow').onclick = guard(async () => { await api('heavy/live/analyse', {}); });
  $('#hvStop').onclick = guard(async () => { await api('heavy/live/stop', {}); poll(); });

  // ---- logs
  const showChosen = () => {
    $('#hvChosen').innerHTML = chosen.length ? chosen.map((p, i) => `<span class="pill" style="margin:2px">${esc(p.split(/[\\/]/).pop())} <a href="#" data-rm="${i}">×</a></span>`).join('') : '<span class="muted">no log chosen</span>';
    $$('#hvChosen [data-rm]').forEach(a => a.onclick = e => { e.preventDefault(); chosen.splice(+a.dataset.rm, 1); showChosen(); });
  };
  showChosen();
  const drop = $('#hvDrop');
  drop.onclick = () => $('#hvFile').click();
  drop.ondragover = e => { e.preventDefault(); drop.classList.add('over'); };
  drop.ondragleave = () => drop.classList.remove('over');
  drop.ondrop = e => { e.preventDefault(); drop.classList.remove('over'); uploadAll([...e.dataTransfer.files]); };
  $('#hvFile').onchange = e => uploadAll([...e.target.files]);
  const uploadAll = guard(async files => {
    try {
      for (const f of files) {
        if (!/\.bin$/i.test(f.name)) { toast(f.name + ' is not a .bin log', 'bad'); continue; }
        busy(true, `copying ${f.name} (${(f.size / 1048576).toFixed(0)} MB)…`);
        const r = await fetch(`/api/heavy/upload?name=${encodeURIComponent(f.name)}`, { method: 'POST', body: f, headers: { 'X-APTuner-Token': TOKEN } });
        const j = await r.json(); if (!r.ok) throw new Error(j.error || r.statusText);
        if (!chosen.includes(j.path)) chosen.push(j.path);
      }
    } finally { busy(false); showChosen(); }
  });
  $('#hvFind').onclick = guard(async () => {
    const l = await api('log/list');
    $('#hvList').innerHTML = l.length ? l.map(x => `<div data-p="${esc(x.path)}"><span>${esc(x.path.split(/[\\/]/).pop())}</span><span class="muted">${(x.size / 1048576).toFixed(0)} MB · ${new Date(x.mtime).toLocaleDateString()}</span></div>`).join('') : '<div class="muted">no .bin logs found</div>';
    $$('#hvList div[data-p]').forEach(d => d.onclick = () => { if (!chosen.includes(d.dataset.p)) chosen.push(d.dataset.p); showChosen(); });
  });
  $('#hvGo').onclick = guard(async () => {
    if (!chosen.length) { toast('Choose one or more logs first', 'bad'); return; }
    busy(true, `building the model from ${chosen.length} log(s) and testing thousands of settings on it (up to a minute)…`);
    try { render(await api('heavy/analyze', { paths: chosen, spacingM: parseFloat($('#hvSpacing').value) || 0 })); } finally { busy(false); }
  });

  // ======================================================================== the result
  function gauge(now, design, lo, hi, good, unit) {
    // a half circle from lo to hi; green beyond `good`; the design value as the arc, the flown value as a marker
    const ang = v => Math.PI * (1 - (Math.max(lo, Math.min(hi, v)) - lo) / (hi - lo));
    const pt = (v, r) => [60 + r * Math.cos(ang(v)), 62 - r * Math.sin(ang(v))];
    const arc = (a, b, r) => { const [x1, y1] = pt(a, r), [x2, y2] = pt(b, r); return `M${x1.toFixed(1)} ${y1.toFixed(1)} A${r} ${r} 0 0 1 ${x2.toFixed(1)} ${y2.toFixed(1)}`; };
    let s = `<svg viewBox="0 0 120 72"><path d="${arc(lo, hi, 48)}" stroke="#223049" stroke-width="10" fill="none" stroke-linecap="round"/><path d="${arc(good, hi, 48)}" stroke="rgba(34,197,94,.25)" stroke-width="10" fill="none"/>`;
    if (typeof design === 'number') s += `<path d="${arc(lo, design, 48)}" stroke="${design >= good ? '#22c55e' : '#f59e0b'}" stroke-width="10" fill="none" stroke-linecap="round"/>`;
    if (typeof now === 'number') { const [x, y] = pt(now, 48), [x0, y0] = pt(now, 30); s += `<line x1="${x0.toFixed(1)}" y1="${y0.toFixed(1)}" x2="${x.toFixed(1)}" y2="${y.toFixed(1)}" stroke="#ef4444" stroke-width="3" stroke-linecap="round"/>`; }
    return s + '</svg>';
  }
  function axisTile(r, a) {
    const iss = (r.issues || []).filter(i => i.axis === a);
    const worst = iss.some(i => i.level === 'bad') ? 'bad' : iss.some(i => i.level === 'warn') ? 'warn' : 'ok';
    const head = iss.find(i => i.level === 'bad') || iss.find(i => i.level === 'warn');
    const x = (r.axes || {})[a] || {}, d = x.design || {}, pf = x.predictedFlown || {}, pd = d.predicted || {};
    const id = (r.identification || {})[a] || {};
    let g = '';
    if (d.kept) g = a === 'yaw'
      ? `<div class="mt-gauge">${gauge(pf.zeta, pf.zeta, 0, 1, 0.35)}<div class="mt-gnum">damping<b>${num(pf.zeta, 2)} · keep</b>swing ${rng(pf.swingDeg)}° · what you fly passes every test</div></div>`
      : `<div class="mt-gauge">${gauge(pf.pmWorstDeg, pf.pmWorstDeg, -30, 70, 40)}<div class="mt-gnum">phase margin<b>${pmTxt(pf.pmWorstDeg)} · keep</b>what you fly passes every test</div></div>`;
    else if (a === 'yaw') g =`<div class="mt-gauge">${gauge(pf.zeta, pd.zeta, 0, 1, 0.35)}<div class="mt-gnum">damping<b>${num(pf.zeta, 2)} → ${d.ok ? num(pd.zeta, 2) : '–'}</b>swing ${rng(pf.swingDeg)}° → ${d.ok ? rng(pd.swingDeg) + '°' : '–'}</div></div>`;
    else g = `<div class="mt-gauge">${gauge(pf.pmWorstDeg, d.ok ? d.pmWorstDeg : undefined, -30, 70, 40)}<div class="mt-gnum">phase margin<b>${pmTxt(pf.pmWorstDeg)} → ${d.ok ? pmTxt(d.pmWorstDeg) : '–'}</b>${pf.oscillationExpected ? 'wobbles now' : 'calm now'} · safe ≥ 40°</div></div>`;
    const valid = r.replay && r.replay[a] ? r.replay[a].ok : null;
    return `<div class="mt-card mt-axis ${worst}"><h3>${a}<span class="pill ${valid ? 'ok' : valid === false ? 'warn' : ''}">${valid ? 'model proven' : valid === false ? 'model not proven' : 'no model'}</span></h3>
      <div class="mt-line">${head ? `<b style="color:var(--text)">${esc(head.title)}</b>${head.cantTune ? ' · tuning can\'t fix' : ''}` : 'Nothing wrong found'}${id.a ? ` · control power ${esc(id.a)}` : ''}</div>${id.ok || x.model && x.model.ok ? g : '<div class="muted small">' + esc(id.reason || 'no model of this axis') + '</div>'}</div>`;
  }
  function predictedText(axis, p) {
    if (!p) return '';
    if (axis === 'yaw') return `heading swing after lift-off <b>${rng(p.swingDeg)}°</b> · damping ${num(p.zeta, 2)} · period ${num(p.periodS, 0)} s · at its limit ${num(p.limitedPct, 0)}% of the time`;
    return `phase margin <b>${pmTxt(p.pmWorstDeg)}</b> · ${p.oscillationExpected ? '<span class="bad">wobble expected</span>' : 'no wobble expected'} · gust wander ${rng(p.gustRmsDeg, 1)}° · 5° step overshoot ${rng(p.stepOvershootDeg, 1)}° · lift-off tilt ${rng(p.takeoffTiltDeg, 1)}°`;
  }
  function render(r) {
    result = r && r.vehicle ? r : null;
    chartsDrawn = false;
    $('#mtEmpty').classList.toggle('hidden', !!result);
    const el = $('#hvResult');
    if (!r || (r.error && !r.hovers)) { el.innerHTML = r ? `<div class="mt-card"><div class="bad"><b>Not analysed</b> ${esc(r.error)}</div></div>` : ''; renderDrone(st); return; }
    const v = r.vehicle || {}, hc = r.heavyClass || {};
    const nIss = (r.issues || []).filter(i => i.level !== 'ok').length;
    let h = `<div class="mt-card mt-head"><div style="font-size:30px">🚁</div><div class="mt-title"><b>System id ${esc(v.sysid)} · ${esc(v.family)} · ${esc(v.firmware || '')}</b><span>${(r.flights || []).map(f => esc(f.name)).join(', ')} · ${(r.hovers || []).length} hover(s)${r.planUsed ? ' · compared with the plan of ' + esc(r.planUsed.created) : ''}</span></div>
      ${hc.heavy ? '<span class="pill warn">heavy / slow vehicle</span>' : '<span class="pill">not heavy</span>'}${r.plan ? '<a class="btn ghost" href="/api/heavy/file?kind=plan" download>Plan</a>' : ''}</div>`;
    if (r.error) h += `<div class="mt-card bad">${esc(r.error)}</div>`;
    if (r.check) {
      const c = r.check, ax = c.axes || {};
      const ok = Object.values(ax).every(a => a.confirmed || a.betterThanPredicted);
      h += `<div class="mt-card" style="border-color:${ok ? '#1f4d3a' : '#6b4e12'}"><h3>${ok ? '✓ The model predicted this flight' : '⚠ Not everything went as predicted'}</h3>
        <p class="small muted">${c.stepFlown ? `This flight is <b>Step ${esc(c.stepFlown)}</b>${c.asProposed ? ', flown as proposed' : ', flown with other values: ' + (c.differences || []).map(d => `${esc(d.name)} ${f3(d.flown)} (proposed ${f3(d.proposed)})`).join(', ')}.` : 'None of the plan\'s steps is in these settings.'} ${esc(c.next || '')}</p>
        <div class="mt-issues">${Object.entries(ax).map(([a, x]) => `<div class="mt-issue ${x.confirmed || x.betterThanPredicted ? 'ok' : 'warn'}"><div class="ico">${x.confirmed || x.betterThanPredicted ? '✓' : '⚠'}</div><div><b style="text-transform:capitalize">${esc(a)}</b><p>${esc(x.text)}</p></div></div>`).join('')}</div></div>`;
    }
    h += `<div class="mt-axes">${AX.map(a => axisTile(r, a)).join('')}</div>`;
    const steps = r.steps || [];
    h += `<div class="mt-tabs" id="mtTabs"><button data-tab="overview">What's wrong<span class="pill ${nIss ? 'warn' : 'ok'}">${nIss}</span></button><button data-tab="model">Model check</button><button data-tab="apply">Settings &amp; apply<span class="pill ${steps.length ? 'ok' : ''}">${steps.length}</span></button><button data-tab="details">Details</button></div>`;
    // overview
    h += `<div class="mt-pane" data-pane="overview"><div class="mt-issues">${(r.issues || []).map(i => `<div class="mt-issue ${i.level}"><div class="ico">${lvlIcon(i.level)}</div><div style="flex:1"><b>${esc(i.axis ? i.axis + ' · ' : '')}${esc(i.title)}${i.cantTune ? ' <span class="pill warn">tuning can\'t fix this</span>' : ''}</b><p>${esc(i.text)}</p>${i.fix ? `<div class="fix">→ ${esc(i.fix)}</div>` : ''}</div></div>`).join('')}</div></div>`;
    // model check
    const rp = r.replay || {};
    h += `<div class="mt-pane" data-pane="model"><div class="mt-card"><h3>Does the model reproduce your flights?</h3><p class="explain">Before anything is proposed, the model flies your real hovers again with the settings you flew: same stick inputs, same lift-off. Only an axis that passes may jump straight to its design; the others move at most 2× per step.</p>
      <div class="mt-issues">${AX.filter(a => rp[a]).map(a => `<div class="mt-issue ${rp[a].ok ? 'ok' : 'warn'}"><div class="ico">${rp[a].ok ? '✓' : '⚠'}</div><div><b style="text-transform:capitalize">${a}</b><p>${esc(rp[a].text)}</p></div></div>`).join('')}</div></div><div class="cards cols-2" id="hvCharts"></div></div>`;
    // settings and apply
    h += `<div class="mt-pane" data-pane="apply">`;
    if (!steps.length) h += `<div class="mt-card"><b>Nothing to change.</b> <span class="muted">${hc.heavy ? 'The settings you fly already pass every test on the model (or no axis could be modelled).' : 'This drone does not look heavy or slow: use the Flight or Log file tabs.'}</span>${(r.notes || []).filter(n => /^Keep/.test(n)).map(n => `<div class="ok small" style="margin-top:6px">✓ ${esc(n)}</div>`).join('')}</div>`;
    else h += `<div class="mt-card small muted">Each step: the <b>proposed</b> values are pre-filled in <b>your value</b>. Change any of them and the model re-checks them at once. Then download the parameter file (exactly the values shown, as the autopilot stores them) or write them to the drone: it must be connected and disarmed, the values are read back one by one, and an UNDO file is saved first. ${(r.notes || []).filter(n => /^Keep/.test(n)).map(n => `<div class="ok" style="margin-top:6px">✓ ${esc(n)}</div>`).join('')}</div>`;
    for (const s of steps) {
      h += `<div class="mt-card mt-step-card" data-step="${esc(s.n)}"><h3>${esc(s.title)}${s.jump ? '<span class="pill ok">validated model</span>' : '<span class="pill warn">limited to 2× per step</span>'}${s.reboot ? '<span class="pill">reboot after writing</span>' : ''}</h3>
        <table class="gains mt-edit"><tr><th>parameter</th><th>flown</th><th>proposed</th><th>your value</th><th class="why">why</th></tr>
        ${(s.rows || []).map(x => `<tr><td class="mt-mono">${esc(x.name)}<div class="muted small lbl">${esc(x.label)}</div></td><td class="mono">${esc(x.fromText != null ? x.fromText : f3(x.from))}</td><td class="mono">${esc(x.text)}</td><td><input data-name="${esc(x.name)}" data-proposed="${esc(x.text)}" value="${esc(x.text)}" inputmode="decimal" spellcheck="false"><div class="small bad" data-err="${esc(x.name)}"></div></td><td class="why">${esc(x.reason)}</td></tr>`).join('')}</table>
        <div class="mt-pred">${Object.entries(s.predicted || {}).map(([a, p]) => `<div><b>${esc(a)} · predicted with the proposal</b>${predictedText(a, p)}</div>`).join('')}</div>
        <div class="mt-verdicts" data-verdicts></div>
        <div class="mt-actions"><button class="btn ghost" data-reset>Reset to proposal</button><span class="grow"></span><button class="btn" data-file>⬇ Parameter file</button><a class="btn" href="/api/heavy/file?step=${esc(s.n)}&kind=undo" download>⬇ UNDO file</a><button class="btn primary" data-write>Write to drone</button></div>
        <div class="small muted" data-writewhy style="text-align:right;margin-top:4px"></div>
        <details style="margin-top:8px"><summary class="small">Test card for this step</summary><ol class="notes">${(s.testCard || []).map(t => `<li>${esc(t)}</li>`).join('')}</ol></details></div>`;
    }
    h += `</div>`;
    // details
    const id = r.identification || {};
    h += `<div class="mt-pane" data-pane="details"><div class="mt-card">${AX.map(a => { const x = (r.axes || {})[a] || {}; const d = x.design || {}; const i = id[a] || {};
        return `<h4 style="margin-top:10px">${a}</h4><div class="small">model: ${esc(i.method || '–')} · control power ${esc(i.a)} rad/s² per unit (range ${esc((i.aRange || []).join('–'))})${i.frequencyHz ? ' · wobble ' + esc(i.frequencyHz) + ' Hz' : ''}${i.trims ? ' · trims ' + esc(i.trims.join(', ')) : ''} · mixer limit ${esc(i.umax)}${i.umaxMeasured ? ' (measured)' : ' (estimated)'}</div>
        <div class="small">model on the flown settings: ${predictedText(a, x.predictedFlown)}</div>
        ${d.ok ? `<div class="small">design: worst phase margin ${esc(d.pmWorstDeg)}° · gain margin ${esc(d.gmWorstDb)} dB · Ms ${esc(d.ms)} · noise ×${esc(d.noiseVsNow)} · ${esc(d.accepted)} of ${esc(d.candidates)} candidates passed</div><div class="small mt-mono">${esc(JSON.stringify(d.stress))}</div>` : `<div class="small warn">${esc(d.reason || '')}</div>`}`; }).join('')}
      <h4 style="margin-top:10px">vibration above 15 Hz</h4><div class="small mt-mono">${esc(JSON.stringify(r.vibration || {}))}</div><ul class="notes">${(r.notes || []).map(n => `<li class="small">${esc(n)}</li>`).join('')}</ul></div></div>`;
    el.innerHTML = h;
    $$('#mtTabs button').forEach(b => b.onclick = () => showTab(b.dataset.tab));
    showTab(tab === 'apply' && !steps.length ? 'overview' : tab);
    for (const card of $$('.mt-step-card')) wireStep(card);
    renderDrone(st);
  }
  function showTab(t) {
    tab = t;
    $$('#mtTabs button').forEach(b => b.classList.toggle('active', b.dataset.tab === t));
    $$('.mt-pane').forEach(p => p.classList.toggle('active', p.dataset.pane === t));
    if (t === 'model' && !chartsDrawn) { chartsDrawn = true; drawCharts(); }
  }
  function drawCharts() {
    const r = result; if (!r) return;
    const ch = $('#hvCharts'), rp = r.replay || {};
    for (const hv of (rp.yaw && rp.yaw.hovers ? rp.yaw.hovers : []).slice(0, 6)) {
      const d = document.createElement('div'); d.className = 'chart';
      d.innerHTML = `<h4>yaw · ${esc(hv.hover)} · heading after lift-off: log ${esc(hv.swingLogDeg)}°, model ${esc(hv.swingModelDeg)}° ${hv.pass ? '✓' : '⚠'}</h4>`;
      ch.appendChild(d);
      mkChart(d, ['logged heading', 'model'], { height: 200, xLabel: 's', dash: [false, true] }).setData([hv.chart.t, hv.chart.log, hv.chart.model]);
    }
    for (const a of ['pitch', 'roll']) {
      if (!rp[a] || !rp[a].chart) continue;
      const c = rp[a].chart, d = document.createElement('div'); d.className = 'chart';
      d.innerHTML = `<h4>${a} · ${esc(c.hover)} · logged angle vs the model with the same settings (gust-driven), deg</h4>`;
      ch.appendChild(d);
      mkChart(d, ['logged ' + a, 'model'], { height: 200, xLabel: 's', dash: [false, true] }).setData([c.t, c.log, c.model]);
    }
  }

  // ======================================================================== a step: edit, check, file, write
  const rowsOf = card => $$('input[data-name]', card).map(i => ({ name: i.dataset.name, value: i.value.trim() }));
  function wireStep(card) {
    const n = +card.dataset.step;
    const check = () => { clearTimeout(evalTimers[n]); evalTimers[n] = setTimeout(() => evaluate(card).catch(e => toast(e.message, 'bad')), 450); };
    $$('input[data-name]', card).forEach(i => i.oninput = () => { i.classList.toggle('changed', i.value.trim() !== i.dataset.proposed); check(); });
    $('[data-reset]', card).onclick = () => { $$('input[data-name]', card).forEach(i => { i.value = i.dataset.proposed; i.classList.remove('changed', 'bad'); }); check(); };
    $('[data-file]', card).onclick = guard(async () => {
      const f = await api('heavy/paramfile', { rows: rowsOf(card), title: `Step ${n}` });
      if (f.errors && f.errors.length) { toast(`${f.errors.length} value(s) left out of the file: ${f.errors.map(e => e.name).join(', ')}`, 'bad'); }
      download(`AP Tuner sysid ${result.vehicle.sysid} Step ${n}.param`, f.text);
    });
    $('[data-write]', card).onclick = guard(() => writeStep(card, n));
    evaluate(card).catch(() => {});
  }
  async function evaluate(card) {
    const ev = await api('heavy/evaluate', { rows: rowsOf(card) });
    for (const r of ev.rows) {
      const inp = $(`input[data-name="${CSS.escape(r.name)}"]`, card); const err = $(`[data-err="${CSS.escape(r.name)}"]`, card);
      if (inp) inp.classList.toggle('bad', !r.ok);
      if (err) err.innerHTML = !r.ok ? esc(r.errors.join(', ')) : (inp && r.text && r.text !== inp.value.trim() ? `<span class="muted">stored as ${esc(r.text)}</span>` : (r.warnings || []).length ? `<span class="warn">${esc(r.warnings[0])}</span>` : '');
    }
    const box = $('[data-verdicts]', card);
    box.innerHTML = AX.filter(a => ev.axes[a] && ev.axes[a].touched).map(a => {
      const x = ev.axes[a];
      if (x.pass === null) return `<div class="mt-verdict"><h4><span>${a}</span><span class="muted">no model</span></h4>${(x.why || []).map(esc).join('')}</div>`;
      return `<div class="mt-verdict ${x.pass ? 'pass' : 'fail'}"><h4><span>${a} · your values</span><span class="${x.pass ? 'ok' : 'bad'}">${x.pass ? '✓ safe on the model' : '✖ fails a safety test'}</span></h4>
        <ul>${(x.tests || []).map(t => `<li class="${t.ok ? 'ok' : t.hard ? 'bad' : 'warn'}">${t.ok ? '✓' : t.hard ? '✖' : '⚠'} <span style="color:var(--text)">${esc(t.text)}</span></li>`).join('')}</ul>
        <div class="muted" style="margin-top:4px">${predictedText(a, x.predicted)}</div></div>`;
    }).join('');
    card.dataset.ok = ev.ok && AX.every(a => !ev.axes[a] || !ev.axes[a].touched || ev.axes[a].pass !== false) ? '1' : '0';
  }
  async function writeStep(card, n) {
    const why = writeBlocked(); if (why) { toast(why, 'bad'); return; }
    await evaluate(card);
    const p = await api('heavy/writeplan', { step: n, rows: rowsOf(card) });
    const fails = AX.filter(a => p.check[a] && p.check[a].touched && p.check[a].pass === false);
    const changes = p.rows.filter(r => !r.same).length;
    const list = `<table class="gains mt-writelist"><tr><th>parameter</th><th>on the drone now</th><th>will be</th></tr>${p.rows.map(r => `<tr><td class="mt-mono">${esc(r.name)}</td><td class="mt-mono ${r.same ? 'same' : ''}">${esc(r.vehicleText)}</td><td class="mt-mono ${r.same ? 'same' : ''}"><b>${esc(r.text)}</b>${r.same ? ' (already)' : ''}</td></tr>`).join('')}</table>`;
    const warn = fails.length ? `<div class="banner bad"><div class="ico">✖</div><div><b>These values fail a safety test on the model (${fails.join(', ')})</b><span>Write them only if you know why.</span></div></div>` : '';
    const ok = await modal(`Write step ${n} to the drone?`, `${warn}<p class="muted">The drone (system id ${esc(p.sysid)}) is disarmed. ${changes} value(s) change. An UNDO file with the drone's values is saved before the first write; every value is read back, and if one does not hold, everything goes back to the value from before.</p>${list}<p class="muted small">list ${esc(p.hash)}</p>`, { yes: changes ? `Write ${changes} value(s)` : 'Nothing to write', danger: fails.length > 0 });
    if (!ok || !changes) return;
    busy(true, 'writing and reading back every value…');
    let w;
    try { w = await api('heavy/write', { id: p.id, hash: p.hash }); } finally { busy(false); }
    const rows = (w.txn && w.txn.rows) || [];
    const ok2 = w.txn && w.txn.ok;
    await modal(ok2 ? '✓ Written and verified' : '✖ Not written', `<p>${esc(w.text)}</p><table class="gains"><tr><th>parameter</th><th>status</th><th>the drone holds</th></tr>${rows.map(r => `<tr><td class="mt-mono">${esc(r.name)}</td><td class="${r.status === 'verified' ? 'ok' : 'bad'}">${esc(r.status)}</td><td class="mt-mono">${esc(f32(r.holds))}</td></tr>`).join('')}</table>${w.undoFile ? `<p class="small muted">UNDO file: ${esc(w.undoFile)}</p>` : ''}${w.reboot ? '<p class="warn">INS_GYRO_FILTER acts only after a reboot: press Reboot (on the ground, disarmed).</p>' : ''}`, { yes: w.reboot ? 'Reboot now' : 'OK', no: w.reboot ? 'Later' : '' }).then(async yes => {
      if (yes && w.reboot) {
        busy(true, 'rebooting the autopilot and reading every value again…');
        let rb; try { rb = await api('heavy/reboot', {}); } finally { busy(false); }
        await modal(rb.ok ? '✓ Rebooted and checked' : '⚠ Reboot not confirmed', `<p>${rb.ok ? 'The autopilot restarted and every value was read again:' : esc(rb.reason || 'the restart could not be proven')}</p><table class="gains"><tr><th>parameter</th><th>expected</th><th>the drone holds</th></tr>${(rb.rows || []).map(x => `<tr><td class="mt-mono">${esc(x.name)}</td><td class="mt-mono">${esc(f32(x.expect))}</td><td class="${x.ok ? 'ok' : 'bad'}">${x.ok ? '✓ ' : '✖ '}${esc(f32(x.holds))}</td></tr>`).join('')}</table>`, { yes: 'OK', no: '' });
      }
    });
    if (ok2) toast(`Step ${n} written and verified`, 'ok');
  }

  api('heavy/result').then(r => { if (r && r.vehicle) render(r); }).catch(() => {});
})();
