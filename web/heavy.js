/* AP Tuner web UI: model tuning for heavy and slow drones (logs or live telemetry through Mission Planner) */
'use strict';
(() => {
  const AX = ['yaw', 'pitch', 'roll'];
  let chosen = [];   // log paths to analyse together
  const busy = (on, text) => { $('#hvBusy').classList.toggle('hidden', !on); if (text) $('#hvBusyText').textContent = text; };
  const lvlIcon = l => l === 'bad' ? '✖' : l === 'warn' ? '⚠' : '✓';
  const rng = a => Array.isArray(a) ? `${num(a[0], 0)}-${num(a[1], 0)}` : '-';

  // ---------------- choosing logs
  const showChosen = () => {
    $('#hvChosen').innerHTML = chosen.length ? '<b>Chosen:</b> ' + chosen.map((p, i) => `<span class="pill">${esc(p.split(/[\\/]/).pop())} <a href="#" data-rm="${i}">×</a></span>`).join(' ') : '<span class="muted">no log chosen yet</span>';
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
    for (const f of files) {
      if (!/\.bin$/i.test(f.name)) { toast(f.name + ' is not a .bin log', 'bad'); continue; }
      busy(true, `copying ${f.name} (${(f.size / 1048576).toFixed(0)} MB)…`);
      const r = await fetch(`/api/heavy/upload?name=${encodeURIComponent(f.name)}`, { method: 'POST', body: f, headers: { 'X-APTuner-Token': TOKEN } });
      const j = await r.json(); if (!r.ok) { busy(false); throw new Error(j.error || r.statusText); }
      if (!chosen.includes(j.path)) chosen.push(j.path);
    }
    busy(false); showChosen();
  });
  $('#hvFind').onclick = guard(async () => {
    const l = await api('log/list');
    $('#hvList').innerHTML = l.length ? l.map(x => `<div data-p="${esc(x.path)}"><span>${esc(x.path)}</span><span class="muted">${(x.size / 1048576).toFixed(0)} MB · ${new Date(x.mtime).toLocaleString()}</span></div>`).join('') : '<div class="muted">no .bin logs found</div>';
    $$('#hvList div[data-p]').forEach(d => d.onclick = () => { if (!chosen.includes(d.dataset.p)) chosen.push(d.dataset.p); showChosen(); });
  });
  $('#hvGo').onclick = guard(async () => {
    if (!chosen.length) { toast('Choose one or more logs first', 'bad'); return; }
    busy(true, `building the model from ${chosen.length} log(s) and testing thousands of settings on it (up to a minute)…`);
    try { render(await api('heavy/analyze', { paths: chosen, spacingM: parseFloat($('#hvSpacing').value) || 0 })); } finally { busy(false); }
  });

  // ---------------- live
  $('#hvConnect').onclick = () => $('#btnConnect').click();
  $('#hvListen').onclick = guard(async () => { const s = await api('heavy/live/start', {}); live(s); toast('Listening: arm and fly', 'ok'); });
  $('#hvNow').onclick = guard(async () => { await api('heavy/live/analyse', {}); });
  $('#hvStop').onclick = guard(async () => { await api('heavy/live/stop', {}); });
  const live = s => {
    if (!s) return;
    const words = { idle: 'Not listening.', reading: 'Reading the drone\'s settings…', waiting: 'Listening. Arm and fly: the recording starts when the drone arms.', recording: `Recording the flight: ${s.recordedS || 0} s (PID data roll ${s.pidSeconds ? s.pidSeconds.roll : 0} s, pitch ${s.pidSeconds ? s.pidSeconds.pitch : 0} s, yaw ${s.pidSeconds ? s.pidSeconds.yaw : 0} s). Land and disarm to analyse.`, analysing: 'Landed: building the model…', done: 'Analysed: see below. Fly again to check the next step.' };
    $('#hvLive').textContent = (words[s.state] || s.state) + (s.info && s.state !== 'recording' && s.state !== 'waiting' ? ' — ' + s.info : '');
    $('#hvNow').classList.toggle('hidden', s.state !== 'recording');
    $('#hvStop').classList.toggle('hidden', !s.listening);
    $('#hvListen').classList.toggle('hidden', !!s.listening);
    busy(s.state === 'analysing', 'landed: building the model from the recorded flight…');
    $('#hvWarn').innerHTML = (s.warnings || []).map(w => `<div class="banner warn"><div class="ico">⚠</div><div><span>${esc(w)}</span></div></div>`).join('');
  };
  es.addEventListener('heavy', e => live(JSON.parse(e.data)));
  es.addEventListener('heavyProgress', e => { const p = JSON.parse(e.data); $('#hvLive').textContent = `Reading the drone's settings ${p.done}/${p.total}…`; });
  es.addEventListener('heavyResult', guard(async () => { render(await api('heavy/result')); toast('Flight analysed', 'ok'); }));
  setInterval(() => { if ($('#view-model').classList.contains('active')) api('heavy/state').then(s => live(s.live)).catch(() => {}); }, 2000);
  api('heavy/result').then(r => { if (r && r.vehicle) render(r); }).catch(() => {});

  // ---------------- result
  function predictedText(axis, p) {
    if (!p) return '';
    if (axis === 'yaw') return `heading swing after lift-off <b>${rng(p.swingDeg)}°</b>, slow-mode damping ${num(p.zeta, 2)} (period ${num(p.periodS, 0)} s), output at its limit ${num(p.limitedPct, 0)}% of the time`;
    const t = a => Array.isArray(a) ? `${num(a[0], 1)}-${num(a[1], 1)}` : '-';
    return `worst phase margin <b>${num(p.pmWorstDeg, 0)}°</b> (safe: 40° or more) · ${p.oscillationExpected ? '<span class="bad">wobble expected</span>' : 'no wobble expected'} · gust wander ${t(p.gustRmsDeg)}° RMS · 5° stick step overshoot ${t(p.stepOvershootDeg)}° · lift-off tilt ${t(p.takeoffTiltDeg)}°`;
  }
  function render(r) {
    const el = $('#hvResult');
    if (!r || r.error && !r.hovers) { el.innerHTML = `<div class="banner bad"><div class="ico">✖</div><div><b>Not analysed</b><span>${esc(r ? r.error : '')}</span></div></div>`; return; }
    const v = r.vehicle || {}, hc = r.heavyClass || {};
    let h = `<div class="banner ${r.error ? 'bad' : 'ok'}"><div class="ico">🚁</div><div style="flex:1"><b>System id ${esc(v.sysid)} · ${esc(v.family)} · ${esc(v.firmware || '')}</b><span>${(r.flights || []).map(f => esc(f.name)).join(', ')} · ${(r.hovers || []).length} hover(s) found${hc.heavy ? ' · heavy / slow vehicle: ' + (hc.why || []).map(esc).join('; ') : ''}${r.planUsed ? ' · compared with the plan of ' + esc(r.planUsed.created) : ''}</span></div>${r.plan ? '<a class="btn ghost" href="/api/heavy/file?kind=plan" download>Download plan</a>' : ''}</div>`;
    if (r.error) h += `<div class="banner bad"><div class="ico">✖</div><div><span>${esc(r.error)}</span></div></div>`;
    // next-flight check
    if (r.check) {
      const c = r.check, ax = c.axes || {};
      const ok = Object.values(ax).every(a => a.confirmed || a.betterThanPredicted);
      h += `<div class="card" style="margin-top:12px"><h3>Check of this flight against the plan</h3>${c.warning ? `<div class="warn small">⚠ ${esc(c.warning)}</div>` : ''}
        <p>${c.stepFlown ? `This flight is <b>Step ${esc(c.stepFlown)}</b>${c.asProposed ? ' of the plan, as proposed' : ' of the plan, flown with other values: ' + (c.differences || []).map(d => `${esc(d.name)} ${f3(d.flown)} (proposed ${f3(d.proposed)})`).join(', ')}.` : 'None of the plan\'s steps is in these settings.'}</p>
        <ul class="notes">${Object.entries(ax).map(([a, x]) => `<li class="${x.confirmed || x.betterThanPredicted ? 'ok' : 'bad'}">${lvlIcon(x.confirmed || x.betterThanPredicted ? 'ok' : 'bad')} ${esc(x.text)}</li>`).join('')}</ul>
        <p class="${ok ? 'ok' : 'warn'}"><b>${ok ? 'The model predicted this flight.' : 'Not everything went as predicted: the model is re-fitted from this flight.'}</b> ${esc(c.next || '')}</p></div>`;
    }
    // issues
    const iss = r.issues || [];
    h += `<div class="card" style="margin-top:12px"><h3>What the flights show</h3><ul class="notes">${iss.map(i => `<li class="${i.level === 'ok' ? 'ok' : i.level === 'bad' ? 'bad' : 'warn'}">${lvlIcon(i.level)} <b>${esc(i.axis ? i.axis + ' · ' : '')}${esc(i.title)}</b>${i.cantTune ? ' <span class="pill warn">tuning can\'t fix this</span>' : ''}<div class="muted">${esc(i.text)}</div>${i.fix ? `<div class="small">→ ${esc(i.fix)}</div>` : ''}</li>`).join('')}</ul></div>`;
    // the model and the replay
    const rp = r.replay || {};
    h += `<div class="card" style="margin-top:12px"><h3>Does the model reproduce your flights?</h3><p class="explain">Before anything is proposed, the model flies your real hovers again with the settings you flew (same stick inputs, same lift-off). Only an axis that passes may jump straight to its design; the others move at most 2× per step.</p><ul class="notes">${AX.filter(a => rp[a]).map(a => `<li class="${rp[a].ok ? 'ok' : 'warn'}">${lvlIcon(rp[a].ok ? 'ok' : 'warn')} <b>${a}</b>: ${esc(rp[a].text)}</li>`).join('')}</ul><div class="cards cols-2" id="hvCharts"></div></div>`;
    // steps
    const steps = r.steps || [];
    h += `<h3 style="margin:16px 0 6px">Your steps</h3>`;
    if (!steps.length) h += `<div class="banner"><div class="ico">ℹ</div><div><span>${hc.heavy ? 'Nothing to change: the flown settings already match the design (or no axis could be modelled).' : 'This drone does not look heavy or slow: use the Flight or Log file tabs.'}</span></div></div>`;
    for (const s of steps) {
      h += `<div class="card" style="margin-top:10px"><div class="row" style="justify-content:space-between"><h3>${esc(s.title)}${s.jump ? '' : ' <span class="pill warn">limited to 2× per step</span>'}${s.reboot ? ' <span class="pill">reboot after writing</span>' : ''}</h3>
        <div class="row"><a class="btn primary" href="/api/heavy/file?step=${esc(s.n)}" download>Download step file</a><a class="btn" href="/api/heavy/file?step=${esc(s.n)}&kind=undo" download>Download UNDO</a></div></div>
        <table class="gains"><tr><th>parameter</th><th>now</th><th>new</th><th>why</th></tr>${(s.rows || []).map(x => `<tr class="changed"><td>${esc(x.name)}<span class="muted small"> ${esc(x.label)}</span></td><td>${f3(x.from)}</td><td>${f3(x.to)}</td><td class="small" style="text-align:left">${esc(x.reason)}</td></tr>`).join('')}</table>
        <p class="small"><b>Predicted:</b> ${Object.entries(s.predicted || {}).map(([a, p]) => `<br>${esc(a)}: ${predictedText(a, p)}`).join('')}</p>
        ${(s.paramErrors || []).map(e => `<div class="bad small">✖ ${esc(e.name)} = ${esc(e.value)} is left out of the file: it ${esc(e.why)}</div>`).join('')}
        <details><summary class="small">Test card for this step</summary><ol class="notes">${(s.testCard || []).map(t => `<li>${esc(t)}</li>`).join('')}</ol></details></div>`;
    }
    // details
    const id = r.identification || {};
    h += `<details class="card" style="margin-top:12px"><summary><b>Details for the expert</b> (model numbers, design margins, stress tests)</summary>
      ${AX.map(a => { const x = (r.axes || {})[a] || {}; const d = x.design || {}; const i = id[a] || {};
        return `<h4 style="margin-top:10px">${a}</h4><div class="small">model: ${esc(i.method || '-')} · control power ${esc(i.a)} rad/s² per unit (range ${esc((i.aRange || []).join('-'))})${i.frequencyHz ? ' · wobble ' + esc(i.frequencyHz) + ' Hz' : ''}${i.trims ? ' · trims ' + esc(i.trims.join(', ')) : ''} · mixer limit ${esc(i.umax)}${i.umaxMeasured ? ' (measured)' : ' (assumed)'}</div>
        <div class="small">flown: ${esc(JSON.stringify(x.flown || {}))}</div><div class="small">model on the flown settings: ${predictedText(a, x.predictedFlown)}</div>
        ${d.ok ? `<div class="small">design: ${esc(JSON.stringify(d.gains))} · worst phase margin ${esc(d.pmWorstDeg)}° · gain margin ${esc(d.gmWorstDb)} dB · Ms ${esc(d.ms)} · noise ×${esc(d.noiseVsNow)} · ${esc(d.accepted)} of ${esc(d.candidates)} candidates passed</div><div class="small">stress: ${esc(JSON.stringify(d.stress))}</div>` : `<div class="small warn">${esc(d.reason || '')}</div>`}`; }).join('')}
      <h4 style="margin-top:10px">vibration above 15 Hz</h4><div class="small">${esc(JSON.stringify(r.vibration || {}))}</div>
      <ul class="notes">${(r.notes || []).map(n => `<li class="small">${esc(n)}</li>`).join('')}</ul></details>`;
    el.innerHTML = h;
    // replay charts: yaw heading per hover (log vs model), roll / pitch wobble
    const ch = $('#hvCharts');
    const yawH = rp.yaw && rp.yaw.hovers ? rp.yaw.hovers : [];
    for (const hv of yawH.slice(0, 6)) {
      const d = document.createElement('div'); d.className = 'chart';
      d.innerHTML = `<h4>yaw · ${esc(hv.hover)} · heading change after lift-off (deg): log ${esc(hv.swingLogDeg)}°, model ${esc(hv.swingModelDeg)}°</h4>`;
      ch.appendChild(d);
      const u = mkChart(d, ['logged heading', 'model'], { height: 200, xLabel: 's', dash: [false, true] }); u.setData([hv.chart.t, hv.chart.log, hv.chart.model]);
    }
    for (const a of ['pitch', 'roll']) {
      if (!rp[a] || !rp[a].chart) continue;
      const c = rp[a].chart, d = document.createElement('div'); d.className = 'chart';
      d.innerHTML = `<h4>${a} · ${esc(c.hover)} · logged angle vs the model with the same settings and random gusts (deg)</h4>`;
      ch.appendChild(d);
      const u = mkChart(d, ['logged ' + a, 'model (gust-driven)'], { height: 200, xLabel: 's', dash: [false, true] }); u.setData([c.t, c.log, c.model]);
    }
  }
})();
