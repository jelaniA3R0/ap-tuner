// Guided flight session. One flight, led by the app:
//   preflight (armed, in a good mode) -> checkup (30 s: issues + proposed changes, nothing written)
//   -> review (pilot presses "Apply in flight", or auto) -> tuning (the Tuner applies in bounded steps
//   with the watchdog until the gains converge) -> filters (notch via ESC RPM or the autopilot FFT)
//   -> recheck (one more pass over all chosen axes with the filter active) -> overall (30 s final
//   check: grades, vibration, hover throttle, notch) -> land (wait for disarm) -> finished (summary).
'use strict';
const EventEmitter = require('events');
const { axisIssues, vehicleIssues } = require('./issues');

const GOOD_MODES = {
  copter: ['STABILIZE', 'ALT_HOLD', 'POSHOLD', 'LOITER'],
  heli: ['STABILIZE', 'ALT_HOLD', 'POSHOLD', 'LOITER'],
  vtol: ['QSTABILIZE', 'QHOVER', 'QLOITER'],
  plane: ['FLY_BY_WIRE_A', 'FLY_BY_WIRE_B', 'CRUISE', 'STABILIZE'],
};
const PHASES = ['preflight', 'checkup', 'review', 'tuning', 'filters', 'recheck', 'overall', 'land', 'finished'];
const RUNNING = new Set(['preflight', 'checkup', 'review', 'tuning', 'filters', 'recheck', 'overall', 'land', 'paused']);

class FlightSession extends EventEmitter {
  constructor(link, tuner, notch) { super(); this.link = link; this.tuner = tuner; this.notch = notch; this.reset(); }

  reset() {
    this.phase = 'idle'; this.since = Date.now(); this.info = '';
    this.opts = { axes: ['roll', 'pitch'], preset: 'normal', auto: false, observeSeconds: 30 };
    this.checkup = null; this.filters = null; this.overall = null; this.summary = null; this.notes = [];
    this.pausedInfo = null; this.resumePhase = null; this.gainsStart = null; this.journalStart = 0; this._tFlying = 0;
    if (this._timer) clearTimeout(this._timer); this._timer = null;
  }

  get family() { return this.tuner.family; }
  get quadplane() { return this.tuner.params.Q_ENABLE > 0; }
  goodModes() { return GOOD_MODES[this.family] || GOOD_MODES.copter; }
  _pretty(list) { return list.map(m => m.replace('FLY_BY_WIRE_', 'FBW').replace('ALT_HOLD', 'AltHold').replace('POSHOLD', 'PosHold').replace('STABILIZE', 'Stabilize').replace('LOITER', 'Loiter').replace('QHOVER', 'QHover').replace('QLOITER', 'QLoiter')).join(', '); }
  goodModesText() { return this.quadplane && this.phase === 'preflight' ? `${this._pretty(GOOD_MODES.vtol)} (hover tune) or ${this._pretty(GOOD_MODES.plane)} (fixed-wing tune)` : this._pretty(this.goodModes()); }
  modeOk() { const v = this.link.vehicle; return !!(v && this.goodModes().includes(v.modeName)); }

  setPhase(p, info) { this.phase = p; this.since = Date.now(); this.info = info || ''; this.emit('phase', { phase: p, info: this.info }); this.tuner.log('flight', `${p}${info ? ': ' + info : ''}`); }
  note(level, text) { this.notes.push({ t: Date.now(), phase: this.phase, level, text }); this.tuner.log('flight', text); }

  /** start a session: the vehicle must have been read (prepare) first */
  start(opts = {}) {
    if (!this.tuner.prepared) throw new Error('read the vehicle first');
    if (RUNNING.has(this.phase)) throw new Error('a flight session is already running');
    this.reset();
    Object.assign(this.opts, opts);
    this.tuner.opts.axes = this.opts.axes; this.tuner.opts.preset = this.opts.preset; this.tuner.opts.autoApply = true;
    this.gainsStart = JSON.parse(JSON.stringify(this.tuner.snapshot().gains));
    this.journalStart = this.tuner.journal.length;
    this.setPhase('preflight', 'arm, take off and hover at a safe height');
    this._tick();
  }
  stop() { if (this._timer) clearTimeout(this._timer); this._timer = null; this.tuner.stop().catch(() => {}); this.setPhase('stopped', 'stopped by you'); }

  /** the pilot's confirmation at a waiting point */
  continue() {
    if (this.phase === 'review') { if (!this.checkup || !this.checkup.summary.anyProposal) throw new Error('nothing to apply yet: repeat the check-up with sharper stick taps'); this._startTuning(); return; }
    if (this.phase === 'land' || this.phase === 'paused') { this._finish(this.phase === 'land' ? 'finished before landing' : 'finished early'); return; }
    throw new Error('nothing to continue in phase ' + this.phase);
  }
  repeatCheckup() { if (this.phase !== 'review' && this.phase !== 'paused') throw new Error('not at the check-up'); for (const ax of this.opts.axes) this.tuner.axes[ax].setBaseline(); this.setPhase('checkup', 'measuring again: short sharp stick taps on roll and pitch'); if (!this._timer) this._tick(); }
  skipFilters() { if (this.phase !== 'filters') throw new Error('not at the filter step'); this.filters = Object.assign(this.filters || {}, { done: true, result: 'skipped' }); this.note('warn', 'filter step skipped'); }
  resume() {
    if (this.phase !== 'paused') { if (this.tuner.state === 'paused') this.tuner.resume(); return; }
    const p = this.resumePhase || 'checkup'; this.pausedInfo = null;
    if (p === 'tuning' || p === 'recheck') { this.setPhase(p, 'continuing'); this.tuner.resume(); }
    else if (p === 'checkup' || p === 'overall') { for (const ax of this.opts.axes) this.tuner.axes[ax].setBaseline(); this.setPhase(p, 'continuing: keep tapping the sticks'); }
    else this.setPhase(p, 'continuing');
    if (!this._timer) this._tick();
  }

  _pause(reason) { if (this.phase !== 'paused') { this.resumePhase = this.phase; this.pausedInfo = reason; this.setPhase('paused', reason); } }
  _tick() {
    if (this._timer) clearTimeout(this._timer);
    const loop = async () => { try { await this._step(); } catch (e) { this.note('bad', 'error: ' + e.message); } this._timer = RUNNING.has(this.phase) ? setTimeout(loop, 1000) : null; };
    this._timer = setTimeout(loop, 300);
  }

  async _step() {
    const v = this.link.vehicle; const armed = !!(v && v.armed); const elapsed = (Date.now() - this.since) / 1000;
    if (!this.link.alive) { this.info = 'link lost: waiting for the vehicle'; return; }
    switch (this.phase) {
      case 'preflight': {
        if (!armed) { this._tFlying = 0; this.info = 'arm and take off, then hover at a safe height in ' + this.goodModesText(); return; }
        if (this.quadplane) {   // a QuadPlane: tune the controller of the mode being flown
          const want = /^Q/.test(v.modeName || '') ? 'vtol' : 'plane';
          if (want !== this.family && GOOD_MODES[want].includes(v.modeName)) {
            this.info = `switching to the ${want === 'vtol' ? 'hover (VTOL)' : 'fixed-wing'} controller`; this._tFlying = 0;
            await this.tuner.switchFamily(want);
            this.tuner.opts.axes = this.opts.axes; this.tuner.opts.preset = this.opts.preset; this.tuner.opts.autoApply = true;
            this.gainsStart = JSON.parse(JSON.stringify(this.tuner.snapshot().gains));
            this.note('ok', `QuadPlane in ${v.modeName}: tuning the ${want === 'vtol' ? 'hover (Q_A_*)' : 'fixed-wing'} gains`);
            return;
          }
        }
        if (!this.modeOk()) { this._tFlying = 0; this.info = `flying in ${v.modeName}: switch to ${this.goodModesText()}`; return; }
        if (!this._tFlying) this._tFlying = Date.now();
        if (Date.now() - this._tFlying < 5000) { this.info = `flying in ${v.modeName}: the check-up starts in a moment`; return; }
        for (const ax of this.opts.axes) this.tuner.axes[ax].setBaseline();
        this.tuner.requestStreams();
        this.setPhase('checkup', 'fly normally and tap the sticks: short sharp inputs on roll and pitch'); return;
      }
      case 'checkup': case 'overall': {
        if (!armed) { this._pause('the vehicle disarmed: take off again and press Resume, or press Finish'); return; }
        if (!this.modeOk()) { this.info = `switch back to ${this.goodModesText()} (now ${v.modeName})`; this.since = Date.now(); return; }
        const w = this.opts.observeSeconds;
        if (elapsed < w) { this.info = `${this.phase === 'checkup' ? 'measuring how it flies now' : 'final check with the new gains'}: ${Math.ceil(w - elapsed)} s left, keep tapping the sticks`; return; }
        const res = {};
        for (const ax of this.opts.axes) { const r = this.tuner.analyze(ax); res[ax] = r; r.issues = axisIssues(r, this.family); }
        const summary = this._summarise(res);
        if (this.phase === 'checkup') {
          this.checkup = { results: res, summary, gains: JSON.parse(JSON.stringify(this.tuner.snapshot().gains)), checks: this.tuner.safetyChecks(), t: Date.now() };
          this.checkup.vehicleIssues = vehicleIssues(this.checkup.checks);
          if (!summary.anyProposal) { this.setPhase('review', 'nothing could be proposed yet (too little stick input): fly sharper taps and press Repeat'); return; }
          if (!summary.changes) { this.note('ok', 'the gains already match the measurement: nothing to apply'); this.setPhase('review', 'the gains are already right; press Continue for the filter and final check, or Repeat'); return; }
          if (this.opts.auto) { this.note('ok', 'auto mode: applying the changes now'); this._startTuning(); return; }
          this.setPhase('review', `${summary.changes} change(s) proposed: read the issues, then press Apply in flight`); return;
        }
        const hoverName = this.family === 'vtol' ? 'Q_M_THST_HOVER' : 'MOT_THST_HOVER';
        const hover = (this.family === 'plane') ? null : await this.link.requestParam(hoverName).catch(() => this.tuner.params[hoverName]);
        this.overall = { results: res, summary, gains: JSON.parse(JSON.stringify(this.tuner.snapshot().gains)), vibe: this.tuner.vibe || null, hover, notch: this.tuner.params.INS_HNTCH_ENABLE > 0, t: Date.now() };
        this.overall.verdict = this._verdict(this.overall);
        this.setPhase('land', this.overall.verdict.ok ? 'all good: land when you are ready' : 'land when you are ready; the list shows what still needs attention'); return;
      }
      case 'tuning': case 'recheck': {
        const st = this.tuner.state;
        if (st === 'done') { if (this.phase === 'tuning') await this._startFilters(); else this._startOverall(); return; }
        if (st === 'paused') { if (!armed) { this._pause('the vehicle disarmed during tuning: take off again and press Resume, or press Finish'); return; } this.pausedInfo = this.tuner.stateInfo; this.info = 'paused: ' + this.tuner.stateInfo + ' (press Resume when it flies normally)'; return; }
        if (st === 'idle') { this._pause('the tuner stopped: press Resume or Finish'); return; }
        this.pausedInfo = null; this.info = this.tuner.stateInfo; return;
      }
      case 'filters': {
        // after the filter step the gains are always checked again (all chosen axes), filter changed or not
        if (this.filters && this.filters.done) this._startRecheck();
        return;
      }
      case 'land': { if (!armed) this._finish('landed'); return; }
      default: return;
    }
  }

  _startTuning() {
    this.tuner.opts.useFreshResult = true;   // the check-up analysis is applied right away, then the normal rounds continue
    this.tuner.start();
    this.setPhase('tuning', 'applying the changes in flight: small steps, each watched for oscillation');
  }
  async _startFilters() {
    this.setPhase('filters', 'checking the notch filter options'); this.filters = { started: Date.now(), done: false };
    try {
      const st = await this.notch.status();
      this.filters.status = { notchActive: st.notchActive, mode: st.mode, esc: st.escTelemetry, fft: st.fftCapable, fftEnabled: st.fftEnabled, freq: st.current.INS_HNTCH_FREQ };
      if (st.notchActive) { this.filters.result = 'kept'; this.note('ok', `notch filter already configured (mode ${st.mode}, ${st.current.INS_HNTCH_FREQ} Hz): kept`); }
      else if (st.escProposal) { const a = await this.notch.applyEsc(); this.filters.result = 'esc'; this.filters.applied = a; this.note('ok', 'notch filter set to track the ESC RPM: ' + Object.entries(a).map(([k, v]) => k.replace('INS_HNTCH_', '') + ' ' + v).join(', ')); }
      else if (st.fftCapable && st.fftEnabled && this.link.vehicle && this.link.vehicle.armed) {
        this.info = 'measuring the motor noise with the autopilot FFT: hold a steady hover for 30 s, sticks centred';
        await this.notch.startFftTune(); await new Promise(r => setTimeout(r, 30000));
        const r = await this.notch.stopFftTune();
        this.filters.result = r.ok ? 'fft' : 'none'; this.filters.applied = r.changed;
        this.note(r.ok ? 'ok' : 'warn', r.ok ? 'the autopilot measured its motor noise and wrote the notch filter' : 'the FFT measurement wrote nothing (it needs a steadier hover): set the notch from a log after landing');
      } else {
        this.filters.result = 'none';
        this.note('warn', st.fftCapable ? 'no ESC RPM telemetry, and the autopilot FFT is off (enable it and reboot on the ground): set the notch from a log after landing' : 'no in-flight route for the notch on this autopilot (no ESC RPM, no FFT): set it from a log after landing');
      }
    } catch (e) { this.filters.result = 'none'; this.note('warn', 'filter step failed: ' + e.message); }
    this.filters.done = true;
  }
  _startRecheck() { this.tuner.opts.useFreshResult = false; this.tuner.start(); this.setPhase('recheck', 'checking the gains again with the filter active'); }
  _startOverall() { for (const ax of this.opts.axes) this.tuner.axes[ax].setBaseline(); this.setPhase('overall', 'final check: fly normally for 30 s with a few stick taps'); }

  _summarise(res) {
    let changes = 0, anyProposal = false, bad = 0, warn = 0, worst = 'A';
    const order = ['A', 'B', 'C', 'D', 'F'];
    for (const r of Object.values(res)) {
      if (r.proposal) { anyProposal = true; if (r.proposal.changes.some(c => ['P', 'D', 'FF', 'angleP', 'tconst'].includes(c.key) && Math.abs(c.pct || 0) > 10)) changes++; }
      for (const i of r.issues || []) { if (i.level === 'bad') bad++; else if (i.level === 'warn') warn++; }
      const g = r.diagnostics && r.diagnostics.grade ? r.diagnostics.grade.letter : null; if (g && order.indexOf(g) > order.indexOf(worst)) worst = g;
    }
    return { changes, anyProposal, bad, warn, worst };
  }
  _verdict(ov) {
    const items = []; let ok = true;
    for (const [ax, r] of Object.entries(ov.results)) {
      const g = r.diagnostics && r.diagnostics.grade; const d = r.diagnostics || {};
      if (!g) { items.push({ ok: false, text: `${ax}: no measurement (too little stick input)` }); ok = false; continue; }
      if (g.gradable === false) { items.push({ ok: false, text: `${ax}: not gradable (too little stick input on this axis): fly a few sharp stick taps to check it` }); ok = false; continue; }
      const good = ['A', 'B'].includes(g.letter) && !d.oscillating;
      if (!good) ok = false;
      items.push({ ok: good, text: `${ax}: grade ${g.letter}, follows ${d.trackingPct !== null && d.trackingPct !== undefined ? d.trackingPct.toFixed(0) + '%' : 'n/a'} of the demand, error ${d.rmsErrorDegs?.toFixed(1)} deg/s` + (d.oscillating ? ', oscillating' : '') });
      const big = r.proposal && r.proposal.changes.some(c => ['P', 'D'].includes(c.key) && Math.abs(c.pct || 0) > 20);
      if (big) items.push({ ok: false, text: `${ax}: the gains could still move more than 20% (${r.proposal.changes.filter(c => ['P', 'D'].includes(c.key)).map(c => c.key + ' ' + Math.round(c.pct) + '%').join(', ')}): run another session later` });
    }
    if (ov.vibe) { const v = ov.vibe; const vib = Math.max(v.x, v.y, v.z); const good = vib < 30 && v.clip === 0; if (!good) ok = false; items.push({ ok: good, text: `vibration ${vib.toFixed(0)} (ArduPilot: below 30 is good)` + (v.clip ? `, ${v.clip} accelerometer clips` : '') }); }
    if (ov.hover !== null && ov.hover !== undefined) { const good = ov.hover > 0.2 && ov.hover < 0.7; items.push({ ok: good, text: `hover throttle ${ov.hover.toFixed(2)}` + (good ? ' (normal range)' : ' (outside 0.2 to 0.7: check the thrust-to-weight ratio)') }); }
    items.push({ ok: ov.notch, text: ov.notch ? 'harmonic notch filter active' : 'no harmonic notch filter yet: set it from a log after landing' });
    if (!ov.notch) { /* a missing notch is advice, not a failure of this flight */ }
    return { ok, items };
  }
  _finish(reason) {
    if (this._timer) clearTimeout(this._timer); this._timer = null;
    if (this.tuner.state !== 'idle') this.tuner.stop().catch(() => {});
    const gainsEnd = JSON.parse(JSON.stringify(this.tuner.snapshot().gains));
    const pick = (res) => res ? Object.fromEntries(Object.entries(res).map(([ax, r]) => [ax, { grade: r.diagnostics && r.diagnostics.grade ? r.diagnostics.grade.letter : null, score: r.diagnostics && r.diagnostics.grade ? r.diagnostics.grade.score : null, trackingPct: r.diagnostics ? r.diagnostics.trackingPct : null, rmsErrorDegs: r.diagnostics ? r.diagnostics.rmsErrorDegs : null, oscRmsDegs: r.diagnostics ? r.diagnostics.oscRmsDegs : null, step: r.measuredStep ? r.measuredStep.metrics : null }])) : null;
    this.summary = { reason, before: this.checkup ? pick(this.checkup.results) : null, after: this.overall ? pick(this.overall.results) : null, gainsStart: this.gainsStart, gainsEnd, writes: this.tuner.journal.length - this.journalStart, filters: this.filters, verdict: this.overall ? this.overall.verdict : null, notes: this.notes, axes: this.opts.axes };
    this.setPhase('finished', reason);
  }

  snapshot() {
    const v = this.link.vehicle;
    const slim = (r) => r ? { issues: r.issues || [], grade: r.diagnostics && r.diagnostics.grade ? r.diagnostics.grade : null, diagnostics: r.diagnostics ? { trackingPct: r.diagnostics.trackingPct, rmsErrorDegs: r.diagnostics.rmsErrorDegs, oscRmsDegs: r.diagnostics.oscRmsDegs, saturationPct: r.diagnostics.saturationPct, oscillating: r.diagnostics.oscillating } : null, proposal: r.proposal ? { changes: r.proposal.changes, margins: r.proposal.margins, overshootPct: r.proposal.overshootPct, predicted: r.proposal.predicted ? { current: r.proposal.predicted.currentMetrics, proposed: r.proposal.predicted.proposedMetrics } : null } : null, measuredStep: r.measuredStep ? r.measuredStep.metrics : null, error: r.error || null, current: r.current || null, warnings: r.warnings || [] } : null;
    const block = (b) => b ? { summary: b.summary, axes: Object.fromEntries(Object.entries(b.results).map(([a, r]) => [a, slim(r)])), gains: b.gains, vehicleIssues: b.vehicleIssues || [], verdict: b.verdict || null, vibe: b.vibe || null, hover: b.hover ?? null, notch: b.notch, t: b.t } : null;
    return { phase: this.phase, phases: PHASES, since: this.since, info: this.info, opts: this.opts, mode: v ? v.modeName : null, armed: !!(v && v.armed), modeOk: this.modeOk(), goodModes: this.goodModesText(),
      pausedInfo: this.pausedInfo, resumePhase: this.resumePhase, checkup: block(this.checkup), filters: this.filters, overall: block(this.overall), summary: this.summary, notes: this.notes.slice(-30),
      tunerState: this.tuner.state, tunerInfo: this.tuner.stateInfo, tunerRound: this.tuner.round, tunerAxis: this.tuner.currentAxis(), writes: this.tuner.journal.length - this.journalStart };
  }
}

module.exports = { FlightSession, GOOD_MODES, PHASES };
