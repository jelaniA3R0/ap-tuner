// The local API only obeys this PC's own app page (2026-10-17 audit F-05): Host (and Origin, when sent) must be
// 127.0.0.1 / localhost on the app's port, every state-changing request must be a POST with the per-run token the
// server writes into the page, /api/setparam is gone, and the page is served with a Content-Security-Policy.
// Starts a server on a free port and checks it from outside:
//   node test/server-security.test.js                   the JavaScript server (src/server.js)
//   node test/server-security.test.js --cpp <aptuner>   the C++ server (aptuner.exe --server)
'use strict';
const assert = require('assert');
const http = require('http');
const net = require('net');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawn } = require('child_process');

const ROOT = path.join(__dirname, '..');
const cppIdx = process.argv.indexOf('--cpp');
const CPP = cppIdx > 0 ? path.resolve(process.argv[cppIdx + 1]) : null;
const sleep = ms => new Promise(r => setTimeout(r, ms));

function freePort() { return new Promise((res, rej) => { const s = net.createServer(); s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => res(p)); }); s.on('error', rej); }); }
function req(port, method, p, { host, headers = {}, body } = {}) {
  return new Promise((resolve, reject) => {
    const data = body !== undefined ? JSON.stringify(body) : null;
    const h = Object.assign({ Host: host || `127.0.0.1:${port}` }, data ? { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) } : {}, headers);
    const r = http.request({ host: '127.0.0.1', port, path: p, method, headers: h }, res => { let s = ''; res.on('data', d => s += d); res.on('end', () => resolve({ status: res.statusCode, headers: res.headers, body: s })); });
    r.on('error', reject); r.setTimeout(10000, () => r.destroy(new Error('timeout ' + p)));
    if (data) r.write(data); r.end();
  });
}

(async () => {
  const port = await freePort();
  const sess = fs.mkdtempSync(path.join(os.tmpdir(), 'aptuner-sec-'));
  const child = CPP
    ? spawn(CPP, ['--server', '--port', String(port), '--web', path.join(ROOT, 'web'), '--sessions', sess], { stdio: 'ignore', windowsHide: true })
    : spawn(process.execPath, [path.join(ROOT, 'src', 'server.js')], { env: Object.assign({}, process.env, { PORT: String(port) }), stdio: 'ignore', windowsHide: true });
  const results = [];
  const check = async (name, fn) => { try { await fn(); results.push(['ok', name]); } catch (e) { results.push(['FAIL', name + ': ' + e.message]); } };
  try {
    for (let i = 0; ; i++) { try { if ((await req(port, 'GET', '/api/presets')).status === 200) break; } catch (e) { /* not up yet */ } if (i > 100) throw new Error('server did not start'); await sleep(100); }
    const page = await req(port, 'GET', '/');
    const m = /<meta name="aptuner-token" content="([0-9a-f]{32,})">/.exec(page.body);
    const token = m ? m[1] : null;
    const T = { 'X-APTuner-Token': token };
    const ORIGIN = { Origin: `http://127.0.0.1:${port}` };
    await check('the page carries a per-run token', async () => { assert.strictEqual(page.status, 200); assert.ok(token, 'token meta present and filled in'); assert.ok(!/__APTUNER_TOKEN__/.test(page.body)); });
    await check('the page has a Content-Security-Policy and refuses framing', async () => {
      const csp = page.headers['content-security-policy'] || '';
      assert.match(csp, /script-src 'self'/); assert.ok(!/script-src[^;]*unsafe-inline/.test(csp), 'no inline scripts allowed'); assert.match(csp, /frame-ancestors 'none'/);
      assert.strictEqual(String(page.headers['x-frame-options'] || '').toUpperCase(), 'DENY');
    });
    await check('foreign Host (DNS rebinding) -> 403, also for the page', async () => {
      assert.strictEqual((await req(port, 'GET', '/', { host: `evil.example:${port}` })).status, 403);
      assert.strictEqual((await req(port, 'GET', '/api/presets', { host: 'evil.example' })).status, 403);
      assert.strictEqual((await req(port, 'POST', '/api/pause', { host: `evil.example:${port}`, headers: T, body: {} })).status, 403);
    });
    await check('localhost:port is accepted as Host', async () => { assert.strictEqual((await req(port, 'GET', '/api/presets', { host: `localhost:${port}` })).status, 200); });
    await check('read-only routes answer GET without a token', async () => { assert.strictEqual((await req(port, 'GET', '/api/presets')).status, 200); assert.strictEqual((await req(port, 'GET', '/api/live/rows')).status, 200); });
    for (const route of ['/api/pause', '/api/restore', '/api/flight/continue', '/api/notch/esc', '/api/notch/reboot', '/api/start', '/api/resume', '/api/apply', '/api/options', '/api/ground/write', '/api/ground/recheck', '/api/ground/reboot', '/api/ground/undo']) {
      await check(`GET ${route} -> 405`, async () => { const r = await req(port, 'GET', route); assert.strictEqual(r.status, 405, `status ${r.status}`); });
    }
    await check('GET /api/log/param?save=1 (writes a file) -> 405', async () => { assert.strictEqual((await req(port, 'GET', '/api/log/param?save=1')).status, 405); });
    await check('POST without the token -> 403', async () => { assert.strictEqual((await req(port, 'POST', '/api/pause', { body: {} })).status, 403); });
    await check('POST with a wrong token -> 403', async () => { assert.strictEqual((await req(port, 'POST', '/api/pause', { headers: { 'X-APTuner-Token': '0'.repeat(48) }, body: {} })).status, 403); });
    await check('POST from another site (Origin) -> 403, even with the token', async () => {
      assert.strictEqual((await req(port, 'POST', '/api/pause', { headers: Object.assign({ Origin: 'http://evil.example' }, T), body: {} })).status, 403);
      assert.strictEqual((await req(port, 'POST', '/api/pause', { headers: Object.assign({ Origin: 'null' }, T), body: {} })).status, 403);
    });
    await check('a cross-site browser request (Sec-Fetch-Site) -> 403', async () => { assert.strictEqual((await req(port, 'GET', '/api/presets', { headers: { 'Sec-Fetch-Site': 'cross-site' } })).status, 403); });
    await check('POST with the token from the app page -> 200', async () => {
      const r = await req(port, 'POST', '/api/pause', { headers: Object.assign({}, ORIGIN, T), body: {} });
      assert.strictEqual(r.status, 200, r.body);
    });
    await check('/api/setparam is gone (404)', async () => { const r = await req(port, 'POST', '/api/setparam', { headers: T, body: { name: 'ATC_RAT_RLL_P', value: 1 } }); assert.strictEqual(r.status, 404, `status ${r.status} ${r.body}`); });
    await check('in-flight writes are off by default, and the opt-in is a POST + token', async () => {
      const a = JSON.parse((await req(port, 'POST', '/api/options', { headers: T, body: {} })).body);
      assert.strictEqual(a.opts.inFlightWrites, false, 'off on a fresh start');
      const b = JSON.parse((await req(port, 'POST', '/api/options', { headers: T, body: { inFlightWrites: 'yes' } })).body);
      assert.strictEqual(b.opts.inFlightWrites, false, 'only a real true turns it on');
      const c = JSON.parse((await req(port, 'POST', '/api/options', { headers: T, body: { inFlightWrites: true } })).body);
      assert.strictEqual(c.opts.inFlightWrites, true);
    });
  } finally {
    child.kill();
    await sleep(300);
    try { fs.rmSync(sess, { recursive: true, force: true }); } catch (e) { /* temp */ }
  }
  let failed = 0;
  for (const [s, n] of results) { console.log(`  ${s === 'ok' ? 'ok  ' : 'FAIL'}  ${n}`); if (s !== 'ok') failed++; }
  console.log(`${CPP ? 'C++' : 'JavaScript'} server: ${results.length} check(s), ${failed} failed`);
  process.exit(failed || !results.length ? 1 : 0);
})().catch(e => { console.error('server security test failed:', e); process.exit(1); });
