// A small client for the app's local API (the tools in this folder): reads the per-run token from the app page
// and sends it with every POST, as the page itself does (2026-10-17 audit F-05). Read-only routes are GETs.
//   const { api, snapshot, getToken } = require('./app-client')(8091);
//   await api('options', { inFlightWrites: true });   // POST + token
//   await api('notch/status');                         // GET
'use strict';
const http = require('http');

module.exports = function appClient(port, host = '127.0.0.1') {
  let token = null;
  const request = (method, p, body, headers = {}) => new Promise((res, rej) => {
    const data = body !== undefined ? JSON.stringify(body) : null;
    const h = Object.assign(data ? { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) } : {}, headers);
    const req = http.request({ host, port, path: p, method, headers: h }, r => { let s = ''; r.on('data', d => s += d); r.on('end', () => res({ status: r.statusCode, body: s })); });
    req.on('error', rej); req.setTimeout(120000, () => req.destroy(new Error('timeout ' + p)));
    if (data) req.write(data); req.end();
  });
  async function getToken() {
    const r = await request('GET', '/');
    const m = /<meta name="aptuner-token" content="([0-9a-f]+)">/.exec(r.body);
    if (!m) throw new Error('no app token in the page (status ' + r.status + ')');
    token = m[1];
    return token;
  }
  async function api(p, body) {
    if (body !== undefined && !token) await getToken();
    const r = await request(body !== undefined ? 'POST' : 'GET', '/api/' + p, body, body !== undefined ? { 'X-APTuner-Token': token } : {});
    let j; try { j = JSON.parse(r.body); } catch (e) { throw new Error('bad response: ' + r.body.slice(0, 200)); }
    if (r.status >= 400) throw Object.assign(new Error(j.error || r.body), { status: r.status });
    return j;
  }
  /** the next snapshot from the event stream */
  const snapshot = () => new Promise((res, rej) => {
    let s = '';
    const req = http.get({ host, port, path: '/api/events' }, r => { r.on('data', d => { s += d; const i = s.indexOf('event: snapshot'); if (i >= 0 && s.indexOf('\n\n', i) > 0) { req.destroy(); try { res(JSON.parse(/event: snapshot\ndata: (.*)\n/.exec(s.slice(i))[1])); } catch (e) { rej(e); } } }); });
    req.on('error', rej); setTimeout(() => { req.destroy(); rej(new Error('snapshot timeout')); }, 8000);
  });
  return { api, snapshot, getToken };
};
