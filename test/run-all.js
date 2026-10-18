// Runs every JavaScript test file and exits non-zero if any fails:  node test/run-all.js
'use strict';
const { spawnSync } = require('child_process');
const path = require('path');

const files = ['synthetic.test.js', 'watchdog.test.js', 'logintegrity.test.js', 'reallogs.test.js',
  'live-safety.test.js', 'write-verify.test.js', 'web-escape.test.js', 'server-security.test.js'];
let bad = 0;
for (const f of files) {
  console.log(`\n===== ${f}`);
  const r = spawnSync(process.execPath, [path.join(__dirname, f)], { stdio: 'inherit' });
  if (r.status !== 0) { bad++; console.log(`===== ${f} FAILED (exit ${r.status})`); }
}
console.log(bad ? `\n${bad} test file(s) failed` : '\nall JavaScript tests passed');
process.exit(bad ? 1 : 0);
