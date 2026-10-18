// The write journal (2026-10-17 audit F-39): one append-only JSON-lines file per app run in sessions/journal/, one
// line per event, written to disk before the app goes on: the identity of every vehicle, every proposal (id, hash,
// rows), every approval, every write (requested, sent, type, value before, echo, read-back, status, error, time) and
// every rollback, reboot and export. Nothing in it is ever rewritten. A forward write that cannot be journaled is not
// made (a correction, a revert or rollback, still is). The C++ edition writes the same lines (cpp/app/journal.cpp).
'use strict';
const fs = require('fs');
const path = require('path');

class Journal {
  constructor(dir) { this.dir = dir; this.file = null; this.error = null; this.count = 0; this.lost = 0; this.recent = []; }
  /** append one event {t, kind, ...data}; false when it could not be written (the error stays: the journal has a gap) */
  append(kind, data = {}) {
    const e = Object.assign({ t: Date.now(), kind }, data);
    try {
      if (!this.file) {
        fs.mkdirSync(this.dir, { recursive: true });
        this.file = path.join(this.dir, new Date().toISOString().replace(/[:.]/g, '-') + '.jsonl');
      }
      fs.appendFileSync(this.file, JSON.stringify(e) + '\n');
      this.count++;
    } catch (err) { this.error = err.message; this.lost++; return false; }
    this.recent.push(e); if (this.recent.length > 300) this.recent.shift();
    return true;
  }
  /** false once an event could not be written: forward writes stop (restart the app after freeing disk space) */
  get ok() { return !this.lost; }
  status() { return { file: this.file, count: this.count, lost: this.lost, error: this.error }; }
}

module.exports = { Journal };
