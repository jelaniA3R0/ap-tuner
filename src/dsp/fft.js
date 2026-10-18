// Radix-2 complex FFT (in-place, iterative) + helpers. Power-of-two sizes.
'use strict';

class FFT {
  constructor(n) {
    if (n & (n - 1)) throw new Error('FFT size must be a power of two');
    this.n = n;
    this.rev = new Uint32Array(n);
    const bits = Math.log2(n);
    for (let i = 0; i < n; i++) { let r = 0, x = i; for (let b = 0; b < bits; b++) { r = (r << 1) | (x & 1); x >>= 1; } this.rev[i] = r; }
    this.cos = new Float64Array(n / 2); this.sin = new Float64Array(n / 2);
    for (let i = 0; i < n / 2; i++) { const a = -2 * Math.PI * i / n; this.cos[i] = Math.cos(a); this.sin[i] = Math.sin(a); }
  }
  // forward transform of (re, im) Float64Arrays in place
  transform(re, im, inverse = false) {
    const n = this.n, rev = this.rev;
    for (let i = 0; i < n; i++) { const j = rev[i]; if (j > i) { let t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; } }
    for (let size = 2; size <= n; size <<= 1) {
      const half = size >> 1, step = n / size;
      for (let i = 0; i < n; i += size) {
        for (let j = 0, k = 0; j < half; j++, k += step) {
          const c = this.cos[k], s = inverse ? -this.sin[k] : this.sin[k];
          const a = i + j, b = a + half;
          const tr = re[b] * c - im[b] * s, ti = re[b] * s + im[b] * c;
          re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti;
        }
      }
    }
    if (inverse) for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
  }
  // real input -> one-sided spectrum arrays (re, im) of length n/2+1
  rfft(x) {
    const re = Float64Array.from(x), im = new Float64Array(this.n);
    this.transform(re, im);
    return { re: re.subarray(0, this.n / 2 + 1), im: im.subarray(0, this.n / 2 + 1) };
  }
}

function hann(n) { const w = new Float64Array(n); for (let i = 0; i < n; i++) w[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / n); return w; }
function nextPow2(n) { let p = 1; while (p < n) p <<= 1; return p; }
/** largest power of two <= n (0 for n < 1) */
function prevPow2(n) { if (!(n >= 1)) return 0; let p = 1; while (p * 2 <= n) p *= 2; return p; }

module.exports = { FFT, hann, nextPow2, prevPow2 };
