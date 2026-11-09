/*
 * dsp.js — the chirp receiver's signal processing, mirrored from the firmware.
 *
 * Pure functions only: no DOM, no drawing. Everything the lab plots comes from
 * decodeFrame(), which runs the same steps as chirp_decoder.c:
 *
 *   receivedSignal()  test signal: cyclically shifted real chirp + noise
 *   bandPass()        chirp_decoder_real_chirp_to_magnitudes, step 1
 *   dechirp()         multiply by the conjugate base chirp (complex_dechirp_vector)
 *   fold()            fold_power: coherent fold of bins k and k - B
 *   finePower()       dechirp_and_fold: folded power every half bin
 *   centroid()        cyclic_weighted_centroid_accumulate / _finalize
 *   peakBin()         cyclic_weighted_centroid_peak_bin
 *   refinePeak()      cyclic_weighted_centroid_refine_peak (mean shift)
 *   decideSymbol()    chirp_decoder_magnitudes_to_symbol + chirp_symbol_rx_set
 *
 * If the firmware changes, change the matching function here and update
 * check.html, which compares these functions against numbers from the C tests.
 *
 * Exposed as the global ChirpDsp (browser) or module.exports (Node).
 */
'use strict';
(function (root) {
  const TAU = 2 * Math.PI;

  /** a mod n, always in [0, n) */
  const mod = (a, n) => ((a % n) + n) % n;

  // ---------------------------------------------------------------------------
  // Random numbers (deterministic, so the noise stays put while parameters change)
  // ---------------------------------------------------------------------------

  function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  /** N standard normal draws (Box–Muller) from a seed */
  function makeNoise(N, seed) {
    const rnd = mulberry32(seed);
    const z = new Float64Array(N);
    for (let i = 0; i < N; i++) {
      let u = 0;
      while (u === 0) u = rnd();
      z[i] = Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * rnd());
    }
    return z;
  }

  // ---------------------------------------------------------------------------
  // FFT
  // ---------------------------------------------------------------------------

  /**
   * In-place radix-2 complex FFT, X[m] = sum x[n] e^{-j2πmn/N}.
   * inverse = true computes the inverse and scales by 1/N, like CMSIS
   * arm_cfft_f32 with ifftFlag = 1.
   */
  function fft(re, im, inverse = false) {
    const N = re.length;
    for (let i = 1, j = 0; i < N; i++) {
      let bit = N >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        [re[i], re[j]] = [re[j], re[i]];
        [im[i], im[j]] = [im[j], im[i]];
      }
    }
    const sign = inverse ? 1 : -1;
    for (let len = 2; len <= N; len <<= 1) {
      const ang = sign * TAU / len, wr = Math.cos(ang), wi = Math.sin(ang);
      for (let i = 0; i < N; i += len) {
        let cr = 1, ci = 0;
        for (let j = 0; j < len / 2; j++) {
          const a = i + j, b = a + len / 2;
          const tr = re[b] * cr - im[b] * ci, ti = re[b] * ci + im[b] * cr;
          re[b] = re[a] - tr; im[b] = im[a] - ti;
          re[a] += tr; im[a] += ti;
          const ncr = cr * wr - ci * wi;
          ci = cr * wi + ci * wr;
          cr = ncr;
        }
      }
    }
    if (inverse) {
      for (let i = 0; i < N; i++) { re[i] /= N; im[i] /= N; }
    }
  }

  /** FFT of a copy: returns { re, im } */
  function spectrumOf(re, im) {
    const r = Float64Array.from(re), q = im ? Float64Array.from(im) : new Float64Array(re.length);
    fft(r, q);
    return { re: r, im: q };
  }

  /** |X|^2 of every bin */
  function powerOf(X) {
    const p = new Float64Array(X.re.length);
    for (let i = 0; i < p.length; i++) p[i] = X.re[i] * X.re[i] + X.im[i] * X.im[i];
    return p;
  }

  // ---------------------------------------------------------------------------
  // Configuration (main.c: FS, F0, FFT_SIZE, MAX_SYMBOLS; chirp_decoder_init)
  // ---------------------------------------------------------------------------

  /**
   * Derived chirp parameters.
   * p: { M symbols, L bins per symbol (firmware: 1), N FFT size, fs Hz, f0 Hz }
   */
  function chirpConfig(p) {
    const { M, L, N, fs, f0 } = p;
    const bins = L * M;                 // Bins spanned by the chirp bandwidth (fold size)
    const binHz = fs / N;
    const B = bins * binHz;             // Bandwidth, Hz
    const T = N / fs;                   // Symbol duration, s
    const chirpRate = B / T;            // Hz per second

    // Band-pass bins (chirp_decoder_init): the chirp band f0 .. f0 + B
    const bandFirst = Math.floor(f0 / binHz);
    const bandLast = Math.min(Math.ceil((f0 + B) / binHz), N / 2 - 1);

    // Fold rotation (chirp_decoder_init): the wrapped piece of a symbol needs
    // rotating by exp(j2π(f0·N/fs − B/2)) to join the first piece in phase
    let turns = f0 * N / fs - 0.5 * bins;
    turns -= Math.floor(turns);

    return {
      ...p, bins, binHz, B, T, chirpRate, bandFirst, bandLast,
      samplesPerSymbol: N / M,
      foldTurns: turns,
      foldRe: Math.cos(TAU * turns),
      foldIm: Math.sin(TAU * turns),
    };
  }

  /** Chirp phase at sample v: 2π(f0·t + ½·rate·t²), t = v/fs */
  function chirpPhase(cfg, v) {
    const t = v / cfg.fs;
    return TAU * (cfg.f0 * t + 0.5 * cfg.chirpRate * t * t);
  }

  /**
   * Conjugate base chirp, generate_complex_dechirp_vector: [cos φ, −sin φ],
   * optionally shifted by shiftBins bins (0.5 for the half-bin spectrum: FFT
   * bin k then holds the spectrum at k + ½).
   */
  function dechirpVector(cfg, shiftBins = 0) {
    const re = new Float64Array(cfg.N), im = new Float64Array(cfg.N);
    for (let n = 0; n < cfg.N; n++) {
      const ph = chirpPhase(cfg, n) + TAU * shiftBins * n / cfg.N;
      re[n] = Math.cos(ph);
      im[n] = -Math.sin(ph);
    }
    return { re, im };
  }

  /** Both de-chirp vectors the decoder uses: { whole, half } */
  function dechirpVectors(cfg) {
    return { whole: dechirpVector(cfg, 0), half: dechirpVector(cfg, 0.5) };
  }

  /** Echo profile used by tests/test_chirp_codec.c (delay in samples, gain) */
  const TEST_ECHOES = [{ delay: 3, gain: 0.6 }, { delay: 7, gain: 0.4 }, { delay: 15, gain: 0.3 }, { delay: 31, gain: 0.2 }];

  // ---------------------------------------------------------------------------
  // Test signal (tests/chirp_channel.c)
  // ---------------------------------------------------------------------------

  /**
   * One received frame of a repeated symbol k (so the frame is a pure cyclic
   * shift whatever the offset), amplitude 1, as seen through the channel:
   *
   *   x[j] = cos(phase((u + k·N/M) mod N)) + noise,   u = j·(1 + ppm·1e-6) + offset
   *
   * offset > 0: the receive window starts late (samples, may be fractional).
   * ppm: time scale — clock mismatch or Doppler, which are the same thing here.
   * snrDb: signal power over white noise power across the full band; null = no noise.
   * echoes: optional [{ delay (samples), gain }], delayed copies (multipath).
   * noise: standard normal draws from makeNoise().
   */
  function receivedSignal(cfg, sig, noise) {
    const { N, M } = cfg;
    const x = new Float64Array(N);
    const scale = 1 + (sig.ppm || 0) * 1e-6;
    const sigma = sig.snrDb == null ? 0 : Math.SQRT1_2 / 10 ** (sig.snrDb / 20);
    const shift = sig.k * N / M;
    const paths = [{ delay: 0, gain: 1 }, ...(sig.echoes || [])];
    for (let j = 0; j < N; j++) {
      const u = j * scale + (sig.offset || 0);
      let v = 0;
      for (const p of paths) v += p.gain * Math.cos(chirpPhase(cfg, mod(u - p.delay + shift, N)));
      x[j] = v + sigma * noise[j];
    }
    return x;
  }

  // ---------------------------------------------------------------------------
  // Decoder steps (chirp_decoder.c)
  // ---------------------------------------------------------------------------

  /**
   * Band-pass: FFT, keep only bins bandFirst .. bandLast, inverse FFT. Gives
   * the complex band-limited (analytic) signal, about half the input amplitude.
   * enabled = false passes the real input straight through (the old decoder).
   * Returns { re, im, before, after } where before/after are the spectra.
   */
  function bandPass(cfg, x, enabled = true) {
    const before = spectrumOf(x, null);
    if (!enabled) {
      return { re: Float64Array.from(x), im: new Float64Array(x.length), before, after: before };
    }
    const re = Float64Array.from(before.re), im = Float64Array.from(before.im);
    for (let i = 0; i < cfg.N; i++) {
      if (i < cfg.bandFirst || i > cfg.bandLast) { re[i] = 0; im[i] = 0; }
    }
    const after = { re: Float64Array.from(re), im: Float64Array.from(im) };
    fft(re, im, true);
    return { re, im, before, after };
  }

  /** Complex multiply by the conjugate base chirp (arm_cmplx_mult_cmplx_f32) */
  function dechirp(sig, dv) {
    const N = sig.re.length, re = new Float64Array(N), im = new Float64Array(N);
    for (let n = 0; n < N; n++) {
      re[n] = sig.re[n] * dv.re[n] - sig.im[n] * dv.im[n];
      im[n] = sig.re[n] * dv.im[n] + sig.im[n] * dv.re[n];
    }
    return { re, im };
  }

  /**
   * Fold the de-chirped spectrum X into `bins` bins (dechirped_to_magnitudes).
   * A symbol wraps part way through the frame; its pieces land in bins k and
   * N − bins + k.
   *   'coherent' (firmware): |X[k] + R·X[N−bins+k]|²
   *   'power'    (old):      |X[k]|² + |X[N−bins+k]|²
   * half = true for the half-bin spectrum, which is folded with −R.
   * Returns { folded, coherent, power, first, second } (first/second: the raw
   * powers of the two pieces).
   */
  function fold(cfg, X, method = 'coherent', half = false) {
    const { N, bins } = cfg;
    const rr = half ? -cfg.foldRe : cfg.foldRe, ri = half ? -cfg.foldIm : cfg.foldIm;
    const coherent = new Float64Array(bins), power = new Float64Array(bins);
    const first = new Float64Array(bins), second = new Float64Array(bins);
    for (let k = 0; k < bins; k++) {
      const m = N - bins + k;
      const ar = X.re[k], ai = X.im[k], br = X.re[m], bi = X.im[m];
      const re = ar + (rr * br - ri * bi), im = ai + (rr * bi + ri * br);
      first[k] = ar * ar + ai * ai;
      second[k] = br * br + bi * bi;
      coherent[k] = re * re + im * im;
      power[k] = first[k] + second[k];
    }
    return { folded: method === 'power' ? power : coherent, coherent, power, first, second };
  }

  /**
   * Folded power every half bin (dechirp_and_fold): [2k] = whole bin k,
   * [2k+1] = k + ½, from the whole-bin fold and the half-bin fold.
   */
  function finePower(wholeFolded, halfFolded) {
    const fine = new Float64Array(2 * wholeFolded.length);
    for (let k = 0; k < wholeFolded.length; k++) { fine[2 * k] = wholeFolded[k]; fine[2 * k + 1] = halfFolded[k]; }
    return fine;
  }

  /**
   * Circular weighted centroid of a spectrum of `size` bins, bin i at angle
   * 2πi/size, weight = value^p (cyclic_weighted_centroid_accumulate/_finalize).
   *   angle in [0, 2π); magnitude = |R|²/(Σw)², in [0, 1];
   *   strength = (|R|²)^(1/(2+p)); x, y = R/Σw (the normalised mean vector)
   */
  function centroid(spectrum, p) {
    const size = spectrum.length;
    let x = 0, y = 0, sum = 0;
    for (let i = 0; i < size; i++) {
      const v = spectrum[i] > 0 ? spectrum[i] : 0;
      const w = p === 0 ? 1 : v ** p;
      x += w * Math.cos(TAU * i / size);
      y += w * Math.sin(TAU * i / size);
      sum += w;
    }
    if (sum <= 0) return { angle: 0, magnitude: 0, strength: 0, x: 0, y: 0 };
    let angle = Math.atan2(y, x);
    if (angle < 0) angle += TAU;
    const r2 = x * x + y * y;
    return { angle, magnitude: r2 / (sum * sum), strength: r2 ** (1 / (2 + p)), x: x / sum, y: y / sum };
  }

  /** Index of the largest bin (cyclic_weighted_centroid_peak_bin) */
  function peakBin(spectrum) {
    let best = 0;
    for (let i = 1; i < spectrum.length; i++) if (spectrum[i] > spectrum[best]) best = i;
    return best;
  }

  /**
   * Mean-shift refinement (cyclic_weighted_centroid_refine_peak): start at
   * startBin and move a ±halfWidth window to the power-weighted mean of what it
   * holds, `iterations` times (CWC_REFINE_ITERATIONS = 4). Bins partly inside
   * the window count proportionally. The spectrum is circular.
   * Returns { centre (bins, may be outside [0, size)), angle in [0, 2π), steps }.
   */
  function refinePeak(spectrum, startBin, halfWidth, iterations = 4) {
    const size = spectrum.length;
    let centre = startBin;
    const steps = [centre];
    for (let it = 0; it < iterations; it++) {
      const lo = centre - halfWidth, hi = centre + halfWidth;
      let moment = 0, total = 0;
      for (let k = Math.floor(lo); k <= Math.ceil(hi); k++) {
        let w = 1;
        if (k < lo) w = 1 - (lo - k);
        if (k > hi) w = 1 - (k - hi);
        if (w <= 0) continue;
        const p = w * spectrum[mod(k, size)];
        moment += k * p;
        total += p;
      }
      if (total > 0) centre = moment / total;
      steps.push(centre);
    }
    return { centre, angle: mod(centre * TAU / size, TAU), steps };
  }

  /**
   * The symbol is the strongest bin; the error is the refined angle relative
   * to it, wrapped to [−π, π) (chirp_decoder_magnitudes_to_symbol,
   * chirp_symbol_rx_set). Deciding from the strongest bin rather than by
   * rounding the refined angle is much more reliable in noise.
   * Returns { symbolIndex, errorAngle (radians) }.
   */
  function decideSymbol(peak, angle, M, bins) {
    const symbolIndex = mod(Math.floor((peak * M + Math.floor(bins / 2)) / bins), M);
    let errorAngle = angle - symbolIndex * TAU / M;
    errorAngle = mod(errorAngle + Math.PI, TAU) - Math.PI;
    return { symbolIndex, errorAngle };
  }

  /** Decoder angle error to samples (main.c timing_error_samples) */
  const errorAngleToSamples = (errorAngle, N) => errorAngle * N / TAU;

  // ---------------------------------------------------------------------------
  // The whole receive path for one frame
  // ---------------------------------------------------------------------------

  /**
   * opts: { bandPass: bool, foldMethod: 'coherent' | 'power',
   *         refineGrid: 'half' (firmware) | 'whole' (old),
   *         powerExponent: integer (firmware 4), halfWidth: bins (firmware 3) }
   * dvs: from dechirpVectors(cfg) (pass it in to avoid recomputing).
   * Returns every intermediate result, for plotting. refined.centre is in bins.
   */
  function decodeFrame(cfg, x, opts, dvs = dechirpVectors(cfg)) {
    const bp = bandPass(cfg, x, opts.bandPass);
    const dc = dechirp(bp, dvs.whole);
    const X = spectrumOf(dc.re, dc.im);
    const raw = powerOf(X);
    const f = fold(cfg, X, opts.foldMethod);
    const dcHalf = dechirp(bp, dvs.half);
    const fHalf = fold(cfg, spectrumOf(dcHalf.re, dcHalf.im), opts.foldMethod, true);
    const fine = finePower(f.folded, fHalf.folded);
    const c = centroid(f.folded, opts.powerExponent);
    const peak = peakBin(f.folded);
    const half = opts.refineGrid !== 'whole';
    const r = half ? refinePeak(fine, 2 * peak, 2 * opts.halfWidth) : refinePeak(f.folded, peak, opts.halfWidth);
    const refined = half ? { ...r, centre: r.centre / 2, steps: r.steps.map(v => v / 2) } : r;
    const sym = decideSymbol(peak, refined.angle, cfg.M, cfg.bins);
    return {
      bp, dc, X, raw, fold: f, foldHalf: fHalf, fine, centroid: c, peak, refined, sym,
      timingError: errorAngleToSamples(sym.errorAngle, cfg.N),
    };
  }

  const api = {
    TAU, mod, makeNoise, fft, spectrumOf, powerOf,
    chirpConfig, chirpPhase, dechirpVector, dechirpVectors, TEST_ECHOES, receivedSignal,
    bandPass, dechirp, fold, finePower, centroid, peakBin, refinePeak, decideSymbol, errorAngleToSamples,
    decodeFrame,
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ChirpDsp = api;
})(typeof window !== 'undefined' ? window : globalThis);
