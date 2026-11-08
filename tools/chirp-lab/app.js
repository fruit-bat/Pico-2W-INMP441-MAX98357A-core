/*
 * app.js — controls, state and wiring for the chirp lab.
 *
 * Flow: a control changes → state is updated → render() runs the decoder once
 * (ChirpDsp.decodeFrame) and each stage function below turns the result into
 * chart descriptions for ChirpCharts.
 *
 * To add a parameter: add an entry to CONTROLS (it builds the input and keeps
 * state[key] up to date), then use state[key] in render() or a stage function.
 * To add a plot: add a <figure id="..."> to index.html and fill it from a
 * stage function.
 */
'use strict';
(function () {
  const D = ChirpDsp, Charts = ChirpCharts;
  const $ = id => document.getElementById(id);

  // ---------------------------------------------------------------------------
  // Controls. Defaults match the firmware (main.c, chirp_decoder_init).
  //   type: 'number' (optionally with a slider: range [lo, hi]), 'select', 'check'
  //   int: round to an integer; min: lower limit
  // ---------------------------------------------------------------------------
  const CONTROLS = [
    { section: 'Symbol' },
    { key: 'k', label: 'Symbol <i>k</i>', type: 'number', def: 20, step: 1, int: true, range: [0, 63] },
    { key: 'M', label: 'Symbols <i>M</i>', type: 'number', def: 64, step: 1, int: true, min: 1 },
    { buttons: [{ id: 'play', label: 'Step through k' }] },

    { section: 'Chirp' },
    { key: 'L', label: 'Bins per symbol <i>L</i>', type: 'number', def: 1, step: 1, int: true, min: 1, hint: 'firmware: 1' },
    { key: 'f0', label: 'Start <i>f₀</i> / Hz', type: 'number', def: 15000, step: 100, min: 0 },
    { key: 'N', label: 'Samples <i>N</i>', type: 'select', def: '1024', options: ['256', '512', '1024', '2048', '4096'] },
    { key: 'fs', label: 'Sample rate <i>f<sub>s</sub></i> / Hz', type: 'number', def: 44100, step: 100, min: 1 },

    { section: 'Channel' },
    { key: 'offset', label: 'Window offset / samples', type: 'number', def: 0, step: 0.25, range: [-8, 8], hint: 'positive: the window starts late' },
    { key: 'ppm', label: 'Time scale / ppm', type: 'number', def: 0, step: 50, range: [-3000, 3000], hint: 'clock mismatch or Doppler; 1000 ppm ≈ 0.34 m/s' },
    { key: 'snr', label: 'SNR / dB', type: 'number', def: 0, step: 1, range: [-30, 30], hint: 'signal vs white noise over the full band' },
    { key: 'noiseOn', label: 'Add noise', type: 'check', def: true },
    { buttons: [{ id: 'reroll', label: 'New noise', primary: true }] },

    { section: 'Decoder' },
    { key: 'bandPass', label: 'Band-pass before de-chirp', type: 'check', def: true },
    { key: 'foldMethod', label: 'Fold', type: 'select', def: 'coherent', options: [['coherent', 'Coherent'], ['power', 'Power sum (old)']] },
    { key: 'p', label: 'Centroid exponent <i>p</i>', type: 'number', def: 4, step: 1, int: true, min: 0, range: [0, 8] },
    { key: 'halfWidth', label: 'Refine half-width / bins', type: 'number', def: 3, step: 1, int: true, min: 1, range: [1, 8] },

    { section: 'Display' },
    { key: 'dcView', label: 'De-chirped spectrum', type: 'select', def: 'band', options: [['band', 'Folded range'], ['full', 'Full']] },
    { buttons: [{ id: 'reset', label: 'Reset' }] },
    { note: 'Noise is held fixed while you change parameters; press New noise for a fresh draw. The symbol is repeated, so every frame is an exact cyclic shift. Settings can be given in the URL, e.g. ?k=20&offset=3.' },
  ];

  const state = { seed: 20261108 };

  function buildRack() {
    const rack = $('rack');
    let html = '', first = true;
    for (const c of CONTROLS) {
      if (c.section) {
        html += (first ? '' : '<div class="divider"></div>') + `<h2>${c.section}</h2>`;
        first = false;
      } else if (c.buttons) {
        html += '<div class="btns">' + c.buttons.map(b =>
          `<button id="${b.id}"${b.primary ? ' class="primary"' : ''} aria-pressed="false">${b.label}</button>`).join('') + '</div>';
      } else if (c.note) {
        html += `<p class="note">${c.note}</p>`;
      } else if (c.type === 'check') {
        html += `<div class="field check"><label for="${c.key}">${c.label}</label><input type="checkbox" id="${c.key}"></div>`;
      } else {
        const input = c.type === 'select'
          ? `<select id="${c.key}">${c.options.map(o => {
              const [v, t] = Array.isArray(o) ? o : [o, o];
              return `<option value="${v}">${t}</option>`;
            }).join('')}</select>`
          : `<input type="number" id="${c.key}" step="${c.step}"${c.min != null ? ` min="${c.min}"` : ''}>`;
        const slider = c.range
          ? `<input type="range" id="${c.key}R" min="${c.range[0]}" max="${c.range[1]}" step="${c.step}" aria-label="${c.key} slider">` : '';
        const hint = c.hint ? `<span class="hint" id="${c.key}Hint">${c.hint}</span>` : '';
        html += `<div class="field"><label for="${c.key}">${c.label}</label>${input}${slider}${hint}</div>`;
      }
    }
    rack.innerHTML = html;

    for (const c of CONTROLS) {
      if (!c.key) continue;
      const el = $(c.key), slider = $(c.key + 'R');
      const ev = c.type === 'number' ? 'input' : 'change';
      el.addEventListener(ev, () => { if (slider) slider.value = el.value; readControls(); });
      if (slider) slider.addEventListener('input', () => { el.value = slider.value; readControls(); });
    }
  }

  function setControls(values) {
    for (const c of CONTROLS) {
      if (!c.key || !(c.key in values)) continue;
      const el = $(c.key), slider = $(c.key + 'R');
      if (c.type === 'check') el.checked = !!values[c.key];
      else el.value = values[c.key];
      if (slider) slider.value = values[c.key];
    }
  }

  function readControls() {
    for (const c of CONTROLS) {
      if (!c.key) continue;
      const el = $(c.key);
      if (c.type === 'check') { state[c.key] = el.checked; continue; }
      if (c.type === 'select') { state[c.key] = el.value; continue; }
      let v = parseFloat(el.value);
      if (!Number.isFinite(v)) v = state[c.key] ?? c.def;
      if (c.int) v = Math.round(v);
      if (c.min != null) v = Math.max(c.min, v);
      state[c.key] = v;
    }
    state.N = parseInt(state.N, 10);
    $('kR').max = state.M - 1;
    schedule();
  }

  const defaults = () => Object.fromEntries(CONTROLS.filter(c => c.key).map(c => [c.key, c.def]));

  /** Settings from the URL, e.g. index.html?k=20&offset=3&bandPass=false */
  function urlSettings() {
    const out = {};
    for (const [key, value] of new URLSearchParams(location.search)) {
      const c = CONTROLS.find(c => c.key === key);
      if (c) out[key] = c.type === 'check' ? value !== 'false' && value !== '0' : value;
    }
    return out;
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  /** Power in dB relative to a full-amplitude tone, (N/2)^2 for amplitude 1 */
  const dbScale = N => { const full = (N / 2) ** 2; return v => 10 * Math.log10(Math.max(v / full, 1e-15)); };

  /** Bin order for a signed axis: -N/2+1 … N/2 */
  function signedOrder(N) {
    const idx = new Int32Array(N), n = new Float64Array(N);
    for (let j = 0; j < N; j++) { idx[j] = D.mod(j - N / 2 + 1, N); n[j] = j - N / 2 + 1; }
    return { idx, n };
  }

  /** dB axis covering the data, 90 dB deep */
  function dbDomain(...arrays) {
    let top = -Infinity;
    for (const a of arrays) for (const v of a) top = Math.max(top, v);
    top = Math.ceil(top / 10) * 10 + 5;
    return [top - 95, top];
  }

  const range = (lo, hi) => Float64Array.from({ length: hi - lo + 1 }, (_, i) => lo + i);
  const signed = (v, d) => (v >= 0 ? '+' : '') + v.toFixed(d);
  const db = v => (v >= 0 ? '+' : '') + v.toFixed(2) + ' dB';

  let dvCache = { key: '', dv: null }, noiseCache = { key: '', z: null };

  // ---------------------------------------------------------------------------
  // Render
  // ---------------------------------------------------------------------------

  function render() {
    const s = state;
    const cfg = D.chirpConfig({ M: s.M, L: s.L, N: s.N, fs: s.fs, f0: s.f0 });

    const dvKey = [s.N, s.fs, s.f0, s.L, s.M].join('|');
    if (dvCache.key !== dvKey) dvCache = { key: dvKey, dv: D.dechirpVector(cfg) };
    const noiseKey = s.N + '|' + s.seed;
    if (noiseCache.key !== noiseKey) noiseCache = { key: noiseKey, z: D.makeNoise(s.N, s.seed) };

    const k = D.mod(s.k, s.M);
    const x = D.receivedSignal(cfg, { k, offset: s.offset, ppm: s.ppm, snrDb: s.noiseOn ? s.snr : null }, noiseCache.z);
    const r = D.decodeFrame(cfg, x, {
      bandPass: s.bandPass, foldMethod: s.foldMethod, powerExponent: s.p, halfWidth: s.halfWidth,
    }, dvCache.dv);

    // Where the tone should be, in bins: symbol, window offset, and the
    // frequency shift a time scale causes (about ε·(f0 + B) Hz on average)
    const dopplerBins = s.ppm * 1e-6 * (cfg.f0 + cfg.B) / cfg.binHz;
    const truePos = k * cfg.L + s.offset * cfg.bins / cfg.N + dopplerBins;
    const ctx = { s, cfg, x, r, k, truePos, dopplerBins, toDb: dbScale(cfg.N), full: (cfg.N / 2) ** 2 };

    stageReceived(ctx);
    stageBandPass(ctx);
    stageDechirp(ctx);
    stageFold(ctx);
    stageDecide(ctx);
    readouts(ctx);
    parameters(ctx);
  }

  // 1. Received signal
  function stageReceived({ cfg, x, r, toDb }) {
    const n = range(0, cfg.N - 1);
    Charts.xy($('figRxTime'), {
      series: [{ x: n, y: x, name: 'x', color: 're', width: 1 }],
      xDomain: [0, cfg.N - 1], yDomain: Charts.symmetricExtent(x), xLabel: 'sample',
    });
    const half = cfg.N / 2, hz = new Float64Array(half + 1), p = new Float64Array(half + 1);
    const before = D.powerOf(r.bp.before);
    for (let i = 0; i <= half; i++) { hz[i] = i * cfg.binHz; p[i] = toDb(before[i]); }
    Charts.xy($('figRxSpec'), {
      series: [{ x: hz, y: p, name: 'dB', color: 're', width: 1 }],
      xDomain: [0, cfg.fs / 2], yDomain: dbDomain(p), xLabel: 'Hz',
      bands: [{ from: cfg.f0, to: cfg.f0 + cfg.B }], tipFmt: v => v.toFixed(1),
    });
  }

  // 2. Band-pass
  function stageBandPass({ s, cfg, r, toDb }) {
    const n = range(0, cfg.N - 1);
    Charts.xy($('figBpTime'), {
      series: [{ x: n, y: r.bp.re, name: 'Re', color: 're', width: 1 }, { x: n, y: r.bp.im, name: 'Im', color: 'im', width: 1 }],
      xDomain: [0, cfg.N - 1], yDomain: Charts.symmetricExtent(r.bp.re, r.bp.im), xLabel: 'sample',
    });
    const { idx, n: sn } = signedOrder(cfg.N);
    const hz = sn.map(v => v * cfg.binHz);
    const pb = D.powerOf(r.bp.before), pa = D.powerOf(r.bp.after);
    const before = Float64Array.from(idx, i => toDb(pb[i])), after = Float64Array.from(idx, i => toDb(pa[i]));
    Charts.xy($('figBpSpec'), {
      series: [{ x: hz, y: before, name: 'before', color: 'mark', width: 1 }, { x: hz, y: after, name: 'after', color: 're', width: 1.25 }],
      xDomain: [hz[0], hz[hz.length - 1]], yDomain: dbDomain(before), xLabel: 'Hz',
      bands: s.bandPass ? [{ from: cfg.bandFirst * cfg.binHz, to: cfg.bandLast * cfg.binHz }] : [],
      tipFmt: v => v.toFixed(1),
    });
  }

  // 3. De-chirp
  function stageDechirp({ s, cfg, r, k, truePos, toDb }) {
    const n = range(0, cfg.N - 1);
    // The shifted chirp wraps where (sample + shift) passes N
    const wrap = D.mod(cfg.N - (k * cfg.samplesPerSymbol + s.offset), cfg.N);
    Charts.xy($('figDcTime'), {
      series: [{ x: n, y: r.dc.re, name: 'Re', color: 're', width: 1 }, { x: n, y: r.dc.im, name: 'Im', color: 'im', width: 1 }],
      xDomain: [0, cfg.N - 1], yDomain: Charts.symmetricExtent(r.dc.re, r.dc.im), xLabel: 'sample',
      markers: wrap > 0.5 && wrap < cfg.N - 0.5 ? [{ x: wrap, label: 'wrap' }] : [],
    });
    const { idx, n: sn } = signedOrder(cfg.N);
    const p = Float64Array.from(idx, i => toDb(r.raw[i]));
    const B = cfg.bins;
    const xDomain = s.dcView === 'full' ? [sn[0], sn[sn.length - 1]] : [-B - 16, B + 16];
    const lo = sn.findIndex(v => v >= xDomain[0]), hi = sn.findIndex(v => v > xDomain[1]);
    const view = [lo, hi < 0 ? sn.length : hi];
    const tp = D.mod(truePos, B);
    Charts.xy($('figDcSpec'), {
      series: [{ x: sn.slice(...view), y: p.slice(...view), name: 'dB', color: 're', mode: s.dcView === 'full' ? 'line' : 'linepoints', width: 1 }],
      xDomain, yDomain: dbDomain(p), xLabel: 'bin',
      bands: [{ from: -B - 0.5, to: -0.5 }, { from: -0.5, to: B - 0.5 }],
      markers: [{ x: tp, label: 'kL' }, { x: tp - B, label: 'kL − B' }],
      tipFmt: v => v.toFixed(1),
    });
  }

  // 4. Fold
  function stageFold({ s, cfg, r, truePos, full }) {
    const B = cfg.bins, bins = range(0, B - 1);
    const coherent = r.fold.coherent.map(v => v / full), power = r.fold.power.map(v => v / full);
    const chosenIsCoherent = s.foldMethod !== 'power';
    const [a, b] = chosenIsCoherent ? [coherent, power] : [power, coherent];
    $('foldLegendA').textContent = chosenIsCoherent ? 'coherent (in use)' : 'power sum (in use)';
    $('foldLegendB').textContent = chosenIsCoherent ? 'power sum (old)' : 'coherent (firmware)';
    Charts.xy($('figFold'), {
      series: [{ x: bins, y: a, name: chosenIsCoherent ? 'coherent' : 'power sum', color: 're', mode: 'bars' },
               { x: bins, y: b, name: chosenIsCoherent ? 'power sum' : 'coherent', color: 'im', mode: 'linepoints', width: 1 }],
      xDomain: [-0.5, B - 0.5], yDomain: [0, Charts.niceTop([...a, ...b])], xLabel: 'bin',
      markers: [{ x: r.peak, label: 'peak' }, { x: D.mod(truePos, B), label: 'true' }],
      tipFmt: v => v.toFixed(4),
    });
  }

  // 5. Decide
  function stageDecide({ s, cfg, r, truePos, full }) {
    const B = cfg.bins, f = r.fold.folded;
    let max = 0;
    for (const v of f) max = Math.max(max, v);
    const weights = Array.from(f, v => (max > 0 ? (v / max) ** s.p : 0));
    Charts.polar($('figPolar'), {
      spokes: weights.map((w, i) => ({ x: w * Math.cos(D.TAU * i / B), y: w * Math.sin(D.TAU * i / B) })),
      vector: { x: r.centroid.x, y: r.centroid.y },
      guide: D.TAU * D.mod(truePos, B) / B,
      tip: i => `<div>bin ${i}</div><div>folded ${(f[i] / full).toFixed(4)} of full tone</div><div>weight ${weights[i].toFixed(4)}</div>`,
    });

    const hw = s.halfWidth, centre = r.refined.centre;
    const xs = range(r.peak - hw - 4, r.peak + hw + 4);
    const ys = xs.map(i => f[D.mod(i, B)] / full);
    const tp = truePos + B * Math.round((centre - truePos) / B);
    Charts.xy($('figRefine'), {
      series: [{ x: xs, y: ys, name: 'folded', color: 're', mode: 'bars' }],
      xDomain: [xs[0] - 0.5, xs[xs.length - 1] + 0.5], yDomain: [0, Charts.niceTop(ys)], xLabel: 'bin',
      bands: [{ from: centre - hw, to: centre + hw }],
      markers: [{ x: centre, label: `refined ${centre.toFixed(2)}` }, { x: tp, label: `true ${tp.toFixed(2)}` }],
      tipFmt: v => v.toFixed(4),
    });
  }

  function readouts({ s, cfg, r, k, dopplerBins }) {
    const ok = r.sym.symbolIndex === k;
    $('roK').textContent = r.sym.symbolIndex;
    $('roK').className = 'v ' + (ok ? 'good' : 'warn');
    $('roKU').textContent = ok ? `matches k = ${k}` : `wrong, sent ${k}`;

    $('roPos').textContent = D.mod(r.refined.centre, cfg.bins).toFixed(3);
    $('roPosU').textContent = `bins · peak bin ${r.peak}`;

    // Where the tone's energy really is (offset plus Doppler bias), relative to
    // the nearest symbol. The decoder's estimate falls short of this for small
    // offsets: the mean shift is pulled towards the bin centre.
    const step = cfg.samplesPerSymbol;
    const biasSamples = dopplerBins * cfg.N / cfg.bins;
    let expected = s.offset + biasSamples;
    expected -= step * Math.round(expected / step);
    $('roTe').textContent = signed(r.timingError, 2);
    $('roTeU').textContent = 'samples, what the tracker sees';
    $('roTx').textContent = signed(expected, 2);
    $('roTxU').textContent = `offset ${signed(s.offset, 2)} + Doppler ${signed(biasSamples, 2)}; decoder under-reads small values`;

    $('roMag').textContent = r.centroid.magnitude.toFixed(3);
    $('roMag').className = 'v ' + (r.centroid.magnitude >= 0.8 ? 'good' : 'warn');
    const st = r.centroid.strength;
    $('roStr').textContent = st >= 1e4 || (st > 0 && st < 1e-2) ? st.toExponential(2) : st.toFixed(2);

    const pk = r.peak;
    $('roGain').textContent = db(10 * Math.log10(r.fold.coherent[pk] / r.fold.power[pk]));

    const f = r.fold.folded;
    let next = 0;
    for (let i = 0; i < f.length; i++) {
      const d = Math.min(D.mod(i - pk, f.length), D.mod(pk - i, f.length));
      if (d > 1) next = Math.max(next, f[i]);
    }
    $('roMargin').textContent = next > 0 ? db(10 * Math.log10(f[pk] / next)) : '∞';
  }

  function parameters({ cfg }) {
    $('nLabel').textContent = cfg.N;
    $('binsLabel').textContent = cfg.bins;
    $('bandLabel').textContent = `${cfg.bandFirst} … ${cfg.bandLast}`;
    $('rotLabel').textContent = `${(cfg.foldTurns * 360).toFixed(1)}°`;
    const biasPer1000 = 1000e-6 * (cfg.f0 + cfg.B) / cfg.binHz * cfg.N / cfg.bins;
    const rows = [
      ['Bandwidth B', `${cfg.B.toFixed(2)} Hz`, `${cfg.bins} bins of ${cfg.binHz.toFixed(3)} Hz`],
      ['Symbol duration T', `${(cfg.T * 1000).toFixed(3)} ms`, `${cfg.N} samples`],
      ['Chirp rate', `${(cfg.chirpRate / 1000).toFixed(2)} kHz/s`, 'B / T'],
      ['One symbol step', `${cfg.samplesPerSymbol} samples`, `= ${cfg.L} bin${cfg.L > 1 ? 's' : ''}`],
      ['One sample of offset', `${(cfg.bins / cfg.N).toFixed(4)} bins`, 'time and frequency share one axis'],
      ['Band-pass', `bins ${cfg.bandFirst} … ${cfg.bandLast}`, `${(cfg.bandFirst * cfg.binHz).toFixed(0)} … ${(cfg.bandLast * cfg.binHz).toFixed(0)} Hz`],
      ['Fold rotation R', `${(cfg.foldTurns * 360).toFixed(3)}°`, 'frac(f0·N/fs − B/2) turns'],
      ['Doppler bias per 1000 ppm', `${biasPer1000.toFixed(2)} samples`, '1000 ppm ≈ 0.34 m/s, or a 0.1% clock error'],
    ];
    $('paramRows').innerHTML = rows.map(([q, v, n]) => `<tr><td>${q}</td><td>${v}</td><td>${n}</td></tr>`).join('');
  }

  // ---------------------------------------------------------------------------
  // Start-up and events
  // ---------------------------------------------------------------------------

  let raf = 0;
  function schedule() { cancelAnimationFrame(raf); raf = requestAnimationFrame(render); }

  let playTimer = null;
  function stopPlay() {
    clearInterval(playTimer); playTimer = null;
    $('play').setAttribute('aria-pressed', 'false'); $('play').textContent = 'Step through k';
  }

  buildRack();
  setControls({ ...defaults(), ...urlSettings() });

  $('play').addEventListener('click', () => {
    if (playTimer) return stopPlay();
    $('play').setAttribute('aria-pressed', 'true'); $('play').textContent = 'Stop';
    playTimer = setInterval(() => { setControls({ k: D.mod(state.k + 1, state.M) }); readControls(); }, 600);
  });
  $('reroll').addEventListener('click', () => { state.seed = (Math.random() * 2 ** 32) >>> 0; schedule(); });
  $('reset').addEventListener('click', () => { stopPlay(); setControls(defaults()); readControls(); });

  new ResizeObserver(() => Charts.redrawAll()).observe(document.querySelector('main'));
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => Charts.redrawAll());
  new MutationObserver(() => Charts.redrawAll()).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  if (document.fonts) document.fonts.ready.then(() => Charts.redrawAll());
  readControls();
})();
