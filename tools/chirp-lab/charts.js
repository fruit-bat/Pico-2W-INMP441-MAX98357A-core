/*
 * charts.js — small canvas charts for the chirp lab. Knows nothing about chirps.
 *
 *   ChirpCharts.xy(figure, cfg)      line / point / bar chart with hover readout
 *   ChirpCharts.polar(figure, data)  polar spokes plus a mean vector
 *   ChirpCharts.redrawAll()          after a resize or theme change
 *
 * `figure` is a <figure> containing a <canvas>. Colours come from CSS custom
 * properties (see style.css); series refer to them by name: 're', 'im',
 * 'mark', 'accent'.
 *
 * xy cfg:
 *   series:  [{ x, y, name, color, mode: 'line' | 'linepoints' | 'bars', width, barWidth }]
 *            barWidth: bar spacing in x units (default 1)
 *            x must be increasing; the hover readout follows series[0].
 *   xDomain, yDomain: [lo, hi]
 *   xLabel:  axis label (also used in the hover readout)
 *   markers: [{ x, label }]                 dashed vertical lines
 *   bands:   [{ from, to, color: 'band' | 'band2' }]  shaded x ranges
 *   yFmt(v): tick label;  tipFmt(v): hover value
 *
 * polar data:
 *   spokes:  [{ x, y }]  in the unit disc (y up)
 *   vector:  { x, y }    mean vector, drawn with an arrow head
 *   guide:   angle in radians for a dashed reference line, or null
 *   tip(i):  HTML for the hover readout of spoke i
 */
'use strict';
(function (root) {
  const TAU = 2 * Math.PI;
  const figures = new Set();

  function colours() {
    const s = getComputedStyle(document.documentElement);
    const g = n => s.getPropertyValue(n).trim();
    return {
      fg: g('--fg'), fg2: g('--fg-2'), fg3: g('--fg-3'), grid: g('--grid'), line: g('--line'), panel: g('--panel'),
      re: g('--re'), im: g('--im'), mark: g('--mark'), accent: g('--accent'),
      band: g('--band'), band2: g('--band-2'), font: g('--font-data'),
    };
  }

  /** Round tick values covering [lo, hi] with about `count` steps */
  function niceTicks(lo, hi, count) {
    const span = hi - lo || 1;
    const raw = span / count, mag = 10 ** Math.floor(Math.log10(raw));
    const step = [1, 2, 2.5, 5, 10].map(m => m * mag).find(s => s >= raw);
    const out = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
    return out;
  }

  /** A tidy upper limit at or above max(values) */
  function niceTop(values) {
    let m = 0;
    for (const v of values) if (v > m) m = v;
    const t = niceTicks(0, m || 1, 4);
    return t[t.length - 1] < m ? t[t.length - 1] + (t[1] - t[0]) : t[t.length - 1];
  }

  /** A symmetric range [-top, top] covering all values of all arrays */
  function symmetricExtent(...arrays) {
    let m = 0;
    for (const a of arrays) for (const v of a) m = Math.max(m, Math.abs(v));
    const top = niceTop([m]);
    return [-top, top];
  }

  function setupCanvas(cv) {
    const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr);
      cv.height = Math.round(h * dpr);
    }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
  }

  function tipFor(fig) {
    let t = fig.querySelector('.tip');
    if (!t) {
      t = document.createElement('div');
      t.className = 'tip';
      t.hidden = true;
      fig.appendChild(t);
    }
    return t;
  }

  function placeTip(fig, tip, x, y) {
    tip.hidden = false;
    const fw = fig.clientWidth, tw = tip.offsetWidth;
    tip.style.left = Math.min(Math.max(4, x + 14), fw - tw - 4) + 'px';
    tip.style.top = Math.max(4, y - 10) + 'px';
  }

  // ---------------------------------------------------------------------------
  // XY chart
  // ---------------------------------------------------------------------------

  function drawXY(fig) {
    const cfg = fig._cfg;
    if (!cfg) return;
    const cv = fig.querySelector('canvas'), { ctx, w, h } = setupCanvas(cv), C = colours();
    const pad = { l: 48, r: 10, t: 12, b: 32 };
    const [x0, x1] = cfg.xDomain, [y0, y1] = cfg.yDomain;
    const X = v => pad.l + (v - x0) / (x1 - x0) * (w - pad.l - pad.r);
    const Y = v => h - pad.b - (v - y0) / (y1 - y0) * (h - pad.t - pad.b);
    fig._pad = pad;
    ctx.font = `11px ${C.font}`;
    ctx.lineWidth = 1;

    for (const b of cfg.bands || []) {
      const a = Math.max(b.from, x0), z = Math.min(b.to, x1);
      if (z <= a) continue;
      ctx.fillStyle = C[b.color || 'band'];
      ctx.fillRect(X(a), pad.t, X(z) - X(a), h - pad.t - pad.b);
    }

    // Grid and ticks
    ctx.strokeStyle = C.grid; ctx.fillStyle = C.fg3; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (const v of niceTicks(y0, y1, 4)) {
      ctx.beginPath(); ctx.moveTo(pad.l, Y(v) + 0.5); ctx.lineTo(w - pad.r, Y(v) + 0.5); ctx.stroke();
      ctx.fillText((cfg.yFmt || String)(v), pad.l - 6, Y(v));
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (const v of niceTicks(x0, x1, Math.max(3, Math.floor((w - pad.l) / 80)))) {
      ctx.strokeStyle = C.grid;
      ctx.beginPath(); ctx.moveTo(X(v) + 0.5, pad.t); ctx.lineTo(X(v) + 0.5, h - pad.b); ctx.stroke();
      ctx.fillText(String(v), X(v), h - pad.b + 5);
    }
    ctx.strokeStyle = C.line;
    ctx.beginPath(); ctx.moveTo(pad.l, h - pad.b + 0.5); ctx.lineTo(w - pad.r, h - pad.b + 0.5); ctx.stroke();
    ctx.fillStyle = C.fg3; ctx.textAlign = 'right';
    ctx.fillText(cfg.xLabel, w - pad.r, h - 13);

    // Markers; labels alternate sides so neighbours don't collide
    (cfg.markers || []).forEach((m, i) => {
      if (m.x < x0 || m.x > x1) return;
      ctx.strokeStyle = C.mark; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(X(m.x) + 0.5, pad.t); ctx.lineTo(X(m.x) + 0.5, h - pad.b); ctx.stroke();
      ctx.setLineDash([]);
      const right = i % 2 === 0 ? X(m.x) < w - 70 : X(m.x) < pad.l + 60;
      ctx.fillStyle = C.fg2; ctx.textAlign = right ? 'left' : 'right'; ctx.textBaseline = 'top';
      ctx.fillText(m.label, X(m.x) + (right ? 4 : -4), pad.t + 2 + i * 13);
    });

    // Series, clipped to the plot area
    ctx.save();
    ctx.beginPath(); ctx.rect(pad.l, pad.t - 2, w - pad.l - pad.r, h - pad.t - pad.b + 4); ctx.clip();
    for (const s of cfg.series) {
      const col = C[s.color];
      ctx.strokeStyle = col; ctx.fillStyle = col;
      if (s.mode === 'bars') {
        const bw = Math.max(1, Math.abs(X(s.barWidth || 1) - X(0)) - 2);
        const base = Y(Math.max(y0, 0));
        for (let i = 0; i < s.x.length; i++) {
          const top = Y(s.y[i]), r = Math.max(0, Math.min(3, bw / 2, (base - top) / 2));
          ctx.beginPath();
          if (ctx.roundRect) ctx.roundRect(X(s.x[i]) - bw / 2, top, bw, base - top, [r, r, 0, 0]);
          else ctx.rect(X(s.x[i]) - bw / 2, top, bw, base - top);
          ctx.fill();
        }
      } else {
        ctx.lineWidth = s.width || 1.25; ctx.lineJoin = 'round';
        ctx.beginPath();
        for (let i = 0; i < s.x.length; i++) {
          const px = X(s.x[i]), py = Y(s.y[i]);
          i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
        }
        ctx.stroke();
        if (s.mode === 'linepoints') {
          for (let i = 0; i < s.x.length; i++) { ctx.beginPath(); ctx.arc(X(s.x[i]), Y(s.y[i]), 2.5, 0, TAU); ctx.fill(); }
        }
      }
    }
    ctx.restore();

    // Hover cursor
    const hi = fig._hi;
    if (hi != null && hi < cfg.series[0].x.length) {
      const hx = X(cfg.series[0].x[hi]);
      ctx.strokeStyle = C.fg3;
      ctx.beginPath(); ctx.moveTo(hx + 0.5, pad.t); ctx.lineTo(hx + 0.5, h - pad.b); ctx.stroke();
      for (const s of cfg.series) {
        if (hi >= s.y.length) continue;
        ctx.beginPath(); ctx.arc(X(s.x[hi]), Y(s.y[hi]), 4, 0, TAU);
        ctx.fillStyle = C[s.color]; ctx.fill();
        ctx.lineWidth = 2; ctx.strokeStyle = C.panel; ctx.stroke(); ctx.lineWidth = 1;
      }
    }
  }

  function hookXY(fig) {
    const cv = fig.querySelector('canvas'), tip = tipFor(fig);
    const move = e => {
      const cfg = fig._cfg;
      if (!cfg || !fig._pad) return;
      const rect = cv.getBoundingClientRect(), mx = e.clientX - rect.left, pad = fig._pad;
      const xs = cfg.series[0].x, [x0, x1] = cfg.xDomain;
      const xv = x0 + (mx - pad.l) / (rect.width - pad.l - pad.r) * (x1 - x0);
      let lo = 0, hi = xs.length - 1;
      while (hi - lo > 1) { const m = (lo + hi) >> 1; xs[m] < xv ? (lo = m) : (hi = m); }
      const i = Math.abs(xs[lo] - xv) < Math.abs(xs[hi] - xv) ? lo : hi;
      fig._hi = i;
      drawXY(fig);
      const C = colours(), f = cfg.tipFmt || (v => v.toFixed(3));
      tip.innerHTML = `<div>${cfg.xLabel} ${+xs[i].toFixed(3)}</div>` + cfg.series
        .filter(s => i < s.y.length)
        .map(s => `<div><span class="sw" style="background:${C[s.color]}"></span>${s.name} ${f(s.y[i])}</div>`).join('');
      placeTip(fig, tip, cv.offsetLeft + mx, cv.offsetTop + (e.clientY - rect.top));
    };
    cv.addEventListener('pointermove', move);
    cv.addEventListener('pointerdown', move);
    cv.addEventListener('pointerleave', () => { fig._hi = null; tip.hidden = true; drawXY(fig); });
  }

  // ---------------------------------------------------------------------------
  // Polar chart
  // ---------------------------------------------------------------------------

  function drawPolar(fig) {
    const d = fig._data;
    if (!d) return;
    const cv = fig.querySelector('canvas'), { ctx, w, h } = setupCanvas(cv), C = colours();
    const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 22;
    ctx.font = `11px ${C.font}`; ctx.lineWidth = 1; ctx.strokeStyle = C.grid;
    for (const r of [0.25, 0.5, 0.75, 1]) { ctx.beginPath(); ctx.arc(cx, cy, r * R, 0, TAU); ctx.stroke(); }
    ctx.fillStyle = C.fg3; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    for (let a = 0; a < 360; a += 45) {
      const t = a * Math.PI / 180;
      ctx.strokeStyle = C.grid;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R * Math.cos(t), cy - R * Math.sin(t)); ctx.stroke();
      ctx.fillText((a > 180 ? a - 360 : a) + '°', cx + (R + 12) * Math.cos(t), cy - (R + 12) * Math.sin(t));
    }
    if (d.guide != null) {
      ctx.strokeStyle = C.mark; ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R * Math.cos(d.guide), cy - R * Math.sin(d.guide)); ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.strokeStyle = C.re; ctx.fillStyle = C.re; ctx.lineWidth = 1.5;
    d.spokes.forEach((s, i) => {
      const x = cx + s.x * R, y = cy - s.y * R;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
      ctx.beginPath(); ctx.arc(x, y, fig._hi === i ? 5 : 2.5, 0, TAU); ctx.fill();
    });
    // Mean vector, at least long enough to read its direction
    const len = Math.hypot(d.vector.x, d.vector.y), scale = len > 0 ? Math.max(len, 0.18) / len : 0;
    const ex = cx + d.vector.x * scale * R, ey = cy - d.vector.y * scale * R;
    ctx.strokeStyle = C.im; ctx.fillStyle = C.im; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
    const ang = Math.atan2(cy - ey, ex - cx);
    ctx.beginPath(); ctx.moveTo(ex, ey);
    ctx.lineTo(ex - 9 * Math.cos(ang - 0.4), ey + 9 * Math.sin(ang - 0.4));
    ctx.lineTo(ex - 9 * Math.cos(ang + 0.4), ey + 9 * Math.sin(ang + 0.4));
    ctx.closePath(); ctx.fill();
    fig._geom = { cx, cy, R };
  }

  function hookPolar(fig) {
    const cv = fig.querySelector('canvas'), tip = tipFor(fig);
    const move = e => {
      const d = fig._data, g = fig._geom;
      if (!d || !g) return;
      const rect = cv.getBoundingClientRect(), mx = e.clientX - rect.left, my = e.clientY - rect.top;
      let best = -1, bd = Infinity;
      d.spokes.forEach((s, i) => {
        const dd = Math.hypot(g.cx + s.x * g.R - mx, g.cy - s.y * g.R - my);
        if (dd < bd) { bd = dd; best = i; }
      });
      if (bd > 24) { fig._hi = null; tip.hidden = true; drawPolar(fig); return; }
      fig._hi = best;
      drawPolar(fig);
      tip.innerHTML = d.tip ? d.tip(best) : String(best);
      placeTip(fig, tip, cv.offsetLeft + mx, cv.offsetTop + my);
    };
    cv.addEventListener('pointermove', move);
    cv.addEventListener('pointerdown', move);
    cv.addEventListener('pointerleave', () => { fig._hi = null; tip.hidden = true; drawPolar(fig); });
  }

  // ---------------------------------------------------------------------------

  function xy(fig, cfg) {
    if (!fig._kind) { fig._kind = 'xy'; hookXY(fig); figures.add(fig); }
    fig._cfg = cfg;
    drawXY(fig);
  }

  function polar(fig, data) {
    if (!fig._kind) { fig._kind = 'polar'; hookPolar(fig); figures.add(fig); }
    fig._data = data;
    drawPolar(fig);
  }

  function redrawAll() {
    for (const f of figures) (f._kind === 'xy' ? drawXY : drawPolar)(f);
  }

  root.ChirpCharts = { xy, polar, redrawAll, niceTicks, niceTop, symmetricExtent };
})(window);
