// Lightweight canvas charts with crosshair tooltips (line + horizontal bar).
import { h } from './dom.js';

export const SERIES = ['--series-1', '--series-2', '--series-3', '--series-4'];
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const tooltip = () => document.getElementById('tooltip');

function niceTicks(lo, hi, n = 5) {
  if (!(hi > lo)) { hi = lo + 1; lo -= 1; }
  const span = hi - lo;
  const step0 = span / n;
  const mag = 10 ** Math.floor(Math.log10(step0));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => span / s <= n + 0.5);
  const t0 = Math.ceil(lo / step - 1e-9) * step;
  const out = [];
  for (let t = t0; t <= hi + step * 1e-9; t += step) out.push(+t.toFixed(10));
  return { ticks: out, step };
}
const fmtTick = (v, step) => {
  const d = step >= 1 ? 0 : Math.min(4, Math.ceil(-Math.log10(step)));
  return Math.abs(v) < 1e-12 ? '0' : v.toFixed(d);
};

/**
 * Line chart. spec: { title, xLabel, yLabel, series: [{name, x, y, color?, dash?}], height,
 *  yZero?: include 0 in range, vlines: [{x, label}], hlines: [{y, label}], xFmt, yFmt, symmetric }
 */
export function lineChart(spec) {
  const wrap = h('div', { class: 'chart' });
  if (spec.title) wrap.append(h('div', { class: 'ctitle' }, spec.title));
  const canvas = h('canvas', { role: 'img', 'aria-label': spec.title || 'chart' });
  wrap.append(canvas);
  const resolve = (c) => (c && c.startsWith('var(') ? css(c.slice(4, -1)) : c);
  const colors = spec.series.map((s, i) => resolve(s.color) || css(SERIES[i % SERIES.length]));
  if (spec.series.length > 1) {
    wrap.append(h('div', { class: 'clegend' }, spec.series.map((s, i) => h('span', {}, h('i', { style: { background: colors[i], opacity: s.dash ? 0.7 : 1 } }), s.name))));
  }
  const H = spec.height || 150;
  let geom = null;
  const draw = (hover = null) => {
    const W = wrap.clientWidth || 360;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = W * dpr; canvas.height = H * dpr; canvas.style.height = `${H}px`;
    const g = canvas.getContext('2d');
    g.scale(dpr, dpr);
    g.clearRect(0, 0, W, H);
    const all = spec.series.flatMap((s) => s.x.map((x, i) => [x, s.y[i]])).filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
    if (!all.length) { g.fillStyle = css('--text-muted'); g.font = "12px 'Barlow', sans-serif"; g.fillText('No data', 10, 20); return; }
    let xmin = Math.min(...all.map((p) => p[0])), xmax = Math.max(...all.map((p) => p[0]));
    let ymin = Math.min(...all.map((p) => p[1])), ymax = Math.max(...all.map((p) => p[1]));
    for (const l of spec.hlines || []) { ymin = Math.min(ymin, l.y); ymax = Math.max(ymax, l.y); }
    if (spec.yZero) { ymin = Math.min(0, ymin); ymax = Math.max(0, ymax); }
    if (spec.minSpan && ymax - ymin < spec.minSpan) { const c = (ymax + ymin) / 2; ymin = c - spec.minSpan / 2; ymax = c + spec.minSpan / 2; }
    const pad = (ymax - ymin) * 0.08 || 1;
    ymin -= pad; ymax += pad;
    const yt = niceTicks(ymin, ymax, 4), xt = niceTicks(xmin, xmax, 5);
    ymin = Math.min(ymin, yt.ticks[0]); ymax = Math.max(ymax, yt.ticks[yt.ticks.length - 1]);
    const ml = 44, mr = 10, mt = 8, mb = spec.xLabel ? 32 : 20;
    const X = (x) => ml + ((x - xmin) / (xmax - xmin || 1)) * (W - ml - mr);
    const Y = (y) => mt + (1 - (y - ymin) / (ymax - ymin || 1)) * (H - mt - mb);
    geom = { X, Y, xmin, xmax, ml, mr, W };
    g.font = "11px 'Barlow', sans-serif";
    g.strokeStyle = css('--grid'); g.lineWidth = 1; g.fillStyle = css('--text-muted');
    g.textAlign = 'right'; g.textBaseline = 'middle';
    for (const t of yt.ticks) {
      if (t < ymin || t > ymax) continue;
      g.beginPath(); g.moveTo(ml, Y(t) + 0.5); g.lineTo(W - mr, Y(t) + 0.5); g.stroke();
      g.fillText(fmtTick(t, yt.step), ml - 6, Y(t));
    }
    g.textAlign = 'center'; g.textBaseline = 'top';
    for (const t of xt.ticks) g.fillText(fmtTick(t, xt.step), X(t), H - mb + 5);
    if (spec.xLabel) g.fillText(spec.xLabel, (ml + W - mr) / 2, H - 13);
    if (spec.yLabel) { g.save(); g.translate(10, (mt + H - mb) / 2); g.rotate(-Math.PI / 2); g.textBaseline = 'middle'; g.fillText(spec.yLabel, 0, 0); g.restore(); }
    // zero line
    if (ymin < 0 && ymax > 0) { g.strokeStyle = css('--border-strong'); g.beginPath(); g.moveTo(ml, Y(0) + 0.5); g.lineTo(W - mr, Y(0) + 0.5); g.stroke(); }
    for (const l of spec.hlines || []) {
      g.strokeStyle = css('--text-muted'); g.setLineDash([4, 4]); g.beginPath(); g.moveTo(ml, Y(l.y)); g.lineTo(W - mr, Y(l.y)); g.stroke(); g.setLineDash([]);
      if (l.label) { g.fillStyle = css('--text-muted'); g.textAlign = 'right'; g.textBaseline = 'bottom'; g.fillText(l.label, W - mr - 2, Y(l.y) - 2); }
    }
    for (const l of spec.vlines || []) {
      if (l.x < xmin || l.x > xmax) continue;
      g.strokeStyle = l.color || css('--text-muted'); g.setLineDash([3, 3]); g.beginPath(); g.moveTo(X(l.x), mt); g.lineTo(X(l.x), H - mb); g.stroke(); g.setLineDash([]);
      if (l.label) { g.fillStyle = css('--text-secondary'); g.textAlign = 'left'; g.textBaseline = 'top'; g.fillText(l.label, X(l.x) + 3, mt); }
    }
    spec.series.forEach((s, i) => {
      g.strokeStyle = colors[i]; g.lineWidth = 2; g.lineJoin = 'round';
      g.setLineDash(s.dash ? [5, 4] : []);
      g.beginPath();
      let started = false;
      s.x.forEach((x, k) => {
        const y = s.y[k];
        if (!Number.isFinite(y)) { started = false; return; }
        if (!started) { g.moveTo(X(x), Y(y)); started = true; } else g.lineTo(X(x), Y(y));
      });
      g.stroke();
      g.setLineDash([]);
      if (s.markers) for (let k = 0; k < s.x.length; k++) { g.fillStyle = colors[i]; g.beginPath(); g.arc(X(s.x[k]), Y(s.y[k]), 4, 0, 7); g.fill(); }
    });
    for (const m of spec.markers || []) {
      g.fillStyle = m.color || css('--text-primary'); g.strokeStyle = css('--surface-1'); g.lineWidth = 2;
      g.beginPath(); g.arc(X(m.x), Y(m.y), 5, 0, 7); g.fill(); g.stroke();
      if (m.label) { g.fillStyle = css('--text-secondary'); g.textAlign = 'left'; g.textBaseline = 'bottom'; g.fillText(m.label, X(m.x) + 7, Y(m.y) - 3); }
    }
    if (hover !== null) {
      const hx = X(hover);
      g.strokeStyle = css('--text-muted'); g.lineWidth = 1; g.beginPath(); g.moveTo(hx + 0.5, mt); g.lineTo(hx + 0.5, H - mb); g.stroke();
      spec.series.forEach((s, i) => {
        const k = nearestIdx(s.x, hover);
        if (k < 0 || !Number.isFinite(s.y[k])) return;
        g.fillStyle = colors[i]; g.strokeStyle = css('--surface-1'); g.lineWidth = 2;
        g.beginPath(); g.arc(X(s.x[k]), Y(s.y[k]), 4, 0, 7); g.fill(); g.stroke();
      });
    }
  };
  const nearestIdx = (xs, x) => { let b = -1, bd = Infinity; xs.forEach((v, i) => { const d = Math.abs(v - x); if (d < bd) { bd = d; b = i; } }); return b; };
  canvas.addEventListener('mousemove', (e) => {
    if (!geom) return;
    const r = canvas.getBoundingClientRect();
    const px = e.clientX - r.left;
    const x = geom.xmin + ((px - geom.ml) / (geom.W - geom.ml - geom.mr)) * (geom.xmax - geom.xmin);
    if (x < geom.xmin || x > geom.xmax) { tooltip().classList.add('hidden'); draw(); return; }
    const k0 = nearestIdx(spec.series[0].x, x);
    const xv = spec.series[0].x[k0];
    draw(xv);
    const tt = tooltip();
    tt.innerHTML = '';
    tt.append(h('div', { class: 'tt-row' }, h('span', {}, spec.xLabel || 'x'), h('b', {}, (spec.xFmt || ((v) => v.toFixed(1)))(xv))));
    spec.series.forEach((s, i) => {
      const k = nearestIdx(s.x, xv);
      if (k < 0) return;
      tt.append(h('div', { class: 'tt-row' }, h('span', {}, h('i', { style: { display: 'inline-block', width: '8px', height: '8px', background: colors[i], borderRadius: '2px', marginRight: '5px' } }), s.name), h('b', {}, (spec.yFmt || ((v) => v.toFixed(3)))(s.y[k]))));
    });
    tt.classList.remove('hidden');
    tt.style.left = `${Math.min(window.innerWidth - 200, e.clientX + 14)}px`;
    tt.style.top = `${e.clientY + 12}px`;
  });
  canvas.addEventListener('mouseleave', () => { tooltip().classList.add('hidden'); draw(); });
  requestAnimationFrame(() => draw());
  new ResizeObserver(() => draw()).observe(wrap);
  return wrap;
}

/** Horizontal bar chart: items [{label, value, color?, note}] */
export function barChart({ title, items, unit = '', fmt = (v) => v.toFixed(1), color }) {
  const wrap = h('div', { class: 'chart' });
  if (title) wrap.append(h('div', { class: 'ctitle' }, title));
  const max = Math.max(...items.map((i) => i.value), 1e-12);
  const list = h('div', { style: { display: 'flex', flexDirection: 'column', gap: '5px', marginTop: '4px' } });
  for (const it of items) {
    const w = Math.max(0.5, (it.value / max) * 100);
    const bar = h('div', { style: { height: '10px', width: `${w}%`, background: it.color || color || css('--series-1'), borderRadius: '0 4px 4px 0' } });
    const row = h('div', { style: { display: 'grid', gridTemplateColumns: '112px 1fr 64px', gap: '6px', alignItems: 'center', fontSize: '11.5px' } },
      h('span', { style: { color: css('--text-secondary'), overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' } }, it.label),
      h('div', { style: { background: 'transparent' } }, bar),
      h('span', { class: 'mono', style: { textAlign: 'right', color: css('--text-primary') } }, `${fmt(it.value)}${unit}`));
    row.addEventListener('mousemove', (e) => {
      const tt = tooltip();
      tt.innerHTML = '';
      tt.append(h('div', { class: 'tt-row' }, h('span', {}, it.label), h('b', {}, `${fmt(it.value)}${unit}`)));
      if (it.note) tt.append(h('div', { class: 'muted' }, it.note));
      tt.classList.remove('hidden');
      tt.style.left = `${e.clientX + 14}px`; tt.style.top = `${e.clientY + 12}px`;
    });
    row.addEventListener('mouseleave', () => tooltip().classList.add('hidden'));
    list.append(row);
  }
  wrap.append(list);
  return wrap;
}
