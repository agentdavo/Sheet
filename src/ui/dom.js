// Tiny DOM + form helpers (no framework).

export function h(tag, attrs = {}, ...children) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2).toLowerCase(), v);
    else if (k === 'html') el.innerHTML = v;
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children.flat(Infinity)) {
    if (c === null || c === undefined || c === false) continue;
    el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return el;
}

export const $ = (sel, root = document) => root.querySelector(sel);

export function getPath(obj, path) {
  return path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), obj);
}
export function setPath(obj, path, v) {
  const ks = path.split('.');
  let o = obj;
  for (const k of ks.slice(0, -1)) o = o[k];
  o[ks[ks.length - 1]] = v;
}

export function section(title, body, { collapsed = false, id } = {}) {
  const s = h('div', { class: `section${collapsed ? ' collapsed' : ''}`, id });
  const t = h('h3', { onclick: () => s.classList.toggle('collapsed') }, title);
  s.append(t, h('div', { class: 'sec-body' }, body));
  return s;
}

export const fmt = (v, d = 1) => (v === null || v === undefined || !Number.isFinite(v) ? '–' : Number(v).toFixed(d));
export const fmtK = (v) => (!Number.isFinite(v) ? '–' : Math.round(v).toLocaleString('en-GB'));

/** Numeric field bound to obj[path]. */
export function num(label, obj, path, { step = 1, min, max, unit = '', onChange, slider = false, digits, title } = {}) {
  const val = () => getPath(obj(), path);
  const input = h('input', { type: 'number', step, min, max, value: round(val(), digits ?? decimals(step)) });
  let range = null;
  const commit = (v, final = true) => {
    if (!Number.isFinite(v)) return;
    if (min !== undefined) v = Math.max(min, v);
    if (max !== undefined) v = Math.min(max, v);
    setPath(obj(), path, v);
    input.value = round(v, digits ?? decimals(step));
    if (range) range.value = v;
    onChange && onChange(v, final);
  };
  input.addEventListener('change', () => commit(parseFloat(input.value)));
  if (slider) {
    range = h('input', { type: 'range', step, min, max, value: val() });
    range.addEventListener('input', () => commit(parseFloat(range.value), false));
    range.addEventListener('change', () => commit(parseFloat(range.value), true));
    return h('div', { class: 'row slider', title },
      h('div', { class: 'top' }, h('label', {}, label), h('div', { class: 'ctl' }, input, h('span', { class: 'unit' }, unit))),
      range);
  }
  return h('div', { class: 'row', title }, h('label', {}, label), h('div', { class: 'ctl' }, input, h('span', { class: 'unit' }, unit)));
}

export function check(label, obj, path, { onChange, title } = {}) {
  const input = h('input', { type: 'checkbox' });
  input.checked = !!getPath(obj(), path);
  input.addEventListener('change', () => { setPath(obj(), path, input.checked); onChange && onChange(input.checked); });
  return h('label', { class: 'row check', title }, input, h('span', {}, label));
}

export function select(label, obj, path, options, { onChange, title } = {}) {
  const sel = h('select', {}, options.map(([v, t]) => h('option', { value: v }, t)));
  sel.value = getPath(obj(), path);
  sel.addEventListener('change', () => { setPath(obj(), path, sel.value); onChange && onChange(sel.value); });
  return h('div', { class: 'row', title }, h('label', {}, label), h('div', { class: 'ctl' }, sel));
}

export function kpi(k, v, unit = '', sub = '', cls = '') {
  return h('div', { class: `kpi ${cls}` }, h('div', { class: 'k' }, k), h('div', { class: 'v' }, v, unit ? h('small', {}, unit) : null), sub ? h('div', { class: 's' }, sub) : null);
}

export function statusDot(level) {
  return h('span', { class: `status-dot ${level}`, 'aria-hidden': 'true' });
}

function decimals(step) {
  const s = String(step);
  return s.includes('.') ? s.split('.')[1].length : 0;
}
function round(v, d) {
  return Number.isFinite(v) ? Number(v).toFixed(d) : '';
}

export function download(name, data, type = 'text/plain') {
  const blob = data instanceof Blob ? data : new Blob([data], { type });
  const a = h('a', { href: URL.createObjectURL(blob), download: name });
  document.body.append(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

export function safeStorage() {
  try {
    const k = '__t';
    localStorage.setItem(k, '1');
    localStorage.removeItem(k);
    return localStorage;
  } catch {
    return null;
  }
}
