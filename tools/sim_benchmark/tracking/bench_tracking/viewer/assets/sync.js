// Everything that has to feel instant runs here, in the browser, with no server round trip.
//
//  * One playhead (time t) for the page, as in Foxglove: hovering any time chart, dragging the playback
//    bar, pressing play or clicking the map moves it. Every `.tsync` chart draws a line at t, and its
//    header shows each run's value at t. The map moves each run's marker; the "moment" panel lists the
//    events around t.
//  * Linked zoom: time charts share their x range; the two charts of a side-by-side pair also share y.
//    Zooming a time chart lights up that stretch of the route on the map.
//  * The run browser: open/close, filter, Esc.
(function () {
  const S = { zoom: false, t: null, playing: false, speed: 1, last: 0, ver: null, poses: {}, events: [], frame: false, busy: false };

  // ---------------------------------------------------------------- helpers
  const $$ = (sel, root) => [...(root || document).querySelectorAll(sel)];
  const tsyncGds = () => $$('.tsync .js-plotly-plot');
  const mapGd = () => document.querySelector('#mapbox .js-plotly-plot');
  function mapIdx() {
    const el = document.querySelector('#mapbox .map-inner');
    try { return el ? JSON.parse(el.dataset.idx) : null; } catch (e) { return null; }
  }
  function loadData() {
    const d = document.getElementById('phdata');
    if (!d) { S.poses = {}; S.events = []; S.ver = null; return; }
    if (d.dataset.ver === S.ver) return;
    S.ver = d.dataset.ver;
    try { S.poses = JSON.parse(d.dataset.poses || '{}'); } catch (e) { S.poses = {}; }
    try { S.events = JSON.parse(d.dataset.events || '[]'); } catch (e) { S.events = []; }
  }
  function at(xs, t) {            // index of the last sample at or before t (-1 if none)
    if (!xs || !xs.length || t < xs[0]) return -1;
    let lo = 0, hi = xs.length - 1;
    while (lo < hi) { const m = (lo + hi + 1) >> 1; if (xs[m] <= t) lo = m; else hi = m - 1; }
    return lo;
  }
  function fmt(v) {
    if (v === null || v === undefined || !isFinite(v)) return '–';
    const a = Math.abs(v);
    return a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
  }
  const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  // ---------------------------------------------------------------- drawing the playhead
  function drawLine(gd, t) {
    const fl = gd._fullLayout;
    if (!fl || !fl.xaxis || !fl._size) return;
    const host = gd.parentElement;
    let line = host.querySelector(':scope > .ph-line');
    if (!line) { line = document.createElement('div'); line.className = 'ph-line'; host.appendChild(line); }
    const xa = fl.xaxis, px = xa._offset + xa.l2p(t);
    if (t === null || !isFinite(px) || px < xa._offset - 1 || px > xa._offset + xa._length + 1) {
      line.style.display = 'none'; return;
    }
    line.style.display = 'block';
    line.style.left = px + 'px';
    line.style.top = fl._size.t + 'px';
    line.style.height = fl._size.h + 'px';
  }
  function readout(gd, t) {
    const box = (gd.closest('.pair-cell') || gd.closest('.card') || document).querySelector('.readout');
    if (!box) return;
    if (t === null) { box.innerHTML = ''; return; }
    const parts = [];
    for (const tr of gd._fullData || []) {
      if (tr.type !== 'scatter' || !tr.visible || !String(tr.mode || '').startsWith('lines') || !tr.name) continue;
      const xs = tr.x, ys = tr.y;
      if (!xs || !ys || typeof ys[0] === 'string') continue;
      const i = at(xs, t);
      const v = i < 0 ? null : ys[i];
      const c = (tr.line && tr.line.color) || '#999';
      parts.push({ c, name: tr.name, v });
    }
    // one run on screen: the header only needs the number; several: name each one
    box.innerHTML = parts.map(p => parts.length === 1
      ? `<span class="ro solo"><b>${fmt(p.v)}</b></span>`
      : `<span class="ro"><i style="background:${p.c}"></i>${esc(p.name)} <b>${fmt(p.v)}</b></span>`).join('');
  }
  function moveMap(t) {
    const gd = mapGd(), idx = mapIdx();
    if (!gd || !idx || !gd._fullData) return;
    const ids = [], xs = [], ys = [];
    for (const rid of Object.keys(idx.cursor_odom || {})) {
      const p = S.poses[rid];
      const i = p && t !== null ? at(p.t, t) : -1;
      const ok = i >= 0 && t <= p.t[p.t.length - 1] + 1;
      ids.push(idx.cursor_odom[rid]); xs.push(ok ? [p.ox[i]] : []); ys.push(ok ? [p.oy[i]] : []);
      ids.push(idx.cursor_slam[rid]); xs.push(ok ? [p.sx[i]] : []); ys.push(ok ? [p.sy[i]] : []);
    }
    if (!S.zoom) {                       // no zoomed stretch: the window traces show an 8 s trail instead
      for (const rid of Object.keys(idx.window || {})) {
        const p = S.poses[rid], x = [], y = [];
        if (p && t !== null) {
          const i1 = at(p.t, t), i0 = at(p.t, t - 8);
          for (let i = Math.max(i0, 0); i1 >= 0 && i <= i1; i++) { x.push(p.ox[i]); y.push(p.oy[i]); }
        }
        ids.push(idx.window[rid]); xs.push(x); ys.push(y);
      }
    }
    if (ids.length) window.Plotly.restyle(gd, { x: xs, y: ys }, ids);
  }
  function moment(t) {
    const box = document.getElementById('phlog');
    if (!box || t === null) return;
    const near = S.events.filter(e => Math.abs(e.t - t) <= 2);
    const rows = near.map(e => `<div class="ev" style="border-left-color:${e.c}"><b>${e.t.toFixed(1)} s</b> ` +
      `<span class="ek">${esc(e.kind)}</span> <span class="dim">${esc(e.run)}</span><div>${esc(e.detail)}</div></div>`);
    box.innerHTML = `<div class="t">t = ${t.toFixed(1)} s</div>` +
      `<div class="dim small">Events within ±2 s</div>` +
      (rows.length ? `<div class="evlist">${rows.join('')}</div>` : `<div class="dim">none</div>`);
  }
  function render() {
    S.frame = false;
    const t = S.t;
    loadData();
    for (const gd of tsyncGds()) { drawLine(gd, t); readout(gd, t); }
    moveMap(t);
    moment(t);
    const r = document.getElementById('ph-range'), lab = document.getElementById('ph-time');
    if (r && t !== null && document.activeElement !== r) r.value = t;
    if (lab && t !== null) lab.textContent = `t = ${t.toFixed(1)} s`;
  }
  function setT(t) {
    if (t === null || t === undefined || !isFinite(t)) return;
    S.t = +t;
    if (!S.frame) { S.frame = true; requestAnimationFrame(render); }
  }

  // ---------------------------------------------------------------- zoom: share x across time charts, y within a pair
  function xRange(ev) {
    let lo = null, hi = null, auto = false;
    for (const [k, v] of Object.entries(ev)) {
      const m = k.match(/^xaxis\d*\.(range\[(0|1)\]|range|autorange)$/);
      if (!m) continue;
      if (m[1] === 'autorange') auto = true;
      else if (m[1] === 'range') { lo = v[0]; hi = v[1]; }
      else if (m[2] === '0') lo = v; else hi = v;
    }
    return auto ? 'auto' : (lo !== null && hi !== null ? [lo, hi] : null);
  }
  function yUpdates(ev) {
    const out = {};
    for (const [k, v] of Object.entries(ev)) if (/^yaxis\d*\.(range(\[[01]\])?|autorange)$/.test(k)) out[k] = v;
    return out;
  }
  function highlight(range) {
    const gd = mapGd(), idx = mapIdx();
    S.zoom = !!(range && range !== 'auto');
    if (!gd || !idx) return;
    const ids = [], xs = [], ys = [];
    for (const rid of Object.keys(idx.window || {})) {
      const p = S.poses[rid];
      let x = [], y = [];
      if (p && range && range !== 'auto') {
        for (let i = 0; i < p.t.length; i++) if (p.t[i] >= range[0] && p.t[i] <= range[1]) { x.push(p.ox[i]); y.push(p.oy[i]); }
      }
      ids.push(idx.window[rid]); xs.push(x); ys.push(y);
    }
    if (ids.length) window.Plotly.restyle(gd, { x: xs, y: ys }, ids);
    if ((idx.base || []).length) window.Plotly.restyle(gd, { opacity: range && range !== 'auto' ? 0.3 : 1 }, idx.base);
  }
  function onRelayout(gd, ev) {
    if (S.busy || !ev) return;
    const jobs = [];
    const xr = gd.closest('.tsync') ? xRange(ev) : null;
    if (xr) {
      for (const o of tsyncGds()) {
        if (o === gd) continue;
        const upd = {};
        for (const ax of Object.keys(o.layout || {}).filter(a => /^xaxis\d*$/.test(a))) {
          if (xr === 'auto') upd[ax + '.autorange'] = true; else upd[ax + '.range'] = xr;
        }
        if (Object.keys(upd).length) jobs.push(window.Plotly.relayout(o, upd));
      }
      loadData();
      highlight(xr);
    }
    const pair = gd.closest('.pair');
    if (pair) {
      const yu = yUpdates(ev), xu = {};
      if (!gd.closest('.tsync')) { const r = xRange(ev); if (r === 'auto') xu['xaxis.autorange'] = true; else if (r) xu['xaxis.range'] = r; }
      for (const o of $$('.js-plotly-plot', pair)) if (o !== gd && Object.keys({ ...yu, ...xu }).length) jobs.push(window.Plotly.relayout(o, { ...yu, ...xu }));
    }
    if (!jobs.length) return;
    S.busy = true;
    Promise.allSettled(jobs).then(() => { S.busy = false; if (S.t !== null) setT(S.t); });
  }

  // ---------------------------------------------------------------- wiring
  function attach(gd) {
    if (gd.__bench || typeof gd.on !== 'function') return;
    gd.__bench = true;
    const isMap = !!gd.closest('#mapbox');
    if (gd.closest('.tsync')) {
      gd.on('plotly_hover', ev => { if (!S.playing && ev && ev.points && ev.points.length) setT(ev.points[0].x); });
    }
    if (isMap) {
      const tOf = ev => { const cd = ev && ev.points && ev.points[0] && ev.points[0].customdata;
                          return typeof cd === 'number' ? cd : Array.isArray(cd) ? +cd[0] : null; };
      gd.on('plotly_hover', ev => { const t = tOf(ev); if (t !== null && !S.playing) setT(t); });
      gd.on('plotly_click', ev => {
        const t = tOf(ev); if (t === null) return;
        setT(t);
        for (const o of tsyncGds()) {
          const upd = {};
          for (const ax of Object.keys(o.layout || {}).filter(a => /^xaxis\d*$/.test(a))) upd[ax + '.range'] = [t - 15, t + 15];
          window.Plotly.relayout(o, upd);
        }
        loadData(); highlight([t - 15, t + 15]);
      });
    }
    gd.on('plotly_relayout', ev => onRelayout(gd, ev));
    gd.on('plotly_afterplot', () => { if (S.t !== null && gd.closest('.tsync')) { drawLine(gd, S.t); } });
  }
  setInterval(() => {
    $$('.tsync .js-plotly-plot, .pair .js-plotly-plot, #mapbox .js-plotly-plot').forEach(attach);
    const p = document.getElementById('player');
    if (!p) { S.t = null; S.playing = false; return; }        // left the page: forget the playhead
    if (S.t === null && tsyncGds().length) setT(+p.dataset.t0); // start at the beginning so the readouts are filled
  }, 400);

  // playback
  function tick(now) {
    if (!S.playing) return;
    const p = document.getElementById('player');
    if (!p) { S.playing = false; return; }
    const dt = S.last ? (now - S.last) / 1000 : 0;
    S.last = now;
    let t = (S.t === null ? +p.dataset.t0 : S.t) + dt * S.speed;
    if (t >= +p.dataset.t1) { t = +p.dataset.t1; S.playing = false; syncPlayBtn(); }
    setT(t);
    if (S.playing) requestAnimationFrame(tick);
  }
  function syncPlayBtn() { const b = document.getElementById('ph-play'); if (b) b.textContent = S.playing ? '❚❚' : '▶'; }
  function togglePlay() {
    const p = document.getElementById('player');
    if (!p) return;
    S.playing = !S.playing;
    if (S.playing && (S.t === null || S.t >= +p.dataset.t1 - 0.05)) S.t = +p.dataset.t0;
    S.last = 0; syncPlayBtn();
    if (S.playing) requestAnimationFrame(tick);
  }

  document.addEventListener('click', e => {
    const el = e.target.closest('button, .js-browse, .js-close');
    if (!el) {
      const wrap = document.getElementById('browser-wrap');
      if (wrap && wrap.classList.contains('open') && !e.target.closest('#browser-wrap')) wrap.classList.remove('open');
      return;
    }
    if (el.id === 'ph-play') { togglePlay(); return; }
    if (el.classList.contains('ph-speed')) {
      S.speed = +el.dataset.speed || 1;
      $$('.ph-speed').forEach(b => b.classList.toggle('on', b === el));
      return;
    }
    const wrap = document.getElementById('browser-wrap');
    if (el.classList.contains('js-browse') && wrap) {
      wrap.classList.toggle('open');
      if (wrap.classList.contains('open')) setTimeout(() => { const f = document.getElementById('run-filter'); if (f) f.focus(); }, 50);
      e.stopPropagation();
      return;
    }
    if (el.classList.contains('js-close') && wrap) wrap.classList.remove('open');
  });
  document.addEventListener('input', e => {
    if (e.target.id === 'ph-range') { S.playing = false; syncPlayBtn(); setT(+e.target.value); }
    if (e.target.id === 'run-filter') {
      const q = e.target.value.trim().toLowerCase().split(/\s+/).filter(Boolean);
      $$('.run-row').forEach(r => { r.style.display = q.every(w => (r.dataset.search || '').includes(w)) ? '' : 'none'; });
    }
  });
  document.addEventListener('keydown', e => {
    if (e.key === 'Escape') { const w = document.getElementById('browser-wrap'); if (w) w.classList.remove('open'); }
    const tag = (e.target.tagName || '').toLowerCase();
    if (tag === 'input' && e.target.type !== 'range' || tag === 'textarea' || e.target.isContentEditable) return;
    if (!document.getElementById('player')) return;
    if (e.key === ' ') { e.preventDefault(); togglePlay(); }
    else if (e.key === 'ArrowRight' || e.key === 'ArrowLeft') {
      e.preventDefault(); S.playing = false; syncPlayBtn();
      setT((S.t || 0) + (e.key === 'ArrowRight' ? 1 : -1) * (e.shiftKey ? 10 : 1));
    }
  });
})();
