/* =========================================================================
 * Lidar 3D view — shared by the overview (5001) and the 3D map (5005).
 *
 * Renders the 'lidar' feed (points in the rover frame: x forward, y left,
 * z up, metres, origin at the sensor) on a plain 2D canvas — no WebGL, so it
 * never competes with Unreal for the GPU.
 *
 *   - perspective camera that frames the scan: FIT / CHASE / TOP / SIDE views,
 *     drag = orbit, right-drag or shift-drag = pan, wheel = zoom,
 *     double-click = fit again
 *   - colour by height above the local ground or by range, scaled to the
 *     2nd..98th percentile so a few far or high points do not wash it out;
 *     legend with the scale
 *   - range rings with labels, the scan's field-of-view wedge, rover marker
 *     with body axes
 *
 * Usage: const v = LidarView(containerDiv, { info: elem? });
 *        socket.on('lidar', m => v.update(m));   v.redraw() after a resize.
 * ========================================================================= */
(function () {
  'use strict';

  const FOV = 50 * Math.PI / 180;           /* camera vertical field of view */
  const RING_STEPS = [2, 5, 10, 20, 50, 100, 200];
  /* Perceptual blue -> cyan -> green -> yellow -> red ramp (turbo-like). */
  const RAMP = [[48, 18, 59], [65, 105, 225], [40, 180, 230], [60, 220, 140],
                [190, 230, 60], [250, 180, 40], [235, 80, 40], [180, 20, 20]];
  /* Viewport colours come from the dashboard theme (theme.js). */
  const themeColor = (name, fallback) =>
    getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;

  const CSS = `
    .lv-root { position: absolute; inset: 0; }
    .lv-root canvas { position: absolute; inset: 0; width: 100%; height: 100%; display: block; cursor: grab; }
    .lv-bar { position: absolute; top: 6px; right: 6px; display: flex; gap: 4px; z-index: 3; flex-wrap: wrap;
              justify-content: flex-end; max-width: 70%; }
    .lv-bar button { font: 9px/1 Consolas, Menlo, monospace; letter-spacing: 1px; padding: 4px 7px; cursor: pointer;
                     color: var(--dim); background: color-mix(in srgb, var(--panel) 85%, transparent); border: 1px solid var(--line); border-radius: 3px; }
    .lv-bar button:hover { color: var(--cyan); border-color: var(--cyan); }
    .lv-bar button.on { color: var(--bg); background: var(--accent); border-color: var(--accent); font-weight: bold; }
    .lv-bar .sep { width: 6px; }
    .lv-legend { position: absolute; left: 8px; bottom: 8px; z-index: 3; pointer-events: none;
                 font: 10px/1.4 Consolas, Menlo, monospace; color: var(--dim); background: color-mix(in srgb, var(--panel) 85%, transparent);
                 border: 1px solid var(--line); border-radius: 4px; padding: 5px 7px; }
    .lv-legend .bar { width: 120px; height: 7px; border-radius: 2px; margin: 3px 0 2px; }
    .lv-legend .ends { display: flex; justify-content: space-between; color: var(--text); }
    .lv-compact .lv-legend { font-size: 9px; padding: 3px 5px; }
    .lv-compact .lv-legend .bar { width: 70px; height: 5px; }
    .lv-compact .lv-bar { max-width: 98%; flex-wrap: nowrap; gap: 3px; }
    .lv-compact .lv-bar button { padding: 3px 4px; font-size: 8px; letter-spacing: .5px; }
    .lv-compact .lv-bar .sep { width: 2px; }`;
  let styled = false;

  function rampColor(t) {
    t = Math.max(0, Math.min(1, t)) * (RAMP.length - 1);
    const i = Math.min(RAMP.length - 2, Math.floor(t)), u = t - i;
    return RAMP[i].map((c, k) => Math.round(c + u * (RAMP[i + 1][k] - c)));
  }
  const PALETTE = Array.from({ length: 64 }, (_, i) => `rgb(${rampColor(i / 63).join(',')})`);
  const RAMP_CSS = 'linear-gradient(90deg,' + RAMP.map(c => `rgb(${c.join(',')})`).join(',') + ')';

  function percentile(sorted, p) {
    if (!sorted.length) return 0;
    return sorted[Math.min(sorted.length - 1, Math.max(0, Math.round(p * (sorted.length - 1))))];
  }

  window.LidarView = function (container, opts) {
    opts = opts || {};
    if (!styled) {
      const s = document.createElement('style'); s.textContent = CSS; document.head.appendChild(s); styled = true;
    }
    const root = document.createElement('div');
    root.className = 'lv-root';
    root.innerHTML =
      '<canvas></canvas>' +
      '<div class="lv-bar">' +
      '<button data-view="fit" title="Frame the whole scan (double-click the view too)">FIT</button>' +
      '<button data-view="chase" title="From behind the rover">CHASE</button>' +
      '<button data-view="top" title="From above">TOP</button>' +
      '<button data-view="side" title="From the right side (forward points right)">SIDE</button>' +
      '<span class="sep"></span>' +
      '<button data-color="height" title="Colour by height above the local ground">HEIGHT</button>' +
      '<button data-color="range" title="Colour by distance from the sensor">RANGE</button>' +
      '</div>' +
      '<div class="lv-legend" style="display:none"><div class="t"></div><div class="bar"></div>' +
      '<div class="ends"><span class="lo"></span><span class="hi"></span></div></div>';
    container.appendChild(root);
    const canvas = root.querySelector('canvas'), ctx = canvas.getContext('2d');
    const legend = root.querySelector('.lv-legend');
    legend.querySelector('.bar').style.background = RAMP_CSS;

    let pts = null;                 /* Float32Array xyz */
    let stats = null;               /* robust extents of the current scan */
    let colorMode = 'height';
    try { colorMode = localStorage.getItem('lidar_color') || 'height'; } catch (e) {}
    /* Camera: orbit around `target`; yaw = heading of the view direction, pitch = look-down angle. */
    const cam = { tx: 10, ty: 0, tz: 0, yaw: 0, pitch: 0.45, dist: 40, ox: 0, oy: 0 };   /* ox/oy: screen offset (px) */
    let preset = 'chase', auto = true;     /* auto: refit on each new scan until the user moves the view */

    function setButtons() {
      root.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('on', auto && b.dataset.view === preset));
      root.querySelectorAll('[data-color]').forEach(b => b.classList.toggle('on', b.dataset.color === colorMode));
    }

    /* ---- scan statistics: robust bounds, ground level, azimuth span ---- */
    function analyse(p) {
      const n = p.length / 3, step = Math.max(1, Math.floor(n / 4000));
      const xs = [], ys = [], zs = [], rs = [], az = [], nearZ = [], sample = [];
      for (let i = 0; i < n; i += step) {
        const x = p[3 * i], y = p[3 * i + 1], z = p[3 * i + 2];
        if (sample.length < 3 * 1500 && (i / step) % 3 === 0) sample.push(x, y, z);
        const r = Math.hypot(x, y, z);
        xs.push(x); ys.push(y); zs.push(z); rs.push(r); az.push(Math.atan2(y, x));
        if (Math.hypot(x, y) < 15) nearZ.push(z);
      }
      [xs, ys, zs, rs, az, nearZ].forEach(a => a.sort((u, v) => u - v));
      const ground = nearZ.length > 20 ? percentile(nearZ, 0.2) : percentile(zs, 0.05);
      return {
        x0: percentile(xs, 0.02), x1: percentile(xs, 0.98), y0: percentile(ys, 0.02), y1: percentile(ys, 0.98),
        z0: percentile(zs, 0.02), z1: percentile(zs, 0.98), r0: percentile(rs, 0.02), r1: percentile(rs, 0.98),
        az0: percentile(az, 0.01), az1: percentile(az, 0.99), rmax: percentile(rs, 0.995), ground,
        h0: 0, h1: Math.max(0.5, percentile(zs, 0.98) - ground),
        sample: sample.concat([0, 0, ground]),           /* scan points (subset) + the rover */
      };
    }

    /* ---- camera ---- */
    function basis() {
      const cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
      const f = [cp * cy, cp * sy, -sp];                       /* view direction */
      const r = [sy, -cy, 0];                                  /* screen right */
      const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
      const c = [cam.tx - cam.dist * f[0], cam.ty - cam.dist * f[1], cam.tz - cam.dist * f[2]];
      return { f, r, u, c };
    }

    function fit(name) {
      if (name) preset = name;
      const s = stats;
      if (preset === 'chase') { cam.yaw = 0; cam.pitch = 0.42; }
      else if (preset === 'top') { cam.yaw = 0; cam.pitch = 1.45; }
      else if (preset === 'side') { cam.yaw = Math.PI / 2; cam.pitch = 0.3; }    /* from the right: forward is right */
      cam.ox = cam.oy = 0;
      if (!s) { cam.tx = 10; cam.ty = 0; cam.tz = 0; cam.dist = 40; return; }
      /* Look at the middle of the scan + rover box. Then find (bisection) the
         closest distance at which the projected 1..99 % extent of the points fits
         the view, and centre that extent with a screen offset. */
      cam.tx = (Math.min(s.x0, -1) + Math.max(s.x1, 1)) / 2;
      cam.ty = (Math.min(s.y0, -1) + Math.max(s.y1, 1)) / 2;
      cam.tz = (Math.min(s.z0, s.ground) + Math.max(s.z1, 0.5)) / 2;
      const W = canvas.clientWidth || 400, H = canvas.clientHeight || 300;
      const F = (H / 2) / Math.tan(FOV / 2), P = s.sample, m = P.length / 3;
      const extent = () => {                   /* [x0, x1, y0, y1] in px from the centre, or null */
        const { f, r, u, c } = basis(), sx = [], sy = [];
        for (let i = 0; i < m; i++) {
          const dx = P[3 * i] - c[0], dy = P[3 * i + 1] - c[1], dz = P[3 * i + 2] - c[2];
          const d = dx * f[0] + dy * f[1] + dz * f[2];
          if (d < 0.5) return null;            /* a point behind the camera: too close */
          sx.push((dx * r[0] + dy * r[1] + dz * r[2]) * F / d);
          sy.push(-(dx * u[0] + dy * u[1] + dz * u[2]) * F / d);
        }
        sx.sort((a, b) => a - b); sy.sort((a, b) => a - b);
        return [percentile(sx, 0.01), percentile(sx, 0.99), percentile(sy, 0.01), percentile(sy, 0.99)];
      };
      const BAR = 34;                          /* keep the button strip at the top clear */
      const fits = e => e && e[1] - e[0] <= W * 0.92 && e[3] - e[2] <= (H - BAR) * 0.9;
      let lo = 1, hi = 3000;
      for (let it = 0; it < 32; it++) {
        cam.dist = Math.sqrt(lo * hi);
        if (fits(extent())) hi = cam.dist; else lo = cam.dist;
      }
      cam.dist = hi;
      const e = extent();
      if (e) { cam.ox = -(e[0] + e[1]) / 2; cam.oy = -(e[2] + e[3]) / 2 + BAR / 2; }
    }

    /* ---- drawing ---- */
    function draw() {
      const W = canvas.clientWidth, H = canvas.clientHeight;
      if (!W || !H) return;
      const dpr = window.devicePixelRatio || 1;
      if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
        canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      root.classList.toggle('lv-compact', H < 300 || W < 520);
      const C = { bg: themeColor('--view', '#10151c'), ring: themeColor('--view-line', '#1b2a3c'),
                  label: themeColor('--view-text', '#4d6580'), rover: themeColor('--text', '#e6edf5') };
      ctx.fillStyle = C.bg; ctx.fillRect(0, 0, W, H);

      const { f, r, u, c } = basis();
      const F = (H / 2) / Math.tan(FOV / 2);
      const proj = (x, y, z) => {
        const dx = x - c[0], dy = y - c[1], dz = z - c[2];
        const d = dx * f[0] + dy * f[1] + dz * f[2];
        if (d < 0.3) return null;
        return [W / 2 + cam.ox + (dx * r[0] + dy * r[1] + dz * r[2]) * F / d,
                H / 2 + cam.oy - (dx * u[0] + dy * u[1] + dz * u[2]) * F / d, d];
      };
      const line = (a, b, color, width) => {
        const p = proj(...a), q = proj(...b);
        if (!p || !q) return;
        ctx.strokeStyle = color; ctx.lineWidth = width || 1;
        ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
      };
      const text = (x, y, z, s, color) => {
        const p = proj(x, y, z);
        if (!p) return;
        ctx.font = '10px Consolas, Menlo, monospace'; ctx.fillStyle = color; ctx.textAlign = 'center';
        ctx.fillText(s, p[0], p[1]);
      };

      const g = stats ? stats.ground : -0.55;
      /* Range rings on the ground, labelled; the field-of-view wedge of the scan. */
      const rmax = stats ? stats.rmax : 40;
      const step = RING_STEPS.find(s => rmax / s <= 5) || 200;
      for (let R = step; R <= rmax * 1.05 + 1e-6; R += step) {
        ctx.strokeStyle = C.ring; ctx.lineWidth = 1; ctx.beginPath();
        let started = false;
        for (let k = 0; k <= 96; k++) {
          const a = (k / 96) * 2 * Math.PI, p = proj(R * Math.cos(a), R * Math.sin(a), g);
          if (!p) { started = false; continue; }
          if (!started) { ctx.moveTo(p[0], p[1]); started = true; } else ctx.lineTo(p[0], p[1]);
        }
        ctx.stroke();
        text(R * Math.cos(-0.35), R * Math.sin(-0.35), g, `${R} m`, C.label);
      }
      if (stats && stats.az1 - stats.az0 < 1.9 * Math.PI) {
        for (const a of [stats.az0, stats.az1]) line([0, 0, g], [rmax * Math.cos(a), rmax * Math.sin(a), g], C.ring, 1);
      }

      if (pts && stats) {
        /* Project, then draw far to near so near points stay on top. */
        const n = pts.length / 3, out = [];
        const lo = colorMode === 'range' ? stats.r0 : stats.h0;
        const hi = colorMode === 'range' ? stats.r1 : stats.h1;
        for (let i = 0; i < n; i++) {
          const x = pts[3 * i], y = pts[3 * i + 1], z = pts[3 * i + 2];
          const p = proj(x, y, z);
          if (!p || p[0] < -2 || p[0] > W + 2 || p[1] < -2 || p[1] > H + 2) continue;
          const v = colorMode === 'range' ? Math.hypot(x, y, z) : z - stats.ground;
          out.push(p[0], p[1], p[2], Math.round(63 * Math.max(0, Math.min(1, (v - lo) / Math.max(1e-3, hi - lo)))));
        }
        const order = Array.from({ length: out.length / 4 }, (_, k) => k).sort((a, b) => out[4 * b + 2] - out[4 * a + 2]);
        const near = cam.dist * 0.6;
        for (const k of order) {
          const d = out[4 * k + 2], s = Math.max(1.4, Math.min(3.2, 3.2 * near / d));
          ctx.fillStyle = PALETTE[out[4 * k + 3]];
          ctx.fillRect(out[4 * k] - s / 2, out[4 * k + 1] - s / 2, s, s);
        }
        legend.style.display = '';
        legend.querySelector('.t').textContent = colorMode === 'range' ? 'range from sensor' : 'height above ground';
        legend.querySelector('.lo').textContent = `${lo.toFixed(1)} m`;
        legend.querySelector('.hi').textContent = `${hi.toFixed(1)} m`;
      }
      /* Rover marker (footprint triangle) and body axes, over the points. */
      line([0.6, 0, g], [-0.3, 0.3, g], C.rover, 1.5); line([-0.3, 0.3, g], [-0.3, -0.3, g], C.rover, 1.5);
      line([-0.3, -0.3, g], [0.6, 0, g], C.rover, 1.5);
      const A = Math.max(1, rmax / 15);
      line([0, 0, 0], [A, 0, 0], '#33d6a6', 2); line([0, 0, 0], [0, A, 0], '#59b0ff', 2); line([0, 0, 0], [0, 0, A * 0.7], '#ffbe5c', 2);
      text(A * 1.15, 0, 0, 'x', '#33d6a6'); text(0, A * 1.15, 0, 'y', '#59b0ff'); text(0, 0, A * 0.8, 'z', '#ffbe5c');
    }

    /* ---- input ---- */
    let drag = null;
    canvas.addEventListener('contextmenu', e => e.preventDefault());
    canvas.addEventListener('mousedown', e => {
      drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey };
      canvas.style.cursor = 'grabbing';
    });
    window.addEventListener('mouseup', () => { drag = null; canvas.style.cursor = 'grab'; });
    window.addEventListener('mousemove', e => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX; drag.y = e.clientY;
      if (drag.pan) {
        const { r, f } = basis(), k = cam.dist / ((canvas.clientHeight || 300) / 2 / Math.tan(FOV / 2));
        const fx = f[0], fy = f[1], fn = Math.hypot(fx, fy) || 1;   /* pan in the ground plane */
        cam.tx += (-dx * r[0] + dy * fx / fn) * k; cam.ty += (-dx * r[1] + dy * fy / fn) * k;
      } else {
        cam.yaw -= dx * 0.008;
        cam.pitch = Math.max(0.02, Math.min(1.55, cam.pitch + dy * 0.008));
      }
      auto = false; setButtons(); draw();
    });
    canvas.addEventListener('wheel', e => {
      e.preventDefault();
      cam.dist = Math.max(1.5, Math.min(800, cam.dist * (e.deltaY > 0 ? 1.12 : 0.89)));
      auto = false; setButtons(); draw();
    }, { passive: false });
    canvas.addEventListener('dblclick', () => { auto = true; fit(); setButtons(); draw(); });
    root.querySelectorAll('[data-view]').forEach(b => b.onclick = () => {
      auto = true; fit(b.dataset.view === 'fit' ? null : b.dataset.view); setButtons(); draw();
    });
    root.querySelectorAll('[data-color]').forEach(b => b.onclick = () => {
      colorMode = b.dataset.color;
      try { localStorage.setItem('lidar_color', colorMode); } catch (e) {}
      setButtons(); draw();
    });

    function update(m) {
      pts = Float32Array.from(m.points || []);
      stats = pts.length >= 30 ? analyse(pts) : null;
      if (auto) fit();
      if (opts.info) {
        opts.info.textContent = stats
          ? `${m.count} pts · ${stats.r0.toFixed(1)}–${stats.r1.toFixed(1)} m · ${m.frame}`
          : `${m.count} pts · ${m.frame}`;
      }
      if (!document.hidden) draw();
    }

    /* Scans arriving while the page is hidden are not drawn: catch up on return. */
    document.addEventListener('visibilitychange', () => { if (!document.hidden) { if (auto) fit(); draw(); } });
    window.addEventListener('gnc-theme', () => { if (!document.hidden) draw(); });

    fit(); setButtons(); draw();
    return { update, redraw: () => { if (auto) fit(); draw(); }, fit: () => { auto = true; fit(); setButtons(); draw(); } };
  };
})();
