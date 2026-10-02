/* =========================================================================
 * Local DEM view — shared by the overview (5001) and the 3D map (5005).
 *
 * Renders the 'local_dem' feed (rover-aligned elevation grid from
 * component_local_mapping, metres above the ground under the rover):
 *   - true-height geometry, coloured by relief above the fitted local ground
 *     on a fixed scale; obstacles in red
 *   - unobserved cells left empty over a reference ground grid
 *   - rover footprint, range arcs and metric rulers; chase and top views
 *
 * Frame: x = rover right, y = rover forward (0 = camera plane), z = up.
 * Requires three.js (r128) loaded before this file.
 * ========================================================================= */
(function () {
  'use strict';

  const V_EXAG = 1.5;                       /* vertical exaggeration */
  const H_MIN = -0.30, H_MAX = 0.50;        /* colour scale limits (m) */
  const UNKNOWN = null;

  /* Height colour ramp (m). The obstacle threshold stop is inserted at runtime. */
  function rampStops(thr) {
    return [
      [H_MIN, [ 59, 111, 216]],             /* depression  */
      [-0.10, [ 95, 163, 199]],
      [ 0.00, [143, 163, 168]],             /* ground      */
      [thr * 0.5, [201, 180, 88]],
      [thr, [255, 138, 61]],                /* obstacle    */
      [H_MAX, [255, 71, 87]],
    ];
  }

  function rampColor(h, stops) {
    if (h <= stops[0][0]) return stops[0][1];
    for (let i = 0; i < stops.length - 1; i++) {
      const [h0, c0] = stops[i], [h1, c1] = stops[i + 1];
      if (h <= h1) {
        const u = (h - h0) / Math.max(1e-6, h1 - h0);
        return [0, 1, 2].map(k => c0[k] + u * (c1[k] - c0[k]));
      }
    }
    return stops[stops.length - 1][1];
  }

  /* High-resolution text sprite, `size` metres tall. anchorX: 0 = left-aligned,
     0.5 = centred, 1 = right-aligned. */
  function label(text, color, size, anchorX) {
    const PX = 96;                           /* font size in canvas pixels */
    const c = document.createElement('canvas');
    const g = c.getContext('2d');
    const font = `bold ${PX}px Consolas, Menlo, monospace`;
    g.font = font;
    c.width = Math.ceil(g.measureText(text).width + PX * 0.5);
    c.height = Math.ceil(PX * 1.4);
    g.font = font;                           /* resizing the canvas resets the context */
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.lineJoin = 'round';
    g.lineWidth = PX * 0.18; g.strokeStyle = 'rgba(16,21,28,0.92)';   /* outline: readable on light terrain */
    g.strokeText(text, c.width / 2, c.height / 2);
    g.fillStyle = color;
    g.fillText(text, c.width / 2, c.height / 2);
    const tex = new THREE.CanvasTexture(c);
    tex.minFilter = THREE.LinearFilter;
    tex.generateMipmaps = false;
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
    s.scale.set(size * c.width / c.height, size, 1);
    s.center.set(anchorX === undefined ? 0.5 : anchorX, 0.5);
    s.renderOrder = 10;
    return s;
  }

  function lineSegments(points, color, opacity) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(points), 3));
    return new THREE.LineSegments(g, new THREE.LineBasicMaterial(
      { color, transparent: opacity < 1, opacity }));
  }

  const STYLE = `
    .dem-overlay { position:absolute; z-index:5; font:10px/1.5 Consolas,Menlo,monospace;
      color:#8aa0b8; background:rgba(16,21,28,.78); border:1px solid #2c3a4e;
      border-radius:6px; padding:6px 8px; letter-spacing:.5px; }
    .dem-legend { left:8px; top:40px; pointer-events:none; }
    .dem-legend .bar { width:150px; height:9px; border-radius:2px; margin:3px 0 2px; }
    .dem-legend .ticks { display:flex; justify-content:space-between; width:150px; color:#cfe0ee; }
    .dem-views { right:8px; top:8px; padding:3px; display:flex; gap:3px; }
    .dem-views button { font:10px Consolas,monospace; letter-spacing:1px; color:#8aa0b8;
      background:#202b3a; border:1px solid #2c3a4e; border-radius:4px; padding:3px 7px; cursor:pointer; }
    .dem-views button.on { color:#33d6a6; border-color:#33d6a6; }
    .dem-stats { left:8px; top:8px; pointer-events:none; color:#cfe0ee; }
    .dem-stats b { color:#ff6b7d; font-weight:normal; }
    .dem-compact .dem-legend { display:none; }
    .dem-compact .dem-overlay { padding:2px 5px; font-size:9px; }
    .dem-compact .dem-views button { padding:1px 5px; font-size:9px; }`;

  /* Below this panel size the legend is hidden and the stats shortened. */
  const COMPACT_W = 520, COMPACT_H = 320;

  window.LocalDemView = function (el, opts) {
    opts = opts || {};
    if (!document.getElementById('local-dem-style')) {
      const st = document.createElement('style');
      st.id = 'local-dem-style'; st.textContent = STYLE;
      document.head.appendChild(st);
    }

    let scene = null, cam = null, renderer = null, mesh = null, ground = null, heightAxis = null;
    let threshold = 0.15, stops = rampStops(threshold);
    let layout = null;                      /* {w, h, res} of the current mesh */
    const views = {
      /* half-extents (m) of what each view must show, used to fit the panel */
      chase: { yaw: -Math.PI / 2, pitch: 0.55, zoom: 1, half: [3.9, 2.0] },
      top:   { yaw: -Math.PI / 2, pitch: 1.53, zoom: 1, half: [3.9, 2.25] },
    };
    const orbit = Object.assign({}, views.chase);
    let target = new THREE.Vector3(0, 1.4, 0);

    /* ---- overlays ---- */
    const legend = document.createElement('div');
    legend.className = 'dem-overlay dem-legend';
    const stats = document.createElement('div');
    stats.className = 'dem-overlay dem-stats';
    stats.style.display = 'none';
    const viewBox = document.createElement('div');
    viewBox.className = 'dem-overlay dem-views';
    const buttons = {};
    for (const name of ['chase', 'top']) {
      const b = document.createElement('button');
      b.textContent = name.toUpperCase();
      b.onclick = () => { Object.assign(orbit, views[name]); setActive(name); draw(); };
      buttons[name] = b; viewBox.appendChild(b);
    }
    function setActive(name) {
      for (const k in buttons) buttons[k].classList.toggle('on', k === name);
    }
    setActive('chase');

    /* lidar overlay toggle (independent of the view buttons) */
    let lidar = null, showLidar = true;
    const lidarBtn = document.createElement('button');
    lidarBtn.textContent = 'LIDAR';
    lidarBtn.classList.add('on');
    lidarBtn.onclick = () => {
      showLidar = !showLidar;
      lidarBtn.classList.toggle('on', showLidar);
      if (lidar) lidar.visible = showLidar;
      draw();
    };
    viewBox.appendChild(lidarBtn);

    function buildLegend() {
      const g = stops.map(([h, c]) =>
        `rgb(${c.map(Math.round).join(',')}) ${((h - H_MIN) / (H_MAX - H_MIN) * 100).toFixed(1)}%`);
      legend.innerHTML =
        `relief above local ground (m) · height ×${V_EXAG}` +
        `<div class="bar" style="background:linear-gradient(90deg,${g.join(',')})"></div>` +
        `<div class="ticks"><span>${H_MIN.toFixed(1)}</span><span>0</span>` +
        `<span>+${threshold.toFixed(2)}</span><span>+${H_MAX.toFixed(1)}</span></div>` +
        `≥ +${threshold.toFixed(2)} m obstacle · dark = unobserved<br>` +
        `grey arcs: 1 m range steps · white dots: lidar returns`;
    }
    buildLegend();

    function init() {
      scene = new THREE.Scene();
      scene.background = new THREE.Color(0x10151c);
      cam = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
      cam.up.set(0, 0, 1);
      renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.domElement.style.cssText = 'position:absolute;inset:0;width:100%;height:100%';
      el.appendChild(renderer.domElement);
      el.appendChild(legend); el.appendChild(stats); el.appendChild(viewBox);
      /* hemisphere fill plus a low sun for soft shading with visible relief */
      scene.add(new THREE.HemisphereLight(0xdfe9f5, 0x2a3442, 0.65));
      const sun = new THREE.DirectionalLight(0xffffff, 0.7);
      sun.position.set(-3, -5, 6);
      scene.add(sun);
    }

    /* Metric rulers: forward (teal) on the left edge, lateral (blue) on the near
       edge, height (amber, true metres) at the far-right corner. Colours match
       the lidar panel. */
    function buildAxes(group, wm, hm) {
      const COL = { fwd: 0x33d6a6, lat: 0x59b0ff, up: 0xffbe5c };
      const TXT = { fwd: '#8fe3c8', lat: '#9fcbf2', up: '#ffd28a' };
      const x0 = -wm / 2, x1 = wm / 2, T = 0.07, Z = 0.004;   /* tick length, lift off ground */
      const fmt = v => (Math.abs(v - Math.round(v)) < 1e-6 ? v.toFixed(0) : v.toFixed(1));

      /* forward axis: left edge, ticks every 0.5 m, labels every 1 m */
      const f = [x0, 0, Z, x0, hm, Z];
      for (let y = 0; y <= hm + 1e-6; y += 0.5) {
        const major = Math.abs(y - Math.round(y)) < 1e-6;
        f.push(x0, y, Z, x0 - (major ? 2 : 1) * T, y, Z);
        if (major) {
          const t = label(fmt(y), TXT.fwd, 0.17, 1);
          t.position.set(x0 - 2.6 * T, y, Z); group.add(t);
        }
      }
      group.add(lineSegments(f, COL.fwd, 1));
      const ft = label('forward (m)', TXT.fwd, 0.17, 0);
      ft.position.set(x0 - 0.05, hm + 0.25, Z); group.add(ft);

      /* lateral axis: near edge (camera plane), right positive */
      const l = [x0, 0, Z, x1, 0, Z];
      for (let x = Math.ceil(x0 * 2) / 2; x <= x1 + 1e-6; x += 0.5) {
        const major = Math.abs(x - Math.round(x)) < 1e-6;
        l.push(x, 0, Z, x, -(major ? 2 : 1) * T, Z);
        if (major && Math.abs(x) > 0.4) {           /* the rover sits at 0 */
          const t = label((x > 0 ? '+' : '') + fmt(x), TXT.lat, 0.17, 0.5);
          t.position.set(x, -0.27, Z); group.add(t);
        }
      }
      group.add(lineSegments(l, COL.lat, 1));
      const lt = label('lateral (m), right +', TXT.lat, 0.17, 1);
      lt.position.set(x1, -0.52, Z); group.add(lt);

      /* height axis: separate group so the top view can hide it (seen edge-on) */
      const up = new THREE.Group();
      heightAxis = up;
      const zLo = -0.25, zHi = 0.75;
      const u = [x1, hm, zLo * V_EXAG, x1, hm, zHi * V_EXAG];
      for (let z = zLo; z <= zHi + 1e-6; z += 0.25) {
        u.push(x1, hm, z * V_EXAG, x1 + 1.5 * T, hm, z * V_EXAG);
        const t = label((z > 0 ? '+' : '') + z.toFixed(2), TXT.up, 0.15, 0);
        t.position.set(x1 + 2.2 * T, hm, z * V_EXAG); up.add(t);
      }
      up.add(lineSegments(u, COL.up, 1));
      const ut = label('height (m)', TXT.up, 0.17, 0.5);
      ut.position.set(x1, hm, zHi * V_EXAG + 0.2); up.add(ut);
      group.add(up);
    }

    /* Ground reference (z = 0): 0.5 m grid, range arcs, rulers and the rover. */
    function buildGround(wm, hm) {
      if (ground) scene.remove(ground);
      ground = new THREE.Group();
      const pts = [];
      const x0 = -wm / 2, x1 = wm / 2;
      for (let x = Math.ceil(x0 / 0.5) * 0.5; x <= x1 + 1e-6; x += 0.5) pts.push(x, 0, 0, x, hm, 0);
      for (let y = 0; y <= hm + 1e-6; y += 0.5) pts.push(x0, y, 0, x1, y, 0);
      ground.add(lineSegments(pts, 0x3a4d66, 0.55));

      /* range arcs around the camera, clipped to the grid */
      const inside = (x, y) => Math.abs(x) <= wm / 2 && y >= 0 && y <= hm;
      for (let r = 1; r <= Math.floor(hm); r++) {
        const arc = [];
        for (let i = 0; i < 64; i++) {
          const t0 = Math.PI * i / 64, t1 = Math.PI * (i + 1) / 64;
          const ax = r * Math.cos(t0), ay = r * Math.sin(t0), bx = r * Math.cos(t1), by = r * Math.sin(t1);
          if (inside(ax, ay) && inside(bx, by)) arc.push(ax, ay, 0.005, bx, by, 0.005);
        }
        ground.add(lineSegments(arc, 0x8aa0b8, 0.45));
      }
      buildAxes(ground, wm, hm);

      /* flat rover footprint behind the camera plane, with a heading arrow */
      const body = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.55, 0.03),
        new THREE.MeshBasicMaterial({ color: 0x33d6a6, transparent: true, opacity: 0.45 }));
      body.position.set(0, -0.3, 0.015);
      ground.add(body);
      ground.add(lineSegments([-0.225, -0.575, 0.03, 0.225, -0.575, 0.03, 0.225, -0.575, 0.03, 0.225, -0.025, 0.03,
                               0.225, -0.025, 0.03, -0.225, -0.025, 0.03, -0.225, -0.025, 0.03, -0.225, -0.575, 0.03],
                              0x33d6a6, 1));
      const arrow = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.2, 3),
        new THREE.MeshBasicMaterial({ color: 0x33d6a6 }));
      arrow.rotation.x = -Math.PI / 2;       /* points +y (forward) */
      arrow.position.set(0, -0.12, 0.04);
      ground.add(arrow);
      scene.add(ground);
    }

    function buildMesh(w, h, res) {
      if (mesh) { scene.remove(mesh); mesh.geometry.dispose(); mesh.material.dispose(); }
      const geo = new THREE.BufferGeometry();
      const pos = new Float32Array(w * h * 3);
      for (let r = 0; r < h; r++)
        for (let c = 0; c < w; c++) {
          const i = (r * w + c) * 3;
          pos[i] = (c - (w - 1) / 2) * res;   /* lateral, right + */
          pos[i + 1] = (r + 0.5) * res;       /* forward from the camera plane */
        }
      geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(w * h * 3), 3));
      mesh = new THREE.Mesh(geo, new THREE.MeshLambertMaterial(
        { vertexColors: true, side: THREE.DoubleSide }));
      scene.add(mesh);
      buildGround(w * res, h * res);
      layout = { w, h, res };
    }

    function draw() {
      if (!renderer) return;
      const w = el.clientWidth, h = el.clientHeight;
      if (!w || !h) return;
      el.classList.toggle('dem-compact', w < COMPACT_W || h < COMPACT_H);
      renderStats();
      /* render at device resolution; a CSS-pixel canvas is blurry on HiDPI screens */
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      if (renderer.getPixelRatio() !== dpr) renderer.setPixelRatio(dpr);
      renderer.setSize(w, h, false);
      cam.aspect = w / h; cam.updateProjectionMatrix();
      const o = orbit;
      const tanV = Math.tan(cam.fov * Math.PI / 360);
      const dist = o.zoom * Math.max(o.half[0] / (tanV * cam.aspect), o.half[1] / tanV);
      if (heightAxis) heightAxis.visible = o.pitch < 1.3;   /* edge-on from above */
      cam.position.set(
        target.x + dist * Math.cos(o.pitch) * Math.cos(o.yaw),
        target.y + dist * Math.cos(o.pitch) * Math.sin(o.yaw),
        target.z + dist * Math.sin(o.pitch));
      cam.lookAt(target);
      renderer.render(scene, cam);
    }

    function update(m) {
      if (document.hidden || !window.THREE) return;
      if (!renderer) init();
      if (opts.nosig) opts.nosig.style.display = 'none';
      if (m.obstacle_m !== undefined && m.obstacle_m !== threshold) {
        threshold = m.obstacle_m; stops = rampStops(threshold); buildLegend();
      }
      if (!layout || layout.w !== m.w || layout.h !== m.h || layout.res !== m.res) {
        buildMesh(m.w, m.h, m.res);
        target = new THREE.Vector3(0, m.h * m.res / 2, 0);
      }

      const geo = mesh.geometry;
      const pos = geo.attributes.position.array, col = geo.attributes.color.array;
      const known = new Uint8Array(m.w * m.h);
      const [pa, pb, pc] = m.plane || [0, 0, 0];   /* local ground z = a*x + b*y + c */
      for (let i = 0; i < m.w * m.h; i++) {
        const zc = m.z[i];
        if (zc === UNKNOWN) { pos[i * 3 + 2] = 0; continue; }
        known[i] = 1;
        const z = zc / 100;
        pos[i * 3 + 2] = z * V_EXAG;
        /* colour by relief above the local ground so slopes are not shown as obstacles */
        const relief = z - (pa * pos[i * 3] + pb * pos[i * 3 + 1] + pc);
        const c = rampColor(relief, stops);
        col[i * 3] = c[0] / 255; col[i * 3 + 1] = c[1] / 255; col[i * 3 + 2] = c[2] / 255;
      }
      /* faces only where all four corners were observed */
      const idx = [];
      for (let r = 0; r < m.h - 1; r++)
        for (let c = 0; c < m.w - 1; c++) {
          const a = r * m.w + c, b = a + 1, d = a + m.w, e = d + 1;
          if (known[a] && known[b] && known[d] && known[e]) idx.push(a, b, e, a, e, d);
        }
      geo.setIndex(idx);
      geo.attributes.position.needsUpdate = true;
      geo.attributes.color.needsUpdate = true;
      geo.computeVertexNormals();

      lastStats = {
        pct: Math.round(100 * m.observed),
        top: m.max_relief_m ?? 0,
        slope: Math.atan(Math.hypot(pa, pb)) * 180 / Math.PI,
        obstacles: m.obstacles,
      };
      if (opts.info) opts.info.textContent = `${m.w}×${m.h} @ ${m.res} m · rover frame`;
      draw();
    }

    let lastStats = null;
    function renderStats() {
      if (!lastStats) return;
      const s = lastStats, sign = s.top >= 0 ? '+' : '';
      stats.style.display = '';
      stats.innerHTML = el.classList.contains('dem-compact')
        ? `${s.pct}% · ${s.slope.toFixed(1)}° · ${sign}${s.top.toFixed(2)} m` +
          (s.obstacles ? ` · <b>${s.obstacles} obst.</b>` : '')
        : `observed ${s.pct}% · ground slope ${s.slope.toFixed(1)}° · ` +
          `max relief ${sign}${s.top.toFixed(2)} m` +
          (s.obstacles ? ` · <b>${s.obstacles} obstacle cells</b>` : ' · no obstacles');
    }

    /* ---- orbit interaction ---- */
    let drag = false, lx = 0, ly = 0;
    el.style.cursor = 'grab';
    el.addEventListener('mousedown', e => {
      if (e.target.tagName === 'BUTTON') return;
      drag = true; lx = e.clientX; ly = e.clientY; el.style.cursor = 'grabbing';
    });
    window.addEventListener('mouseup', () => { drag = false; el.style.cursor = 'grab'; });
    window.addEventListener('mousemove', e => {
      if (!drag) return;
      orbit.yaw -= (e.clientX - lx) * 0.008;
      orbit.pitch = Math.max(0.08, Math.min(1.55, orbit.pitch + (e.clientY - ly) * 0.008));
      lx = e.clientX; ly = e.clientY;
      setActive(null);
      draw();
    });
    el.addEventListener('wheel', e => {
      e.preventDefault();
      orbit.zoom = Math.max(0.25, Math.min(4, orbit.zoom * (e.deltaY > 0 ? 1.12 : 0.89)));
      draw();
    }, { passive: false });
    window.addEventListener('resize', draw);

    /* Lidar returns in the DEM frame (flat x, y, z list in metres). */
    function updateLidar(m) {
      if (document.hidden || !renderer || !m.xyz) return;
      const n = m.xyz.length / 3;
      const pos = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) {
        pos[i * 3] = m.xyz[i * 3];
        pos[i * 3 + 1] = m.xyz[i * 3 + 1];
        pos[i * 3 + 2] = m.xyz[i * 3 + 2] * V_EXAG + 0.01;   /* just above the surface */
      }
      if (!lidar) {
        lidar = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial(
          { color: 0xffffff, size: 0.035, transparent: true, opacity: 0.9 }));
        scene.add(lidar);
      }
      lidar.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      lidar.geometry.computeBoundingSphere();
      lidar.visible = showLidar;
      draw();
    }

    return { update, updateLidar, redraw: draw };
  };
})();
