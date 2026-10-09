/* =========================================================================
 * Planned-path overlay on the central navcam — overview, camera wall, teleop.
 *
 * Draws the 'path_overlay' feed (planned path ahead of the rover projected
 * into the image by the dashboard backend): a rover-width ribbon, its centre
 * line and distance marks. A PATH button toggles it (remembered).
 *
 * Usage: NavOverlay.attach(socket, [feedEl, …]); each feed element holds the
 * navcam <img> and must be position:relative.
 * ========================================================================= */
(function () {
  'use strict';

  const STYLE = `
    .po-canvas { position:absolute; inset:0; width:100%; height:100%; pointer-events:none; z-index:3; }
    .po-toggle { position:absolute; right:5px; top:5px; z-index:5; cursor:pointer;
      font:calc(8px * var(--fs, 1)) Consolas,Menlo,monospace; letter-spacing:1px; color:#8aa0b8;
      background:rgba(16,21,28,.75); border:1px solid #2c3a4e; border-radius:3px; padding:1px 5px; }
    .po-toggle:hover { color:#e6edf5; border-color:#8aa0b8; }
    .po-toggle.on { color:#ffbe5c; border-color:#ffbe5c; }`;

  function shown() {
    try { return localStorage.getItem('po_show') !== '0'; } catch (e) { return true; }
  }

  /* Displayed image rectangle inside the feed (object-fit: contain). */
  function imageRect(feed) {
    const img = feed.querySelector('img');
    const W = feed.clientWidth, H = feed.clientHeight;
    const iw = (img && img.naturalWidth) || 640, ih = (img && img.naturalHeight) || 480;
    const s = Math.min(W / iw, H / ih);
    return { x: (W - iw * s) / 2, y: (H - ih * s) / 2, w: iw * s, h: ih * s };
  }

  function draw(feed, canvas, m) {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = feed.clientWidth * dpr; canvas.height = feed.clientHeight * dpr;
    const g = canvas.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, feed.clientWidth, feed.clientHeight);
    if (!m || !m.center || !m.center.length || !shown()) return;

    const r = imageRect(feed);
    const P = uv => uv && [r.x + uv[0] * r.w, r.y + uv[1] * r.h];
    g.save();
    g.beginPath(); g.rect(r.x, r.y, r.w, r.h); g.clip();

    /* ribbon: one quad per sample pair where all four corners are in front of the camera */
    g.fillStyle = 'rgba(255,190,92,0.22)';
    for (let i = 0; i + 1 < m.center.length; i++) {
      const a = P(m.left[i]), b = P(m.left[i + 1]), c = P(m.right[i + 1]), d = P(m.right[i]);
      if (!a || !b || !c || !d) continue;
      g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.lineTo(...c); g.lineTo(...d); g.closePath(); g.fill();
    }
    /* ribbon edges and dashed centre line */
    const line = (pts, style, width, dash) => {
      g.strokeStyle = style; g.lineWidth = width; g.setLineDash(dash || []);
      g.beginPath(); let pen = false;
      for (const uv of pts) {
        const q = P(uv);
        if (!q) { pen = false; continue; }
        pen ? g.lineTo(...q) : g.moveTo(...q); pen = true;
      }
      g.stroke();
    };
    line(m.left, 'rgba(255,190,92,0.85)', 1.5);
    line(m.right, 'rgba(255,190,92,0.85)', 1.5);
    line(m.center, 'rgba(255,255,255,0.75)', 1.2, [6, 5]);
    g.setLineDash([]);

    /* Metre marks across the ribbon, labelled every 2 m. Marks crowd towards
       the horizon, so they are placed near to far: a tick is skipped when it
       would sit on the previous one, and labelling stops at the first label
       that would overlap, so the numbers stay in sequence. */
    g.font = 'bold 10px Consolas, Menlo, monospace'; g.textBaseline = 'middle';
    const MIN_TICK_GAP = 4, PAD = 2;
    const boxes = [];                          /* screen boxes of drawn labels */
    let lastTick = null, labelling = true;
    for (let i = 0; i < m.dist.length; i++) {
      const dm = m.dist[i];
      if (dm < 1 || Math.abs(dm - Math.round(dm)) > 1e-6) continue;
      const a = P(m.left[i]), b = P(m.right[i]);
      if (!a || !b) continue;
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      if (lastTick && Math.hypot(mid[0] - lastTick[0], mid[1] - lastTick[1]) < MIN_TICK_GAP) continue;
      lastTick = mid;
      g.strokeStyle = 'rgba(255,190,92,0.95)'; g.lineWidth = 1.5;
      g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke();
      if (!labelling || Math.round(dm) % 2 !== 0) continue;

      const txt = `${Math.round(dm)} m`;
      const edge = a[0] > b[0] ? a : b;        /* label beside the right-hand end */
      const tw = g.measureText(txt).width, th = 11;
      const box = { x: edge[0] + 5 - PAD, y: edge[1] - th / 2 - PAD, w: tw + 2 * PAD, h: th + 2 * PAD };
      if (boxes.some(o => box.x < o.x + o.w && o.x < box.x + box.w && box.y < o.y + o.h && o.y < box.y + box.h)) {
        labelling = false;
        continue;
      }
      boxes.push(box);
      g.lineWidth = 3; g.strokeStyle = 'rgba(16,21,28,0.9)'; g.strokeText(txt, edge[0] + 5, edge[1]);
      g.fillStyle = '#ffd28a'; g.fillText(txt, edge[0] + 5, edge[1]);
    }
    g.restore();
  }

  window.NavOverlay = {
    attach(socket, feeds) {
      if (!document.getElementById('nav-overlay-style')) {
        const st = document.createElement('style');
        st.id = 'nav-overlay-style'; st.textContent = STYLE;
        document.head.appendChild(st);
      }
      let last = null;
      const layers = feeds.filter(Boolean).map(feed => {
        const canvas = document.createElement('canvas');
        canvas.className = 'po-canvas';
        feed.appendChild(canvas);
        const t = document.createElement('button');
        t.className = 'po-toggle';
        t.textContent = 'PATH';
        t.title = 'Show / hide the planned path';
        t.classList.toggle('on', shown());
        t.onclick = e => {
          e.stopPropagation();
          const on = !shown();
          try { localStorage.setItem('po_show', on ? '1' : '0'); } catch (err) {}
          document.querySelectorAll('.po-toggle').forEach(b => b.classList.toggle('on', on));
          redraw();
        };
        feed.appendChild(t);
        return { feed, canvas };
      });
      const redraw = () => layers.forEach(l => draw(l.feed, l.canvas, last));
      socket.on('path_overlay', m => { last = m; redraw(); });
      window.addEventListener('resize', redraw);
    },
  };
})();
