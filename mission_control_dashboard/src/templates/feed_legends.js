/* =========================================================================
 * Feed legends — shared by the overview, camera wall and teleop pages.
 *
 *   depth: JET colour bar over 0 … depth_max_distance (component_stereo_depth)
 *   mask:   class colours and stencil ids of the object mask, named by
 *           object_mask_mapping and grouped into ground / objects
 *   shadow: shadow mask colours with the shadow_label class ids, grouped
 *
 * Usage: FeedLegends.attach(socket, { depth: [feedEl, …], mask: [feedEl, …],
 *                                    shadow: [feedEl, …] })
 * Each feed element must be position:relative; the legend is overlaid on it,
 * with a LEGEND button to toggle it (remembered per legend type).
 * ========================================================================= */
(function () {
  'use strict';

  /* Object-mask colour per id: the fixed palette of PPM_GroundTruthMask
     (MASK_PALETTE in Terrain Studio's unreal_ground_truth.py; lowest bit of each
     channel cleared, as it carries the shadow mask). Keep the two in step. */
  const MASK_COLOURS = {
    '0':  [0, 0, 0],         /* unlabeled   */
    '1':  [36, 40, 96],      /* sky         */
    '2':  [250, 194, 16],    /* rover       */
    '3':  [28, 128, 162],    /* sun         */
    '4':  [230, 14, 118],    /* sun glare   */
    '5':  [10, 96, 216],     /* landscape   */
    '6':  [124, 50, 208],    /* crater      */
    '7':  [86, 52, 82],      /* crater rim  */
    '8':  [64, 124, 162],    /* wheel track */
    '9':  [238, 244, 242],   /* boulder     */
    '10': [210, 62, 6],      /* small rock  */
  };

  /* Object-mask classes per group (by name, from object_mask_mapping). */
  const MASK_GROUPS = [
    ['ground', ['landscape', 'crater', 'crater_rim', 'wheel_track']],
    ['objects', ['rover', 'boulder', 'small_rock']],
    ['sky', ['sky', 'sun', 'sun_glare']],
    ['other', ['unlabeled']],
  ];

  /* Shadow feed: colour and shadow_label id per class, grouped. Must match
     SHADOW_IDS in component_stereo_depth. */
  const SHADOW_GROUPS = [
    ['ground', [[1, 'sunlit', 'rgb(40,120,255)'], [2, 'shadowed', '#000']]],
    ['objects', [[3, 'sunlit', 'rgb(128,128,128)'], [4, 'shadowed', '#000']]],
    ['rover shadow', [[5, 'on ground or rocks', 'rgb(255,140,0)']]],
    ['sky', [[0, 'sky', '#000']]],
  ];

  /* OpenCV COLORMAP_JET stops */
  const JET = 'linear-gradient(90deg, rgb(0,0,128) 0%, rgb(0,0,255) 12.5%, ' +
              'rgb(0,255,255) 37.5%, rgb(255,255,0) 62.5%, rgb(255,0,0) 87.5%, rgb(128,0,0) 100%)';

  const STYLE = `
    .fl-box { position:absolute; left:6px; bottom:6px; z-index:4; pointer-events:none;
      font:9px/1.4 Consolas,Menlo,monospace; color:#cfe0ee; letter-spacing:.4px;
      background:rgba(16,21,28,.82); border:1px solid #2c3a4e; border-radius:4px;
      padding:4px 6px; }
    .fl-bar { width:120px; height:7px; border-radius:2px; margin:2px 0 1px; }
    .fl-ticks { display:flex; justify-content:space-between; width:120px; }
    .fl-row { display:flex; align-items:center; gap:5px; white-space:nowrap; }
    .fl-sw { width:9px; height:9px; border-radius:2px; border:1px solid rgba(255,255,255,.25); }
    .fl-id { color:#33d6a6; min-width:14px; text-align:right; }
    .fl-grp { color:#8aa0b8; font-size:8px; letter-spacing:1px; text-transform:uppercase; margin-top:2px; }
    .fl-grp:first-child { margin-top:0; }
    /* compact form for small tiles: one wrapped line, smaller text */
    .fl-mini { left:3px; right:3px; bottom:3px; padding:2px 4px; font-size:8px; line-height:1.3;
               display:flex; flex-wrap:wrap; gap:1px 7px; align-items:center; }
    .fl-mini .fl-bar, .fl-mini .fl-ticks { width:70px; }
    .fl-mini .fl-bar { height:5px; }
    .fl-mini .fl-sw { width:7px; height:7px; }
    .fl-mini .fl-id { min-width:0; }
    .fl-mini .fl-grp { margin-top:0; font-size:7px; }
    .fl-toggle { position:absolute; right:5px; top:5px; z-index:5; cursor:pointer;
      font:8px Consolas,Menlo,monospace; letter-spacing:1px; color:#8aa0b8;
      background:rgba(16,21,28,.75); border:1px solid #2c3a4e; border-radius:3px; padding:1px 5px; }
    .fl-toggle:hover { color:#e6edf5; border-color:#8aa0b8; }
    .fl-toggle.on { color:#33d6a6; border-color:#33d6a6; }
    .fl-box.fl-off { display:none; }`;

  /* Below this feed width the legends switch to the compact form. */
  const MINI_WIDTH = 330;

  function ensureStyle() {
    if (document.getElementById('feed-legends-style')) return;
    const st = document.createElement('style');
    st.id = 'feed-legends-style'; st.textContent = STYLE;
    document.head.appendChild(st);
  }

  /* Legend visibility per type, shared by all pages and remembered. */
  function shown(kind) {
    try { return localStorage.getItem('fl_show_' + kind) !== '0'; } catch (e) { return true; }
  }
  function setShown(kind, on) {
    try { localStorage.setItem('fl_show_' + kind, on ? '1' : '0'); } catch (e) {}
    document.querySelectorAll('.fl-box.' + kind).forEach(b => b.classList.toggle('fl-off', !on));
    document.querySelectorAll('.fl-toggle.' + kind).forEach(t => t.classList.toggle('on', on));
  }

  function box(feed, kind) {
    let b = feed.querySelector('.fl-box.' + kind);
    if (!b) {
      b = document.createElement('div');
      b.className = 'fl-box ' + kind;
      feed.appendChild(b);
      const t = document.createElement('button');
      t.className = 'fl-toggle ' + kind;
      t.textContent = 'LEGEND';
      t.title = 'Show / hide the legend';
      t.onclick = e => { e.stopPropagation(); setShown(kind, !shown(kind)); };
      feed.appendChild(t);
      b.classList.toggle('fl-off', !shown(kind));
      t.classList.toggle('on', shown(kind));
    }
    b.classList.toggle('fl-mini', feed.clientWidth < MINI_WIDTH);
    return b;
  }

  function isMini(feed) { return feed.clientWidth < MINI_WIDTH; }

  function depth(feed, maxM) {
    const m = maxM || 6;
    const b = box(feed, 'depth');
    if (isMini(feed)) {
      b.innerHTML = `<span>0</span><div class="fl-bar" style="background:${JET};margin:0"></div>` +
                    `<span>${m.toFixed(0)} m</span>`;
      return;
    }
    const t = [0, m / 3, 2 * m / 3, m].map(v => v.toFixed(v < 10 ? 1 : 0));
    b.innerHTML =
      `depth (m) · black = no match<div class="fl-bar" style="background:${JET}"></div>` +
      `<div class="fl-ticks">${t.map(v => `<span>${v}</span>`).join('')}</div>`;
  }

  function row(colour, id, name) {
    return `<div class="fl-row"><span class="fl-sw" style="${colour}"></span>` +
           `<span class="fl-id">${id}</span>${name}</div>`;
  }

  function mask(feed, legend) {
    const ids = Object.keys(legend).sort((a, b) => a - b);
    const groupOf = id => {
      const g = MASK_GROUPS.find(([, names]) => names.includes(legend[id]));
      return g ? g[0] : 'other';
    };
    const rows = [];
    ['ground', 'objects', 'other', 'sky'].forEach(group => {
      const members = ids.filter(id => groupOf(id) === group);
      if (!members.length) return;
      rows.push(`<div class="fl-grp">${group}</div>`);
      members.forEach(id => {
        const c = MASK_COLOURS[id];
        const sw = c ? `background:rgb(${c.join(',')})` : 'background:repeating-linear-gradient(45deg,#555 0 2px,#222 2px 4px)';
        rows.push(row(sw, id, legend[id].replace('_', ' ')));
      });
    });
    box(feed, 'mask').innerHTML = rows.join('');
  }

  function shadow(feed) {
    const b = box(feed, 'shadow');
    const rows = [];
    SHADOW_GROUPS.forEach(([group, classes]) => {
      rows.push(`<div class="fl-grp">${group}</div>`);
      classes.forEach(([id, name, colour]) => rows.push(row(`background:${colour}`, id, name)));
    });
    if (!isMini(feed)) rows.push('<div class="fl-grp">ids = shadow_label</div>');
    b.innerHTML = rows.join('');
  }

  window.FeedLegends = {
    attach(socket, els) {
      ensureStyle();
      let maxM = null;
      (els.depth || []).forEach(f => depth(f, maxM));
      (els.shadow || []).forEach(f => shadow(f));
      socket.on('config', c => {
        if (c && c.depth_max_distance) {
          maxM = c.depth_max_distance;
          (els.depth || []).forEach(f => depth(f, maxM));
        }
      });
      let lastLegend = null;
      socket.on('mask_legend', legend => {
        lastLegend = legend;
        (els.mask || []).forEach(f => mask(f, legend));
      });
      /* re-render on resize so the compact form follows the tile size */
      window.addEventListener('resize', () => {
        (els.depth || []).forEach(f => depth(f, maxM));
        if (lastLegend) (els.mask || []).forEach(f => mask(f, lastLegend));
        (els.shadow || []).forEach(f => shadow(f));
      });
    },
  };
})();
