/* Dashboard colour themes and text size, chosen in the hub (menu) and shared by every window.
 *
 * Each window serves this file as /theme.js with the current theme filled in
 * and loads it first, in <head>, so a page never paints in the wrong colours.
 * The choice lives on the dashboard server (the windows are separate origins,
 * so browser storage would not be shared); open windows follow a change within
 * a few seconds. A theme sets the pages' CSS variables, including the viewport
 * colours (--view*) that empty camera tiles, the lidar view and the elevation
 * map use; canvas views listen for the 'gnc-theme' event to repaint.
 */
(function () {
  'use strict';
  const THEMES = {
    mission: { label: 'Mission', vars: {
      '--bg': '#10151c', '--panel': '#18212e', '--panel2': '#202b3a', '--line': '#2c3a4e', '--grid': '#131e2c',
      '--text': '#e6edf5', '--dim': '#8aa0b8', '--accent': '#33d6a6', '--cyan': '#59b0ff', '--amber': '#ffbe5c',
      '--red': '#ff6b7d', '--view': '#10151c', '--view2': '#161d28', '--view-line': '#1b2a3c',
      '--view-text': '#4d6580' } },
    midnight: { label: 'Midnight', vars: {
      '--bg': '#06080b', '--panel': '#0d1117', '--panel2': '#141a22', '--line': '#26303d', '--grid': '#11161d',
      '--text': '#f3f6fa', '--dim': '#9aabbf', '--accent': '#4fd6ff', '--cyan': '#8fb6ff', '--amber': '#ffc857',
      '--red': '#ff5c74', '--view': '#06080b', '--view2': '#0b0f14', '--view-line': '#1a2330',
      '--view-text': '#4f6178' } },
    night: { label: 'Night Ops', vars: {
      '--bg': '#0b0505', '--panel': '#130909', '--panel2': '#1b0d0d', '--line': '#3a1919', '--grid': '#1d0c0c',
      '--text': '#eab7aa', '--dim': '#9c6b60', '--accent': '#ff6a45', '--cyan': '#d98a74', '--amber': '#e8a24c',
      '--red': '#ff3b3b', '--view': '#0b0505', '--view2': '#110808', '--view-line': '#2e1414',
      '--view-text': '#7a443a' } },
    daylight: { label: 'Daylight', light: true, vars: {
      '--bg': '#eef1f5', '--panel': '#ffffff', '--panel2': '#f3f5f8', '--line': '#cfd7e2', '--grid': '#e4e9f0',
      '--text': '#1b2533', '--dim': '#5b6b80', '--accent': '#0a8a68', '--cyan': '#1f6fd1', '--amber': '#a86400',
      '--red': '#c8323f', '--view': '#e6ebf1', '--view2': '#dee4eb', '--view-line': '#c2ccd8',
      '--view-text': '#7a8a9e' } },
    slate: { label: 'Slate', vars: {
      '--bg': '#16181c', '--panel': '#1f2228', '--panel2': '#272b33', '--line': '#383d47', '--grid': '#1b1e23',
      '--text': '#e4e6ea', '--dim': '#9097a3', '--accent': '#7aa2f7', '--cyan': '#7dcfff', '--amber': '#e0af68',
      '--red': '#f7768e', '--view': '#121418', '--view2': '#181b20', '--view-line': '#2a2f38',
      '--view-text': '#5c6370' } },
    solarized: { label: 'Solarized', vars: {
      '--bg': '#002b36', '--panel': '#073642', '--panel2': '#0b4150', '--line': '#1d5563', '--grid': '#04313d',
      '--text': '#eee8d5', '--dim': '#93a1a1', '--accent': '#2aa198', '--cyan': '#268bd2', '--amber': '#b58900',
      '--red': '#dc322f', '--view': '#00222b', '--view2': '#032a34', '--view-line': '#164a57',
      '--view-text': '#4f7380' } },
    amber: { label: 'Amber CRT', vars: {
      '--bg': '#0f0a02', '--panel': '#171005', '--panel2': '#201707', '--line': '#3d2c0c', '--grid': '#1a1204',
      '--text': '#ffcc66', '--dim': '#b38a3c', '--accent': '#ffb000', '--cyan': '#ffd27a', '--amber': '#ff8c1a',
      '--red': '#ff4d2e', '--view': '#0b0701', '--view2': '#120c03', '--view-line': '#33240a',
      '--view-text': '#7a5a20' } },
    contrast: { label: 'High Contrast', vars: {
      '--bg': '#000000', '--panel': '#0a0a0a', '--panel2': '#141414', '--line': '#6b6b6b', '--grid': '#1f1f1f',
      '--text': '#ffffff', '--dim': '#d0d0d0', '--accent': '#00ff9c', '--cyan': '#00d4ff', '--amber': '#ffd400',
      '--red': '#ff4040', '--view': '#000000', '--view2': '#0a0a0a', '--view-line': '#505050',
      '--view-text': '#a0a0a0' } },
    paper: { label: 'Paper', light: true, vars: {
      '--bg': '#f4efe6', '--panel': '#fbf8f2', '--panel2': '#f0e9dc', '--line': '#d6ccb8', '--grid': '#e8e0d0',
      '--text': '#2b2620', '--dim': '#6e6354', '--accent': '#1f7a5c', '--cyan': '#2c5d8f', '--amber': '#9a5b00',
      '--red': '#b3312c', '--view': '#e9e2d4', '--view2': '#e2dacb', '--view-line': '#c8bca6',
      '--view-text': '#8a7d68' } },
  };

  /* Text size: every CSS font size in the dashboard is a multiple of --fs. */
  const FONTS = { small: { label: 'S', scale: 0.9 }, normal: { label: 'M', scale: 1 },
                  large: { label: 'L', scale: 1.15 }, xlarge: { label: 'XL', scale: 1.3 } };

  function applyFont(key) {
    const font = FONTS[key] ? key : 'normal';
    document.documentElement.style.setProperty('--fs', String(FONTS[font].scale));
    window.GNC_FONT = font;
    window.dispatchEvent(new Event('resize'));       /* views that measure text re-layout */
  }

  /* ---- logo: a shaded moon with craters, circled by an orbit and a spacecraft ----
     The back half of the orbit is drawn first so the moon hides it. The header logo
     uses the theme variables; the favicon gets the same drawing with the colours
     filled in (an icon cannot read CSS variables). */
  function logoSvg(c, cls) {
    return `<svg ${cls ? `class="${cls}" ` : ''}xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" aria-hidden="true">` +
      `<g fill="none" stroke="${c.accent}" stroke-width="2.6" stroke-linecap="round" transform="rotate(-20 32 32)">` +
      `<path d="M3 32 A29 10 0 0 1 61 32"/></g>` +
      `<circle cx="32" cy="32" r="17" fill="${c.panel2}" stroke="${c.text}" stroke-width="2.2"/>` +
      `<path d="M32 15 A17 17 0 0 1 32 49 A9 17 0 0 0 32 15 Z" fill="${c.bg}" opacity=".6"/>` +
      `<g fill="none" stroke="${c.dim}" stroke-width="1.6"><circle cx="25" cy="27" r="4"/>` +
      `<circle cx="31" cy="40" r="3"/><circle cx="35" cy="22" r="2"/></g>` +
      `<g transform="rotate(-20 32 32)"><path d="M3 32 A29 10 0 0 0 61 32" fill="none" stroke="${c.accent}" ` +
      `stroke-width="2.6" stroke-linecap="round"/><circle cx="46.5" cy="40.7" r="3" fill="${c.amber}"/></g></svg>`;
  }
  const CSS_COLOURS = { accent: 'var(--accent)', panel2: 'var(--panel2)', text: 'var(--text)', bg: 'var(--bg)',
                        dim: 'var(--dim)', amber: 'var(--amber)' };
  /* One-colour outline of the logo, used as a mask for the empty-feed watermark. */
  const MARK = 'url("data:image/svg+xml,' + encodeURIComponent(
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" fill="none" stroke="#000" stroke-linecap="round">' +
    '<circle cx="32" cy="32" r="17" stroke-width="2.2"/><circle cx="25" cy="27" r="4" stroke-width="1.6"/>' +
    '<circle cx="31" cy="40" r="3" stroke-width="1.6"/><circle cx="35" cy="22" r="2" stroke-width="1.6"/>' +
    '<g transform="rotate(-20 32 32)"><path d="M3 32 A29 10 0 0 0 61 32" stroke-width="2.6"/>' +
    '<circle cx="46.5" cy="40.7" r="3" fill="#000" stroke="none"/></g></svg>') + '")';

  const style = document.createElement('style');
  style.textContent = `
    .gnc-logo { width: calc(30px * var(--fs, 1)); height: calc(30px * var(--fs, 1)); flex: 0 0 auto; display: block; }
    header.hero .gnc-logo { width: 68px; height: 68px; margin: 0 auto 10px;
                            animation: gnc-glow 4s ease-in-out infinite; }
    @keyframes gnc-glow { 0%, 100% { filter: drop-shadow(0 0 4px color-mix(in srgb, var(--accent) 35%, transparent)); }
                          50%      { filter: drop-shadow(0 0 14px color-mix(in srgb, var(--accent) 60%, transparent)); } }
    @media (prefers-reduced-motion: reduce) { header.hero .gnc-logo { animation: none; } }
    .nosig { flex-direction: column; gap: 10px; }
    .nosig::before { content: ''; width: clamp(26px, 18%, 72px); aspect-ratio: 1; flex: 0 0 auto;
                     background: var(--view-text, #4d6580); opacity: .45;
                     -webkit-mask: ${MARK} center / contain no-repeat; mask: ${MARK} center / contain no-repeat; }`;
  document.head.appendChild(style);

  function setFavicon(vars) {
    const c = {};
    for (const k of Object.keys(CSS_COLOURS)) c[k] = vars['--' + k];
    let link = document.querySelector('link[rel="icon"]');
    if (!link) { link = document.createElement('link'); link.rel = 'icon'; document.head.appendChild(link); }
    link.href = 'data:image/svg+xml,' + encodeURIComponent(logoSvg(c));
  }

  /* The logo goes in front of each window's title. */
  document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('header .brand').forEach(b => {
      if (!b.previousElementSibling || !b.previousElementSibling.classList.contains('gnc-logo')) {
        b.insertAdjacentHTML('beforebegin', logoSvg(CSS_COLOURS, 'gnc-logo'));
      }
    });
  });

  function apply(name) {
    const theme = THEMES[name] || THEMES.mission;
    const root = document.documentElement;
    for (const [k, v] of Object.entries(theme.vars)) root.style.setProperty(k, v);
    root.dataset.theme = THEMES[name] ? name : 'mission';
    root.style.colorScheme = theme.light ? 'light' : 'dark';
    window.GNC_THEME = root.dataset.theme;
    setFavicon(theme.vars);
    window.dispatchEvent(new Event('gnc-theme'));
  }

  apply('__THEME__');
  applyFont('__FONT__');
  window.GNCTheme = { THEMES, FONTS, apply, applyFont, current: () => window.GNC_THEME,
                      currentFont: () => window.GNC_FONT };

  /* Follow a change made in the hub while this window is open. */
  setInterval(() => {
    fetch('/theme.json', { cache: 'no-store' }).then(r => r.json())
      .then(j => {
        if (j.theme && j.theme !== window.GNC_THEME) apply(j.theme);
        if (j.font && j.font !== window.GNC_FONT) applyFont(j.font);
      }).catch(() => {});
  }, 4000);
})();
