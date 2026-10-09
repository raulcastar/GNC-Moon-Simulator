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

  function apply(name) {
    const theme = THEMES[name] || THEMES.mission;
    const root = document.documentElement;
    for (const [k, v] of Object.entries(theme.vars)) root.style.setProperty(k, v);
    root.dataset.theme = THEMES[name] ? name : 'mission';
    root.style.colorScheme = theme.light ? 'light' : 'dark';
    window.GNC_THEME = root.dataset.theme;
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
