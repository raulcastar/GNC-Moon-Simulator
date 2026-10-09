/* Dashboard colour themes, chosen in the hub (menu) and shared by every window.
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
    daylight: { label: 'Daylight', vars: {
      '--bg': '#eef1f5', '--panel': '#ffffff', '--panel2': '#f3f5f8', '--line': '#cfd7e2', '--grid': '#e4e9f0',
      '--text': '#1b2533', '--dim': '#5b6b80', '--accent': '#0a8a68', '--cyan': '#1f6fd1', '--amber': '#a86400',
      '--red': '#c8323f', '--view': '#e6ebf1', '--view2': '#dee4eb', '--view-line': '#c2ccd8',
      '--view-text': '#7a8a9e' } },
  };

  function apply(name) {
    const theme = THEMES[name] || THEMES.mission;
    const root = document.documentElement;
    for (const [k, v] of Object.entries(theme.vars)) root.style.setProperty(k, v);
    root.dataset.theme = THEMES[name] ? name : 'mission';
    root.style.colorScheme = name === 'daylight' ? 'light' : 'dark';
    window.GNC_THEME = root.dataset.theme;
    window.dispatchEvent(new Event('gnc-theme'));
  }

  apply('__THEME__');
  window.GNCTheme = { THEMES, apply, current: () => window.GNC_THEME };

  /* Follow a change made in the hub while this window is open. */
  setInterval(() => {
    fetch('/theme.json', { cache: 'no-store' }).then(r => r.json())
      .then(j => { if (j.theme && j.theme !== window.GNC_THEME) apply(j.theme); }).catch(() => {});
  }, 4000);
})();
