/* Alert banner, shared by every dashboard window.
 *
 *   GNCAlerts.attach(socket);
 *
 * The server's 'health' push (once a second) carries `alerts`: feeds that went
 * quiet while Unreal is in Play and lifecycle nodes that left 'active'. They are
 * shown as a stack under the window's header; an alert disappears by itself when
 * the problem clears. × hides one until it clears and comes back.
 */
(function () {
  'use strict';
  const MAX_SHOWN = 4;

  const style = document.createElement('style');
  style.textContent = `
    #gnc-alerts { position: fixed; top: calc(58px * var(--fs, 1)); right: 12px; z-index: 1000;
                  display: flex; flex-direction: column; gap: 6px; align-items: flex-end;
                  max-width: min(440px, calc(100vw - 24px)); pointer-events: none; }
    #gnc-alerts .al { pointer-events: auto; display: flex; align-items: center; gap: 10px;
                      font: calc(11px * var(--fs, 1))/1.4 var(--mono, monospace); letter-spacing: .5px;
                      color: var(--text); background: var(--panel); border: 1px solid var(--line);
                      border-left: 4px solid var(--amber); border-radius: 4px; padding: 7px 8px 7px 10px;
                      box-shadow: 0 6px 20px rgba(0, 0, 0, .35); animation: gnc-al-in .2s ease-out; }
    #gnc-alerts .al.error { border-left-color: var(--red); }
    #gnc-alerts .al.info  { border-left-color: var(--cyan); }
    #gnc-alerts .al .lv { font-size: calc(9px * var(--fs, 1)); letter-spacing: 1px; color: var(--amber); }
    #gnc-alerts .al.error .lv { color: var(--red); }
    #gnc-alerts .al.info .lv  { color: var(--cyan); }
    #gnc-alerts .al .tx { flex: 1; }
    #gnc-alerts .al button { background: none; border: 0; color: var(--dim); cursor: pointer;
                             font: inherit; font-size: calc(13px * var(--fs, 1)); padding: 0 2px; }
    #gnc-alerts .al button:hover { color: var(--text); }
    #gnc-alerts .more { pointer-events: none; color: var(--dim); font: calc(10px * var(--fs, 1)) var(--mono, monospace); }
    @keyframes gnc-al-in { from { opacity: 0; transform: translateY(-4px); } to { opacity: 1; transform: none; } }
    @media (prefers-reduced-motion: reduce) { #gnc-alerts .al { animation: none; } }`;
  document.head.appendChild(style);

  const LABEL = { error: 'ERROR', warn: 'WARNING', info: 'INFO' };
  const RANK = { error: 0, warn: 1, info: 2 };
  const dismissed = new Set();      /* keys hidden with ×, until they clear */
  let box = null, last = [];

  function render() {
    if (!box) {
      box = document.createElement('div');
      box.id = 'gnc-alerts';
      box.setAttribute('role', 'status');
      box.setAttribute('aria-live', 'polite');
      document.body.appendChild(box);
    }
    const shown = last.filter(a => !dismissed.has(a.key))
                      .sort((a, b) => (RANK[a.level] ?? 1) - (RANK[b.level] ?? 1));
    const keep = new Set(shown.slice(0, MAX_SHOWN).map(a => a.key));
    /* keep existing rows (no re-animation), add new ones, drop cleared ones */
    box.querySelectorAll('.al').forEach(el => { if (!keep.has(el.dataset.key)) el.remove(); });
    shown.slice(0, MAX_SHOWN).forEach((a, i) => {
      let el = box.querySelector(`.al[data-key="${CSS.escape(a.key)}"]`);
      if (!el) {
        el = document.createElement('div');
        el.dataset.key = a.key;
        el.innerHTML = '<span class="lv"></span><span class="tx"></span><button title="Hide until it clears">×</button>';
        el.querySelector('button').onclick = () => { dismissed.add(a.key); render(); };
      }
      el.className = 'al ' + (a.level || 'warn');
      el.querySelector('.lv').textContent = LABEL[a.level] || 'WARNING';
      el.querySelector('.tx').textContent = a.text;
      if (box.children[i] !== el) box.insertBefore(el, box.children[i] || null);
    });
    let more = box.querySelector('.more');
    const extra = shown.length - MAX_SHOWN;
    if (extra > 0) {
      if (!more) { more = document.createElement('div'); more.className = 'more'; }
      more.textContent = `+${extra} more — see the System window`;
      box.appendChild(more);
    } else if (more) more.remove();
  }

  function update(alerts) {
    last = Array.isArray(alerts) ? alerts : [];
    const live = new Set(last.map(a => a.key));
    for (const k of [...dismissed]) if (!live.has(k)) dismissed.delete(k);   /* cleared: may show again */
    if (document.body) render();
  }

  window.GNCAlerts = {
    attach(socket) {
      socket.on('health', h => update(h && h.alerts));
      socket.on('disconnect', () => update([]));
    },
  };
})();
