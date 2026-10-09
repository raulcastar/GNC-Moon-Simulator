/* Helpers shared by the dashboard pages (overview, teleop, system).
 *
 *   escHtml(text)          text made safe to put inside HTML
 *   ViewPicker.attach(o)   a "VIEWS ▾" button opening a list of toggle chips
 */

const escHtml = t => String(t).replace(/[&<>"]/g,
  c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

/* ViewPicker: which of a page's views are shown, remembered per page.
 *
 *   ViewPicker.attach({
 *     button,                  the element that opens the list
 *     items,                   [{ key, label }] in display order
 *     storageKey,              localStorage key for the selection
 *     onChange(selection),     { key: true|false }; called once at attach time too
 *     title = 'CAMERA VIEWS',
 *   })
 *
 * At least one item stays on. ALL turns everything on. The list closes on an
 * outside click or Esc. The button gives up focus after a click, so keys such
 * as Space keep their page meaning (teleop: stop). */
const ViewPicker = (() => {
  const CSS = `
    .vp-pop { display: none; position: fixed; z-index: 50; width: 300px; padding: 8px 10px; gap: 5px;
              flex-wrap: wrap; background: var(--panel); border: 1px solid var(--line); border-radius: 6px;
              box-shadow: 0 8px 28px rgba(0,0,0,.5); font-family: var(--mono); font-size: calc(10px * var(--fs, 1)); }
    .vp-pop.open { display: flex; }
    .vp-pop h4 { flex-basis: 100%; margin: 0 0 3px; color: var(--dim); font-weight: normal;
                 font-size: calc(9px * var(--fs, 1)); letter-spacing: 2px; }
    .vp-pop .chip { font-size: calc(10px * var(--fs, 1)); letter-spacing: 1px; cursor: pointer; user-select: none;
                    color: var(--dim); border: 1px solid var(--line); border-radius: 4px; padding: 4px 8px; }
    .vp-pop .chip:hover { border-color: var(--cyan); }
    .vp-pop .chip.on { color: var(--bg); background: var(--accent); border-color: var(--accent); font-weight: bold; }
    .vp-pop .all { margin-left: auto; color: var(--cyan); cursor: pointer; letter-spacing: 1px; }`;
  let styled = false;

  function attach({ button, items, storageKey, onChange, title = 'CAMERA VIEWS' }) {
    if (!styled) {
      const style = document.createElement('style');
      style.textContent = CSS;
      document.head.appendChild(style);
      styled = true;
    }
    let sel = {};
    try { sel = JSON.parse(localStorage.getItem(storageKey) || '{}'); } catch (e) { sel = {}; }
    items.forEach(i => { if (!(i.key in sel)) sel[i.key] = true; });
    if (!items.some(i => sel[i.key])) items.forEach(i => sel[i.key] = true);

    const pop = document.createElement('div');
    pop.className = 'vp-pop';
    document.body.appendChild(pop);
    pop.addEventListener('click', e => e.stopPropagation());   /* rebuilt chips are detached */

    const changed = () => {
      try { localStorage.setItem(storageKey, JSON.stringify(sel)); } catch (e) {}
      onChange({ ...sel });
    };
    function build() {
      pop.innerHTML = `<h4>${escHtml(title)} <span class="all">ALL</span></h4>`;
      items.forEach(i => {
        const chip = document.createElement('span');
        chip.className = 'chip' + (sel[i.key] ? ' on' : '');
        chip.textContent = i.label;
        chip.onclick = () => {
          if (sel[i.key] && items.filter(j => sel[j.key]).length === 1) return;   /* keep one */
          sel[i.key] = !sel[i.key];
          chip.classList.toggle('on', sel[i.key]);
          changed();
        };
        pop.appendChild(chip);
      });
      pop.querySelector('.all').onclick = () => { items.forEach(i => sel[i.key] = true); build(); changed(); };
    }
    button.addEventListener('click', e => {
      e.stopPropagation();
      button.blur();
      if (pop.classList.toggle('open')) {
        build();
        const r = button.getBoundingClientRect();
        pop.style.top = (r.bottom + 4) + 'px';
        pop.style.left = Math.max(6, Math.min(r.left, window.innerWidth - 310)) + 'px';
      }
    });
    document.addEventListener('click', () => pop.classList.remove('open'));
    window.addEventListener('keydown', e => { if (e.key === 'Escape') pop.classList.remove('open'); });
    onChange({ ...sel });
  }

  return { attach };
})();
