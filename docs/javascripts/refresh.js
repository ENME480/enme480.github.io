/* Tells a tab that has been open a while that the page it is showing is out of
 * date. GitHub Pages caches HTML for ten minutes, so a browser that is closed
 * and reopened already gets fresh content. The case this covers is a tab left
 * open for hours or days, which matters here because students keep the lab page
 * up while they work and would otherwise follow instructions we have corrected.
 */
(function () {
  // navigation.instant re-runs this on every page swap; only set up once.
  if (window.__enme480Refresh) return;
  window.__enme480Refresh = true;

  var meta = document.querySelector('meta[name="site-build"]');
  if (!meta || !meta.content) return;

  var loaded = meta.content;
  var CHECK_INTERVAL = 5 * 60 * 1000;
  var shown = false;

  function showBanner() {
    if (shown) return;
    shown = true;

    var bar = document.createElement('div');
    bar.setAttribute('role', 'status');
    bar.style.cssText =
      'position:fixed;left:0;right:0;bottom:0;z-index:20;' +
      'display:flex;flex-wrap:wrap;gap:.75rem;align-items:center;' +
      'justify-content:center;padding:.75rem 1rem;' +
      'padding-bottom:calc(.75rem + env(safe-area-inset-bottom, 0px));' +
      'background:var(--md-primary-fg-color,#3f51b5);' +
      'color:var(--md-primary-bg-color,#fff);' +
      'font-size:.75rem;line-height:1.4;' +
      'box-shadow:0 -2px 8px rgba(0,0,0,.2)';

    var text = document.createElement('span');
    text.textContent = 'This page has been updated since you opened it.';
    bar.appendChild(text);

    var reload = document.createElement('button');
    reload.type = 'button';
    reload.textContent = 'Reload';
    reload.style.cssText =
      'cursor:pointer;border:1px solid currentColor;border-radius:.2rem;' +
      'background:transparent;color:inherit;font:inherit;padding:.25rem .75rem';
    reload.addEventListener('click', function () {
      window.location.reload();
    });
    bar.appendChild(reload);

    var dismiss = document.createElement('button');
    dismiss.type = 'button';
    dismiss.textContent = 'Not now';
    dismiss.setAttribute('aria-label', 'Dismiss update notice');
    dismiss.style.cssText =
      'cursor:pointer;border:0;background:transparent;color:inherit;' +
      'font:inherit;padding:.25rem .5rem;opacity:.8';
    dismiss.addEventListener('click', function () {
      bar.remove();
    });
    bar.appendChild(dismiss);

    document.body.appendChild(bar);
  }

  function check() {
    if (shown || document.hidden) return;
    fetch('/version.txt?t=' + Date.now(), { cache: 'no-store' })
      .then(function (r) { return r.ok ? r.text() : null; })
      .then(function (body) {
        if (!body) return;
        var latest = body.trim();
        if (latest && latest !== loaded) showBanner();
      })
      .catch(function () { /* offline or blocked: stay quiet */ });
  }

  setInterval(check, CHECK_INTERVAL);
  document.addEventListener('visibilitychange', function () {
    if (!document.hidden) check();
  });
  check();
})();
