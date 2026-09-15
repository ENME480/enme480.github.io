/* Reloads a tab whose page has been superseded by a newer build.
 *
 * GitHub Pages caches HTML for ten minutes, so a browser that is closed and
 * reopened already gets fresh content. The case this covers is a tab left open
 * for hours or days, which is how students use the lab pages, and which
 * navigation.instant keeps alive across page swaps.
 *
 * The site is not deployed during lab, so reloading without asking is safe.
 */
(function () {
  // navigation.instant re-runs this on every page swap; only set up once.
  if (window.__enme480Refresh) return;
  window.__enme480Refresh = true;

  var meta = document.querySelector('meta[name="site-build"]');
  if (!meta || !meta.content) return;

  var loaded = meta.content;
  var CHECK_INTERVAL = 5 * 60 * 1000;
  // Longer than the ten minute HTML cache, so a retry is guaranteed to be able
  // to fetch the new page rather than the same stale one.
  var QUIET_AFTER_RELOAD = 11 * 60 * 1000;
  var MAX_RELOADS = 3;
  var STORE_KEY = 'enme480-reloads';
  var busy = false;

  function reloadCount() {
    try { return parseInt(sessionStorage.getItem(STORE_KEY), 10) || 0; }
    catch (e) { return 0; }
  }

  function noteReload() {
    try { sessionStorage.setItem(STORE_KEY, String(reloadCount() + 1)); }
    catch (e) { /* private mode: the timing guard below still bounds us */ }
  }

  function clearCount() {
    try { sessionStorage.removeItem(STORE_KEY); } catch (e) {}
  }

  // If this page load was itself a reload and the page is still young, the new
  // HTML has not reached us yet. Reloading again would just spin.
  function reloadedRecently() {
    var nav = (performance.getEntriesByType('navigation') || [])[0];
    return nav && nav.type === 'reload' && performance.now() < QUIET_AFTER_RELOAD;
  }

  // Do not yank the page out from under someone mid-sentence in the search box.
  function isTyping() {
    var el = document.activeElement;
    if (!el) return false;
    var tag = (el.tagName || '').toLowerCase();
    if (el.isContentEditable) return true;
    return (tag === 'input' || tag === 'textarea') && !!el.value;
  }

  function check() {
    if (busy) return;
    busy = true;

    fetch('/version.txt?t=' + Date.now(), { cache: 'no-store' })
      .then(function (r) { return r.ok ? r.text() : null; })
      .then(function (body) {
        busy = false;
        if (!body) return;

        var latest = body.trim();
        if (!latest || latest === loaded) {
          clearCount();          // we are current
          return;
        }
        if (reloadCount() >= MAX_RELOADS) return;   // something is wrong, stop
        if (reloadedRecently()) return;             // wait out the HTML cache
        if (isTyping()) return;                     // retry on the next tick

        noteReload();
        window.location.reload();
      })
      .catch(function () { busy = false; });
  }

  setInterval(check, CHECK_INTERVAL);

  // A hidden tab is the best possible moment to reload: nobody sees it happen.
  document.addEventListener('visibilitychange', function () {
    if (!document.hidden) check();
  });

  check();
})();
