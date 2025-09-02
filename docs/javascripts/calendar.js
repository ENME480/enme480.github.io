// docs/javascripts/calendar.js
(function () {
  function initCalendar() {
    const el = document.getElementById('enme480-calendar');
    if (!el || typeof FullCalendar === 'undefined') return;

    // controls
    const filters = {
      lecture: document.getElementById('filter-lecture'),
      studio:  document.getElementById('filter-studio'),
      lab:     document.getElementById('filter-lab'),
      "0101":  document.getElementById('filter-0101'),
      "0102":  document.getElementById('filter-0102'),
      "0103":  document.getElementById('filter-0103'),
    };

    let allEvents = [];

    fetch(new URL('../calendar/events.json', window.location).href)
      .then(r => r.json())
      .then(json => {
        allEvents = json;
        render();
      });

    function filtered() {
      return allEvents.filter(e => {
        const typeOk = filters[e.type]?.checked ?? true;
        const sec = e.section || 'lecture';
        const secOk = (e.type === 'lecture') ? true : !!filters[sec]?.checked;
        return typeOk && secOk;
      });
    }

    let calendar;
    function render() {
      if (calendar) calendar.destroy();

      calendar = new FullCalendar.Calendar(el, {
        initialView: 'dayGridMonth',
        height: 'auto',
        headerToolbar: {
          left: 'prev,next today',
          center: 'title',
          right: 'dayGridMonth,listWeek'
        },
        events: filtered(),
        eventClassNames: function(arg) {
          const t = arg.event.extendedProps.type;
          const sec = arg.event.extendedProps.section;
          const classes = [];
          if (t === 'lab') classes.push('lab-event');
          if (t === 'studio') classes.push('studio-event');
          if (t === 'lecture') classes.push('lecture-event');
          if (sec) classes.push('sec-' + sec);
          return classes;
        },
        eventContent: function(arg) {
          // Bold labs; include location on 2nd line
          const title = arg.event.title || '';
          const loc = arg.event.extendedProps.location || '';
          const isLab = arg.event.extendedProps.type === 'lab';
          const inner = `
            <div class="fc-title-line ${isLab ? 'fc-lab-strong' : ''}">
              ${title}
            </div>
            <div class="fc-sub">${loc}</div>`;
          return { html: inner };
        }
      });

      calendar.render();
    }

    Object.values(filters).forEach(cb => cb?.addEventListener('change', render));
  }

  // Works with Material's instant loading
  (window.document$ || { subscribe: (fn) => window.addEventListener('DOMContentLoaded', fn) })
    .subscribe(initCalendar);
})();
