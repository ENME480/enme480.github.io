// docs/javascripts/tablesort.js
document$.subscribe(function() {
  var tables = document.querySelectorAll("article table:not([class])");
  tables.forEach(function(t) { new Tablesort(t); });
});
  