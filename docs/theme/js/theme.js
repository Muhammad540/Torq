document.addEventListener('DOMContentLoaded', function () {
  var toggle = document.getElementById('sidebar-toggle');
  var sidebar = document.getElementById('sidebar');

  if (toggle && sidebar) {
    toggle.addEventListener('click', function () {
      sidebar.classList.toggle('open');
    });

    document.addEventListener('click', function (e) {
      if (sidebar.classList.contains('open') &&
          !sidebar.contains(e.target) &&
          !toggle.contains(e.target)) {
        sidebar.classList.remove('open');
      }
    });
  }

  var tocLinks = document.querySelectorAll('.toc-link');
  if (tocLinks.length) {
    var headings = [];
    tocLinks.forEach(function (link) {
      var id = link.getAttribute('href');
      if (id && id.startsWith('#')) {
        var el = document.getElementById(id.slice(1));
        if (el) headings.push({ el: el, link: link });
      }
    });

    function updateActive() {
      var scrollY = window.scrollY + 80;
      var active = null;
      for (var i = headings.length - 1; i >= 0; i--) {
        if (headings[i].el.offsetTop <= scrollY) {
          active = headings[i];
          break;
        }
      }
      tocLinks.forEach(function (l) { l.style.borderLeftColor = 'transparent'; l.style.color = ''; });
      if (active) {
        active.link.style.borderLeftColor = '#2383e2';
        active.link.style.color = '#37352f';
      }
    }

    window.addEventListener('scroll', updateActive, { passive: true });
    updateActive();
  }
});
