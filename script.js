const menuButton = document.querySelector('.menu-button');
const mobileMenu = document.querySelector('.mobile-menu');

menuButton?.addEventListener('click', () => {
  const open = menuButton.getAttribute('aria-expanded') === 'true';
  menuButton.setAttribute('aria-expanded', String(!open));
  menuButton.setAttribute('aria-label', open ? 'Open menu' : 'Close menu');
  mobileMenu.classList.toggle('open', !open);
});

mobileMenu?.querySelectorAll('a').forEach((link) => link.addEventListener('click', () => {
  menuButton.setAttribute('aria-expanded', 'false');
  menuButton.setAttribute('aria-label', 'Open menu');
  mobileMenu.classList.remove('open');
}));

const header = document.querySelector('.site-header');
const headerTop = header.offsetTop;
window.addEventListener('scroll', () => {
  const stuck = window.scrollY > headerTop + 140;
  header.classList.toggle('stuck', stuck);
  document.body.style.paddingTop = stuck ? `${header.offsetHeight}px` : '0';
}, { passive: true });

const filters = document.querySelectorAll('.filter');
const menuCards = document.querySelectorAll('.menu-card');
filters.forEach((button) => button.addEventListener('click', () => {
  filters.forEach((item) => item.classList.remove('active'));
  button.classList.add('active');
  const filter = button.dataset.filter;
  menuCards.forEach((card) => {
    card.classList.toggle('hidden', filter !== 'all' && card.dataset.category !== filter);
  });
}));

const revealObserver = new IntersectionObserver((entries) => {
  entries.forEach((entry) => {
    if (entry.isIntersecting) {
      entry.target.classList.add('visible');
      revealObserver.unobserve(entry.target);
    }
  });
}, { threshold: 0.12 });
document.querySelectorAll('.reveal').forEach((el) => revealObserver.observe(el));

function getPacificTime() {
  const values = new Intl.DateTimeFormat('en-US', {
    timeZone: 'America/Los_Angeles', hour: 'numeric', hour12: false, weekday: 'short'
  }).formatToParts(new Date());
  return Object.fromEntries(values.map(({ type, value }) => [type, value]));
}

function updateOpenStatus() {
  const { hour, weekday } = getPacificTime();
  const currentHour = Number(hour === '24' ? 0 : hour);
  document.querySelectorAll('.location-card').forEach((card) => {
    let open = true;
    if (card.dataset.location === 'baker') {
      const weekend = weekday === 'Sat' || weekday === 'Sun';
      open = currentHour >= (weekend ? 7 : 6) && currentHour < 21;
    }
    if (card.dataset.location === 'mill-valley') open = currentHour >= 6 && currentHour < 18;
    const status = card.querySelector('.location-status');
    if (card.dataset.location !== 'polk') card.querySelector('[data-status]').textContent = open ? 'Open now' : 'Closed now';
    status.classList.toggle('closed', !open);
  });
}

updateOpenStatus();
document.querySelector('#year').textContent = new Date().getFullYear();
