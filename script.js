const menuToggle = document.querySelector(".menu-toggle");
const siteNav = document.querySelector("#site-nav");

menuToggle?.addEventListener("click", () => {
  const isOpen = menuToggle.getAttribute("aria-expanded") === "true";
  menuToggle.setAttribute("aria-expanded", String(!isOpen));
  siteNav.classList.toggle("open", !isOpen);
});

siteNav?.querySelectorAll("a").forEach((link) => {
  link.addEventListener("click", () => {
    menuToggle?.setAttribute("aria-expanded", "false");
    siteNav.classList.remove("open");
  });
});

document.querySelectorAll("details").forEach((detail) => {
  detail.addEventListener("toggle", () => {
    if (!detail.open) return;
    const group = detail.closest(".travel-list, .faq-list");
    group?.querySelectorAll("details").forEach((sibling) => {
      if (sibling !== detail) sibling.open = false;
    });
  });
});

const countdown = document.querySelector(".countdown");
if (countdown) {
  const target = new Date(countdown.dataset.date).getTime();

  const renderCountdown = () => {
    const distance = Math.max(0, target - Date.now());
    const values = {
      days: Math.floor(distance / 86_400_000),
      hours: Math.floor((distance / 3_600_000) % 24),
      minutes: Math.floor((distance / 60_000) % 60),
      seconds: Math.floor((distance / 1_000) % 60),
    };

    Object.entries(values).forEach(([unit, value]) => {
      const node = countdown.querySelector(`[data-unit="${unit}"]`);
      if (node) node.textContent = String(value).padStart(unit === "days" ? 3 : 2, "0");
    });
  };

  renderCountdown();
  setInterval(renderCountdown, 1000);
}

const rsvpForm = document.querySelector(".rsvp-form");
rsvpForm?.addEventListener("submit", async (event) => {
  event.preventDefault();
  const status = rsvpForm.querySelector(".form-status");
  const submit = rsvpForm.querySelector("button[type='submit']");
  submit.disabled = true;
  status.textContent = "Sending your RSVP…";

  try {
    const response = await fetch(rsvpForm.action, {
      method: "POST",
      body: new FormData(rsvpForm),
    });

    if (!response.ok) throw new Error("Unable to submit");
    const result = await response.json();
    status.textContent = result.message || "You’re on the list. Thank you!";
    rsvpForm.reset();
  } catch {
    status.textContent = "We couldn’t send that just now. Please try again in a moment.";
  } finally {
    submit.disabled = false;
  }
});
