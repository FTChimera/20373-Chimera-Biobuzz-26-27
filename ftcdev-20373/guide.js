document.addEventListener('DOMContentLoaded', () => {
    const links = [...document.querySelectorAll('.toc a[href^="#"]')];
    const targets = links.map(link => document.querySelector(link.getAttribute('href'))).filter(Boolean);
    const observer = new IntersectionObserver(entries => {
        const visible = entries.filter(entry => entry.isIntersecting).sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0];
        if (!visible) return;
        links.forEach(link => link.classList.toggle('active', link.getAttribute('href') === `#${visible.target.id}`));
    }, { rootMargin: '-20% 0px -65% 0px', threshold: [0, .3, .7] });
    targets.forEach(target => observer.observe(target));
});
