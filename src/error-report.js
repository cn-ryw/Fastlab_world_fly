const lastShownByKey = new Map();

export function formatError(error) {
    if (error && error.message) return error.message;
    return String(error || 'unknown error');
}

export function reportUserError(context, error, options = {}) {
    const message = formatError(error);
    const title = context ? `${context}: ${message}` : message;
    const key = options.key || title;
    const intervalMs = Number.isFinite(options.intervalMs) ? options.intervalMs : 3000;
    const now = performance.now ? performance.now() : Date.now();
    const last = lastShownByKey.get(key) || 0;

    console.error(`[${context || 'Error'}]`, error);
    if (intervalMs > 0 && now - last < intervalMs) return;
    lastShownByKey.set(key, now);

    const banner = ensureErrorBanner();
    banner.textContent = title;
    banner.style.display = 'block';

    if (options.overlay) {
        const overlay = document.getElementById('loading-overlay');
        const progress = document.getElementById('loading-progress');
        if (overlay) overlay.classList.add('visible');
        if (progress) {
            progress.textContent = title;
            progress.style.color = '#f44';
        }
    }
}

function ensureErrorBanner() {
    let banner = document.getElementById('runtime-error-banner');
    if (banner) return banner;

    banner = document.createElement('div');
    banner.id = 'runtime-error-banner';
    banner.setAttribute('role', 'alert');
    banner.style.cssText = [
        'position:fixed',
        'top:12px',
        'left:50%',
        'transform:translateX(-50%)',
        'max-width:min(720px,calc(100vw - 32px))',
        'z-index:30000',
        'display:none',
        'background:rgba(127,29,29,0.96)',
        'color:#fee2e2',
        'border:1px solid rgba(248,113,113,0.9)',
        'border-radius:6px',
        'padding:10px 14px',
        'font:12px/1.45 Courier New,monospace',
        'box-shadow:0 10px 26px rgba(0,0,0,0.45)',
        'white-space:normal',
        'pointer-events:none',
    ].join(';');
    document.body.appendChild(banner);
    return banner;
}
