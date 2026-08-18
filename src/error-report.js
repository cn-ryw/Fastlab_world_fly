const lastShownByKey = new Map();
const ERROR_BANNER_DISPLAY_MS = 5000;

let errorBannerHideTimer = null;
let errorBannerDisplayVersion = 0;

const URL_CANDIDATE_RE = /https?:\/\/[^\s<>"'`]+/giu;

/**
 * Keep URLs useful for diagnostics without ever exposing credentials or
 * request parameters. Google 3D Tiles puts its session and API key in the
 * query string, and Firefox includes the complete URL in network errors.
 */
export function redactSensitiveUrls(value) {
    return String(value ?? '').replace(URL_CANDIDATE_RE, (rawMatch) => {
        let candidate = rawMatch;
        let suffix = '';
        // Sentence punctuation is not part of the URL. Strip it before URL
        // parsing, then retain it in the human-readable error text.
        while (/[),.;!?\]}]$/.test(candidate)) {
            suffix = candidate.slice(-1) + suffix;
            candidate = candidate.slice(0, -1);
        }
        try {
            const url = new URL(candidate);
            url.username = '';
            url.password = '';
            url.search = '';
            url.hash = '';
            return `${url.toString()}${suffix}`;
        } catch (_) {
            // A malformed URL is still potentially sensitive. Keep only the
            // scheme/authority/path-shaped prefix and discard userinfo/query.
            const withoutFragment = candidate.split('#', 1)[0];
            const withoutQuery = withoutFragment.split('?', 1)[0];
            const schemeEnd = withoutQuery.indexOf('://');
            if (schemeEnd < 0) return `<redacted-url>${suffix}`;
            const scheme = withoutQuery.slice(0, schemeEnd + 3);
            const remainder = withoutQuery.slice(schemeEnd + 3);
            const slash = remainder.indexOf('/');
            const authority = slash >= 0 ? remainder.slice(0, slash) : remainder;
            const path = slash >= 0 ? remainder.slice(slash) : '';
            return `${scheme}${authority.split('@').pop()}${path}${suffix}`;
        }
    });
}

export function formatError(error) {
    const message = error && error.message ? error.message : String(error || 'unknown error');
    return redactSensitiveUrls(message);
}

export function reportUserError(context, error, options = {}) {
    const message = formatError(error);
    const safeContext = redactSensitiveUrls(context || '');
    const title = safeContext ? `${safeContext}: ${message}` : message;
    const key = redactSensitiveUrls(options.key || title);
    const intervalMs = Number.isFinite(options.intervalMs) ? options.intervalMs : 3000;
    const now = performance.now ? performance.now() : Date.now();
    const last = lastShownByKey.get(key);

    // Throttle before writing to the console. Logging the raw Cesium error on
    // every failed tile both leaked its query string and amplified a flaky
    // network connection into a Firefox main-thread log storm.
    if (intervalMs > 0 && last !== undefined && now - last < intervalMs) return;
    lastShownByKey.set(key, now);
    console.error(`[${safeContext || 'Error'}] ${message}`);

    const banner = ensureErrorBanner();
    banner.textContent = title;
    banner.style.display = 'block';
    scheduleErrorBannerHide(banner);

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

function scheduleErrorBannerHide(banner) {
    const displayVersion = ++errorBannerDisplayVersion;
    if (errorBannerHideTimer !== null) {
        clearTimeout(errorBannerHideTimer);
    }

    errorBannerHideTimer = setTimeout(() => {
        // clearTimeout prevents normal stale callbacks, while the version
        // check also covers a callback that was already queued when cleared.
        if (displayVersion !== errorBannerDisplayVersion) return;
        banner.textContent = '';
        banner.style.display = 'none';
        errorBannerHideTimer = null;
    }, ERROR_BANNER_DISPLAY_MS);
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
