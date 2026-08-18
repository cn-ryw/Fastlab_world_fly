/** Runtime errors must never expose URL credentials, queries or fragments. */

globalThis.window = { location: { search: '' } };
const banner = {
    textContent: '',
    style: {},
    setAttribute() {},
};
const overlayClasses = new Set();
const overlay = {
    classList: {
        add(value) { overlayClasses.add(value); },
    },
};
const progress = {
    textContent: '',
    style: {},
};
globalThis.document = {
    body: { appendChild() {} },
    getElementById(id) {
        if (id === 'runtime-error-banner') return banner;
        if (id === 'loading-overlay') return overlay;
        if (id === 'loading-progress') return progress;
        return null;
    },
    createElement() { return banner; },
};

const originalSetTimeout = globalThis.setTimeout;
const originalClearTimeout = globalThis.clearTimeout;
const timers = [];
let nextTimerId = 1;

globalThis.setTimeout = (callback, delay, ...args) => {
    const timer = {
        id: nextTimerId++,
        delay,
        callback: () => callback(...args),
        cleared: false,
        ran: false,
    };
    timers.push(timer);
    return timer.id;
};
globalThis.clearTimeout = (id) => {
    const timer = timers.find((candidate) => candidate.id === id);
    if (timer) timer.cleared = true;
};

function runTimer(timer, { force = false } = {}) {
    if (!timer || timer.ran || (timer.cleared && !force)) return;
    timer.ran = true;
    timer.callback();
}

function assert(condition, message) {
    if (!condition) throw new Error(message);
}

function captureReport(reportUserError, context, error, options) {
    const originalConsoleError = console.error;
    const lines = [];
    console.error = (...values) => lines.push(values.map(String).join(' '));
    try {
        reportUserError(context, error, options);
    } finally {
        console.error = originalConsoleError;
    }
    return lines;
}

const { formatError, redactSensitiveUrls, reportUserError } = await import('../src/error-report.js?redaction-test');

const source = 'request failed: https://alice:secret@example.test/a/b.json?key=private&session=hidden#trace.';
const redacted = redactSensitiveUrls(source);
if (redacted.includes('alice')
    || redacted.includes('secret')
    || redacted.includes('private')
    || redacted.includes('hidden')
    || redacted.includes('?')
    || redacted.includes('#')) {
    throw new Error(`sensitive URL material survived redaction: ${redacted}`);
}
if (!redacted.includes('https://example.test/a/b.json')) {
    throw new Error(`diagnostic host/path was removed: ${redacted}`);
}

const formatted = formatError(new Error(
    'tile https://tile.example.test/content.glb?key=do-not-log&session=also-hidden failed',
));
if (formatted.includes('do-not-log') || formatted.includes('also-hidden') || formatted.includes('?')) {
    throw new Error(`formatError leaked a query: ${formatted}`);
}

const consoleLines = captureReport(
    reportUserError,
    'Tile request failed https://admin:pw@context.example.test/root?key=context-secret#frag',
    {
        message: 'Request has failed.',
        url: 'https://user:pass@tile.example.test/content.glb?key=raw-object-secret',
    },
    { intervalMs: 0 },
);
const consoleText = consoleLines.join('\n');
if (consoleText.includes('raw-object-secret')
    || consoleText.includes('user:pass')
    || consoleText.includes('context-secret')
    || consoleText.includes('admin:pw')
    || banner.textContent.includes('context-secret')
    || banner.textContent.includes('admin:pw')) {
    throw new Error(`reportUserError logged the raw error object: ${consoleText}`);
}

captureReport(reportUserError, 'Timed error', new Error('hide me'), { intervalMs: 0 });
const timedHide = timers.at(-1);
assert(timedHide.delay === 5000, `error banner timeout was ${timedHide.delay}ms instead of 5000ms`);
assert(banner.style.display === 'block', 'new error banner was not displayed');
runTimer(timedHide);
assert(banner.style.display === 'none', 'error banner did not hide after its timeout');
assert(banner.textContent === '', 'hidden error banner retained stale text');

captureReport(reportUserError, 'First error', new Error('old message'), { intervalMs: 0 });
const staleHide = timers.at(-1);
captureReport(reportUserError, 'Second error', new Error('new message'), { intervalMs: 0 });
const currentHide = timers.at(-1);
assert(staleHide.cleared, 'showing a new error did not clear the previous hide timer');
runTimer(staleHide, { force: true });
assert(banner.style.display === 'block', 'a stale hide callback hid the current error banner');
assert(banner.textContent === 'Second error: new message', 'a stale hide callback cleared the current error');
runTimer(currentHide);
assert(banner.style.display === 'none', 'the replacement error did not hide after its timeout');
assert(banner.textContent === '', 'the replacement error text was not cleared on hide');

captureReport(reportUserError, 'Throttled error', new Error('same message'), {
    key: 'throttled-error-test',
    intervalMs: 60000,
});
const throttledHide = timers.at(-1);
const timerCountBeforeThrottle = timers.length;
captureReport(reportUserError, 'Throttled error', new Error('same message'), {
    key: 'throttled-error-test',
    intervalMs: 60000,
});
assert(timers.length === timerCountBeforeThrottle, 'a throttled duplicate extended the banner timeout');
assert(!throttledHide.cleared, 'a throttled duplicate cleared the active banner timeout');
runTimer(throttledHide);

captureReport(reportUserError, 'Startup failed', new Error('cannot initialize'), {
    overlay: true,
    intervalMs: 0,
});
const overlayBannerHide = timers.at(-1);
const overlayMessage = progress.textContent;
runTimer(overlayBannerHide);
assert(banner.style.display === 'none' && banner.textContent === '', 'overlay error banner did not auto-hide');
assert(overlayClasses.has('visible'), 'auto-hiding the banner also hid the loading overlay');
assert(progress.textContent === overlayMessage, 'auto-hiding the banner cleared the loading overlay message');
assert(progress.style.color === '#f44', 'overlay error styling was not preserved');

globalThis.setTimeout = originalSetTimeout;
globalThis.clearTimeout = originalClearTimeout;

console.log('Error report tests: all passed');
