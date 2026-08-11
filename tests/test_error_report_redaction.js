/** Runtime errors must never expose URL credentials, queries or fragments. */

globalThis.window = { location: { search: '' } };
const banner = {
    textContent: '',
    style: {},
    setAttribute() {},
};
globalThis.document = {
    body: { appendChild() {} },
    getElementById(id) { return id === 'runtime-error-banner' ? banner : null; },
    createElement() { return banner; },
};

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

const originalConsoleError = console.error;
const consoleLines = [];
console.error = (...values) => consoleLines.push(values.map(String).join(' '));
try {
    reportUserError('Tile request failed https://admin:pw@context.example.test/root?key=context-secret#frag', {
        message: 'Request has failed.',
        url: 'https://user:pass@tile.example.test/content.glb?key=raw-object-secret',
    }, { intervalMs: 0 });
} finally {
    console.error = originalConsoleError;
}
const consoleText = consoleLines.join('\n');
if (consoleText.includes('raw-object-secret')
    || consoleText.includes('user:pass')
    || consoleText.includes('context-secret')
    || consoleText.includes('admin:pw')
    || banner.textContent.includes('context-secret')
    || banner.textContent.includes('admin:pw')) {
    throw new Error(`reportUserError logged the raw error object: ${consoleText}`);
}

console.log('Error URL redaction tests: all passed');
