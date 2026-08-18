import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const { cacheControlForPath } = require('../scripts/static-cache-policy.cjs');

for (const path of ['/src/main.js', '/src/error-report.js', '/api/path/demo.json']) {
    assert.equal(cacheControlForPath(path), 'no-store', `${path} must not stay stale`);
}

for (const path of [
    '/ThirdParty/Cesium/Cesium.js',
    '/ThirdParty/Cesium/Widgets/widgets.css',
    '/asset/vendor/playcanvas.min.js',
]) {
    assert.equal(
        cacheControlForPath(path),
        'public, max-age=86400',
        `${path} may use the vendor cache policy`,
    );
}

for (const path of ['/', '/index.html', '/asset/config/controller-defaults.json', '', null]) {
    assert.equal(cacheControlForPath(path), 'no-cache', `${String(path)} must revalidate`);
}

console.log('static cache policy tests passed');
