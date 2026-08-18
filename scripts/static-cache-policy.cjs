'use strict';

/**
 * Keep first-party runtime modules fresh while allowing immutable vendor
 * bundles to use the same one-day cache policy as the Python development
 * server. The function is separate from Express so CI can execute the exact
 * mapping used by the container image.
 */
function cacheControlForPath(requestPath) {
    const path = typeof requestPath === 'string' ? requestPath : '';
    if (path.startsWith('/src/') || path.startsWith('/api/')) {
        return 'no-store';
    }
    if (path.startsWith('/ThirdParty/') || path.startsWith('/asset/vendor/')) {
        return 'public, max-age=86400';
    }
    return 'no-cache';
}

module.exports = { cacheControlForPath };
