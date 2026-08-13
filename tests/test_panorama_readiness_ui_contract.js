import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const main = readFileSync(new URL('../src/main.js', import.meta.url), 'utf8');
const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8');

assert.match(html, /id="panorama-rgb-readiness"/);
assert.match(html, /RGB LOADING 0\/6/);
assert.match(main, /RGB READY \$\{capturedFaces\}\/\$\{totalFaces\}/);
assert.match(main, /RGB CAPTURING \$\{capturedFaces\}\/\$\{totalFaces\}/);
assert.doesNotMatch(main, /AUTO UNVERIFIED|TILE ERROR|SETTLED/);

// A structured panorama can have a projected canvas while one or more 3D-tile
// faces are still incomplete. Explicit strict preload must require both facts.
assert.match(
    main,
    /result\?\.ready === true && result\?\.allFacesTileReady === true/,
);
assert.match(main, /if \(!framePrimed \|\| \(strictPreload && !sceneTilesReady\)\)/);
assert.match(main, /if \(strictPreload\) throw new Error\(message\)/);
assert.match(main, /signal: preloadController\.signal/);
assert.match(main, /preloadController\.abort\('panorama-preload-finished-with-error'\)/);

// Readiness is diagnostic in the current experimental mode: the UI consumes
// it, but goal creation and YOPO trajectory application remain ungated.
const beginNavigation = main.slice(
    main.indexOf('function beginNavigationSession'),
    main.indexOf('async function enterPlacementMode'),
);
assert.doesNotMatch(beginNavigation, /rgbTilesReady|rgbReadyFaces/);
const yopoApply = main.slice(
    main.indexOf('panoramaSensor.onYopoResult'),
    main.indexOf('// Setup click-to-goal'),
);
assert.doesNotMatch(yopoApply, /rgbTilesReady|rgbReadyFaces/);
assert.match(yopoApply, /context\?\.commitIfFresh/);
assert.match(yopoApply, /setYopoTrajectory\(endstate, trajTime, context\)/);
assert.match(
    yopoApply,
    /invalidateYopoTrajectory\?\.\([\s\S]*trajectory-apply-deadline-exceeded[\s\S]*context/,
);

console.log('Panorama readiness UI contract: all assertions passed');
