#!/usr/bin/env node

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const root = path.resolve(__dirname, '..');
const source = fs.readFileSync(path.join(root, 'src', 'cesium-world.js'), 'utf8');
const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');

assert(source.includes('window.MINDCLOUD_RUNTIME_CONFIG?.cesiumIonToken'));
assert(source.includes('CESIUM_ION_TOKEN'));
assert(!source.includes("urlString('ionToken'"));
assert(!source.includes('DEFAULT_ION_TOKEN'));
assert(html.includes('<script src="/runtime-config.js"></script>'));

console.log('Cesium runtime configuration contract passed');
