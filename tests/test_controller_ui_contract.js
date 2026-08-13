/** Static controller UI contract for the four public flight modes. */
import fs from 'node:fs';

const html = fs.readFileSync(new URL('../index.html', import.meta.url), 'utf8');
const controller = fs.readFileSync(new URL('../src/controller.js', import.meta.url), 'utf8');

let passed = 0;
let failed = 0;
function assert(condition, message) {
    if (condition) { passed++; return; }
    failed++;
    console.error(`FAIL: ${message}`);
}

for (const mode of ['drone', 'fpv', 'stabilized', 'so3']) {
    assert(html.includes(`option value="${mode}"`), `${mode} remains a public mode ID`);
}
assert(html.includes('SO3 (YOPO Auto)'), 'SO3 is labelled as YOPO Auto');
assert(html.includes('Level (Self-Level)'), 'Level is labelled as self-level attitude mode');
assert(html.includes('value="yup" selected'), 'simulation coordinate display is Y-Up');
assert(!html.includes('value="zup" selected'), 'UI no longer claims Z-Up');
assert(html.includes('manual thrust—there is no altitude hold'), 'Level manual-thrust semantics are explained');
assert(html.includes('connected RC transmitter continuously moves a 50 m rolling goal'),
    'SO3 T8L rolling-goal semantics are explained');

for (const obsoleteId of ['so3-kx', 'so3-kv', 'so3-kr', 'so3-komega']) {
    assert(!html.includes(`id="${obsoleteId}"`), `${obsoleteId} scalar control is absent`);
}
assert(html.includes('XY 5.7') && html.includes('Y-Up 6.2'), 'authority position gains are displayed by axis');
assert(html.includes('XY 3.4') && html.includes('Y-Up 4.0'), 'authority velocity gains are displayed by axis');

const easyDefaults = {
    'ctrl-pos-kp': '0.95', 'ctrl-pos-ki': '0', 'ctrl-pos-kd': '0',
    'ctrl-vel-kp': '1.8', 'ctrl-vel-ki': '0.4', 'ctrl-vel-kd': '0.2',
    'ctrl-alt-kp': '4.0', 'ctrl-alt-ki': '2.0', 'ctrl-alt-kd': '0',
};
for (const [id, value] of Object.entries(easyDefaults)) {
    const pattern = new RegExp(`id="${id}"[^>]*value="${value.replace('.', '\\.')}"`);
    assert(pattern.test(html), `${id} uses the v6 Easy baseline ${value}`);
}

assert(controller.includes('const CONFIG_VERSION = 6;'), 'controller persists config v6');
assert(controller.includes("['drone', 'fpv', 'stabilized', 'so3']"), 'controller validates the four-mode enum');

console.log(`\nController UI contract: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
