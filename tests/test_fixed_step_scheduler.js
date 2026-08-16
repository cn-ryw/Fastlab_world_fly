import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

import {
    CONTROL_DT,
    FixedStepScheduler,
    MAX_CONTROL_CATCH_UP_SECONDS,
    MAX_CONTROL_STEPS_PER_FRAME,
} from '../src/fixed-step-scheduler.js';

assert.equal(CONTROL_DT, 0.005, 'control cadence must remain 200 Hz');
assert.equal(MAX_CONTROL_STEPS_PER_FRAME, 20, 'one render may execute at most 20 control steps');
assert.equal(MAX_CONTROL_CATCH_UP_SECONDS, 0.1, 'one render may accept at most 100 ms');

const mainSource = await readFile(new URL('../src/main.js', import.meta.url), 'utf8');
const updateFlightSource = mainSource.slice(
    mainSource.indexOf('function updateFlight(dt)'),
    mainSource.indexOf('function applyDisplaySettings()'),
);
assert.equal(updateFlightSource.match(/controller\.update\(\)/g)?.length, 1,
    'the render loop must poll controller input exactly once');
assert.match(updateFlightSource,
    /flightControlScheduler\.advance\([\s\S]*?drone\.update\(stepDt, input, collisionProvider, frameCollisionCadence\)/,
    'every fixed step must reuse the input snapshot and frame collision cadence');
assert.equal(updateFlightSource.match(/panoramaSensor\?\.update\(/g)?.length, 1,
    'panorama capture must remain a once-per-render operation');
assert.ok(
    updateFlightSource.indexOf('panoramaSensor?.update(') > updateFlightSource.indexOf('flightControlScheduler.advance('),
    'perception must run after, and outside, the fixed control steps',
);

function runAtFps(fps, durationSeconds = 10) {
    const scheduler = new FixedStepScheduler();
    const input = Object.freeze({ forward: 0.75 });
    let inputPolls = 0;
    let steps = 0;
    let state = 0;
    for (let frame = 0; frame < fps * durationSeconds; frame++) {
        inputPolls += 1;
        scheduler.advance(1 / fps, (dt) => {
            assert.equal(input.forward, 0.75, 'all substeps reuse the render frame input snapshot');
            state += input.forward * dt;
            steps += 1;
        });
    }
    return { scheduler, inputPolls, steps, state };
}

const at30 = runAtFps(30);
const at60 = runAtFps(60);
const at120 = runAtFps(120);
for (const result of [at30, at60, at120]) {
    assert.equal(result.steps, 2000, '10 seconds must execute exactly 2000 fixed steps');
    assert.ok(Math.abs(result.scheduler.simulatedSeconds - 10) < 1e-10);
    assert.ok(Math.abs(result.state - 7.5) < 1e-10);
    assert.equal(result.scheduler.totalDroppedSeconds, 0);
}
assert.equal(at30.inputPolls, 300);
assert.equal(at60.inputPolls, 600);
assert.equal(at120.inputPolls, 1200);
assert.ok(Math.abs(at30.state - at60.state) < 1e-12, '30/60 FPS end state must match');
assert.ok(Math.abs(at60.state - at120.state) < 1e-12, '60/120 FPS end state must match');

{
    const scheduler = new FixedStepScheduler();
    const events = [];
    const result = scheduler.advance(0.35, () => events.push('step'), {
        onOverrun: (overrun) => {
            events.push('overrun');
            assert.ok(Math.abs(overrun.droppedSeconds - 0.25) < 1e-12);
            assert.equal(overrun.simulatedSeconds, 0);
            assert.equal(overrun.maxStepsPerFrame, 20);
        },
    });
    assert.equal(events[0], 'overrun', 'overrun must invalidate stale control before catch-up steps');
    assert.equal(result.steps, 20, 'a long frame may execute at most 20 steps');
    assert.ok(Math.abs(result.simulatedThisFrameSeconds - 0.1) < 1e-12);
    assert.ok(Math.abs(result.droppedSeconds - 0.25) < 1e-12);
    assert.equal(result.accumulatorSeconds, 0, 'discarded wall time must not remain for later catch-up');
    assert.equal(scheduler.advance(0, () => events.push('unexpected')).steps, 0);
}

{
    const scheduler = new FixedStepScheduler();
    assert.equal(scheduler.advance(0.004, () => {}).steps, 0);
    scheduler.reset();
    assert.equal(scheduler.advance(0.001, () => {}).steps, 0,
        'reset must remove fractional time from the previous flight/reset session');
    assert.equal(scheduler.accumulatorSeconds, 0.001);
    assert.equal(scheduler.simulatedSeconds, 0);
    assert.equal(scheduler.totalDroppedSeconds, 0);
}

{
    const scheduler = new FixedStepScheduler();
    let steps = 0;
    scheduler.advance(Number.NaN, () => { steps += 1; });
    scheduler.advance(-1, () => { steps += 1; });
    assert.equal(steps, 0, 'invalid and negative frame deltas are ignored');
}

console.log('Fixed-step scheduler: all tests passed');
