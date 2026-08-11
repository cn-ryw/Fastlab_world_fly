/*
 * Copyright 2026 Manifold Tech Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

/** Authoritative browser control cadence: 200 Hz. */
export const CONTROL_DT = 0.005;
export const MAX_CONTROL_STEPS_PER_FRAME = 20;
export const MAX_CONTROL_CATCH_UP_SECONDS = CONTROL_DT * MAX_CONTROL_STEPS_PER_FRAME;

const ROUNDING_EPSILON = 1e-12;

function positiveFinite(value, label) {
    if (!Number.isFinite(value) || value <= 0) {
        throw new RangeError(`${label} must be a positive finite number`);
    }
    return value;
}

/**
 * Fixed-step accumulator independent of requestAnimationFrame and the DOM.
 *
 * Wall time beyond maxCatchUpSeconds is deliberately discarded instead of
 * being left in the accumulator. Consequently simulation/trajectory time can
 * slow down after a stalled render frame, but can never race forward later to
 * catch the wall clock.
 */
export class FixedStepScheduler {
    constructor({
        stepSeconds = CONTROL_DT,
        maxStepsPerFrame = MAX_CONTROL_STEPS_PER_FRAME,
        maxCatchUpSeconds = MAX_CONTROL_CATCH_UP_SECONDS,
    } = {}) {
        this.stepSeconds = positiveFinite(stepSeconds, 'stepSeconds');
        this.maxStepsPerFrame = Math.trunc(positiveFinite(maxStepsPerFrame, 'maxStepsPerFrame'));
        this.maxCatchUpSeconds = positiveFinite(maxCatchUpSeconds, 'maxCatchUpSeconds');
        if (this.maxStepsPerFrame * this.stepSeconds + ROUNDING_EPSILON < this.maxCatchUpSeconds) {
            throw new RangeError('maxStepsPerFrame must cover maxCatchUpSeconds');
        }
        this.reset();
    }

    reset() {
        this.accumulatorSeconds = 0;
        this.simulatedSeconds = 0;
        this.totalDroppedSeconds = 0;
        this.frameCount = 0;
    }

    /**
     * Advance one render frame.
     *
     * onOverrun runs before the first fixed step so a vehicle can invalidate a
     * stale trajectory before any of the accepted catch-up interval is applied.
     */
    advance(frameSeconds, step, { onOverrun = null } = {}) {
        if (typeof step !== 'function') throw new TypeError('step must be a function');

        const requestedSeconds = Number.isFinite(frameSeconds) && frameSeconds > 0
            ? frameSeconds
            : 0;
        const acceptedSeconds = Math.min(requestedSeconds, this.maxCatchUpSeconds);
        const droppedSeconds = requestedSeconds - acceptedSeconds;

        this.frameCount += 1;
        if (droppedSeconds > 0) {
            this.totalDroppedSeconds += droppedSeconds;
            if (typeof onOverrun === 'function') {
                onOverrun(Object.freeze({
                    frameSeconds: requestedSeconds,
                    acceptedSeconds,
                    droppedSeconds,
                    totalDroppedSeconds: this.totalDroppedSeconds,
                    simulatedSeconds: this.simulatedSeconds,
                    accumulatorSeconds: this.accumulatorSeconds,
                    controlDt: this.stepSeconds,
                    maxStepsPerFrame: this.maxStepsPerFrame,
                    maxCatchUpSeconds: this.maxCatchUpSeconds,
                }));
            }
        }

        this.accumulatorSeconds += acceptedSeconds;
        const scheduledSteps = Math.min(
            this.maxStepsPerFrame,
            Math.floor((this.accumulatorSeconds + ROUNDING_EPSILON) / this.stepSeconds),
        );

        for (let stepIndex = 0; stepIndex < scheduledSteps; stepIndex++) {
            // Primitive context arguments avoid allocating 200 short-lived
            // objects per simulated second in the browser hot path.
            step(this.stepSeconds, this.simulatedSeconds, stepIndex);
            this.accumulatorSeconds -= this.stepSeconds;
            this.simulatedSeconds += this.stepSeconds;
        }

        if (Math.abs(this.accumulatorSeconds) < ROUNDING_EPSILON) {
            this.accumulatorSeconds = 0;
        }

        return Object.freeze({
            frameSeconds: requestedSeconds,
            acceptedSeconds,
            droppedSeconds,
            steps: scheduledSteps,
            simulatedThisFrameSeconds: scheduledSteps * this.stepSeconds,
            simulatedSeconds: this.simulatedSeconds,
            accumulatorSeconds: this.accumulatorSeconds,
            totalDroppedSeconds: this.totalDroppedSeconds,
        });
    }
}
