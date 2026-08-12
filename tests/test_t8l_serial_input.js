import assert from 'node:assert/strict';
import {
    T8L_BAUD_RATE,
    T8L_USB_PRODUCT_ID,
    T8L_USB_VENDOR_ID,
    T8LPacketParser,
    T8LSerialInput,
    normalizeT8lPwm,
    t8lCrc8,
} from '../src/t8l-serial-input.js';

function packet(channels, command = 0xEE) {
    const length = 27;
    const bytes = new Uint8Array(length + 2);
    bytes[0] = command;
    bytes[1] = length;
    channels.forEach((value, index) => {
        const offset = 8 + index * 2;
        bytes[offset] = value & 0xff;
        bytes[offset + 1] = value >> 8;
    });
    bytes[bytes.length - 1] = t8lCrc8(bytes.slice(2, -1));
    return bytes;
}

const expected = [988, 1000, 1250, 1500, 1750, 2000, 2012, 1499, 1501, 1600];
const frame = packet(expected);
const parser = new T8LPacketParser();
assert.deepEqual(parser.push(frame.slice(0, 7)), [], 'partial packet waits');
assert.deepEqual(parser.push(frame.slice(7)), [expected], 'split packet rejoins');

const second = packet(expected.map(value => value + 1));
assert.deepEqual(parser.push(new Uint8Array([1, 2, 3, ...frame, ...second])), [
    expected,
    expected.map(value => value + 1),
], 'noise and concatenated packets resynchronize');

const broken = frame.slice();
broken[10] ^= 0xff;
assert.deepEqual(parser.push(new Uint8Array([...broken, ...second])), [
    expected.map(value => value + 1),
], 'bad CRC is discarded without losing the next frame');

assert.equal(normalizeT8lPwm(988), -1);
assert.equal(normalizeT8lPwm(1500), 0);
assert.equal(normalizeT8lPwm(2012), 1);
assert.equal(normalizeT8lPwm(1750), 0.5);

let now = 1000;
const input = new T8LSerialInput({ serial: null, clock: () => now });
input._connected = true;
input._connectedAt = now;
assert.equal(input.snapshot().fresh, false, 'an open port is not fresh before a valid frame');
input._acceptChannels(expected);
assert.equal(input.snapshot().fresh, true, 'valid frame is fresh');
assert.equal(input.snapshot().rawChannels[0], 988);
now += 251;
assert.equal(input.snapshot().fresh, false, 'input expires after 250 ms');
input._parser.push(broken);
assert.equal(input.snapshot().fresh, false, 'invalid frame cannot refresh freshness');
await input.disconnect('test');
assert.equal(input.snapshot().connected, false);
assert.deepEqual(input.snapshot().axes, new Array(10).fill(0), 'disconnect clears axes');
input._acceptChannels(expected);
assert.deepEqual(input.snapshot().axes, new Array(10).fill(0), 'late bytes cannot repopulate a disconnected input');

let readResolve;
const writes = [];
let requestedOptions = null;
let openedOptions = null;
let closed = false;
const reader = {
    read: () => new Promise(resolve => { readResolve = resolve; }),
    cancel: async () => readResolve?.({ done: true }),
    releaseLock() {},
};
const writer = {
    async write(bytes) { writes.push([...bytes]); },
    releaseLock() {},
};
const port = {
    readable: { getReader: () => reader },
    writable: { getWriter: () => writer },
    async open(options) { openedOptions = options; },
    async close() { closed = true; },
};
const serial = {
    async requestPort(options) { requestedOptions = options; return port; },
};
const live = new T8LSerialInput({ serial });
let disconnectedReason = null;
live.addEventListener('disconnect', event => { disconnectedReason = event.detail.reason; });
await live.connect();
await Promise.resolve();
assert.deepEqual(requestedOptions, {
    filters: [{ usbVendorId: T8L_USB_VENDOR_ID, usbProductId: T8L_USB_PRODUCT_ID }],
});
assert.deepEqual(openedOptions, { baudRate: T8L_BAUD_RATE });
assert.deepEqual(writes[0], [0xA5, 0x55, 0x1B, 0x0D, 0x0A], 'processed-channel request is sent immediately');
await live.disconnect('test-live-disconnect');
assert.equal(disconnectedReason, 'test-live-disconnect');
assert.equal(closed, true);

console.log('T8L serial input: all tests passed');
