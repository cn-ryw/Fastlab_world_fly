/* RadioMaster T8L Web Serial input. The wire contract mirrors the official
 * RM-Web-Page configurator: request processed output channels with command
 * A5 55 1B 0D 0A and parse CRC-protected 0xEE packets. */

export const T8L_USB_VENDOR_ID = 0x19f5;
export const T8L_USB_PRODUCT_ID = 0x5740;
export const T8L_BAUD_RATE = 460800;
export const T8L_CHANNEL_COUNT = 10;
export const T8L_STALE_TIMEOUT_MS = 250;

const OUTPUT_CHANNEL_REQUEST = new Uint8Array([0xA5, 0x55, 0x1B, 0x0D, 0x0A]);
const MAX_PACKET_BYTES = 128;

const CRC8_TABLE = Object.freeze([
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9,
]);

export function t8lCrc8(bytes) {
    let crc = 0;
    for (const byte of bytes) crc = CRC8_TABLE[crc ^ byte];
    return crc;
}

export function normalizeT8lPwm(pwm) {
    return Math.max(-1, Math.min(1, (Number(pwm) - 1500) / 500));
}

export class T8LPacketParser {
    constructor(onChannels = null) {
        this._buffer = [];
        this._onChannels = onChannels;
    }

    push(chunk) {
        if (chunk) this._buffer.push(...new Uint8Array(chunk));
        const parsed = [];
        while (this._buffer.length >= 2) {
            const header = this._buffer.indexOf(0xEE);
            if (header < 0) {
                this._buffer.length = 0;
                break;
            }
            if (header > 0) this._buffer.splice(0, header);
            if (this._buffer.length < 2) break;
            const payloadLength = this._buffer[1];
            const totalLength = payloadLength + 2;
            if (payloadLength < 27 || totalLength > MAX_PACKET_BYTES) {
                this._buffer.shift();
                continue;
            }
            if (this._buffer.length < totalLength) break;
            const packet = this._buffer.slice(0, totalLength);
            const expectedCrc = packet[totalLength - 1];
            const calculatedCrc = t8lCrc8(packet.slice(2, totalLength - 1));
            if (expectedCrc !== calculatedCrc) {
                this._buffer.shift();
                continue;
            }
            const channels = [];
            for (let index = 0; index < T8L_CHANNEL_COUNT; index++) {
                const offset = 8 + index * 2;
                channels.push(packet[offset] | (packet[offset + 1] << 8));
            }
            this._buffer.splice(0, totalLength);
            parsed.push(channels);
            if (this._onChannels) this._onChannels(channels);
        }
        return parsed;
    }

    reset() {
        this._buffer.length = 0;
    }
}

export class T8LSerialInput extends EventTarget {
    constructor({ serial = globalThis.navigator?.serial, clock = () => performance.now() } = {}) {
        super();
        this._serial = serial;
        this._clock = clock;
        this._port = null;
        this._reader = null;
        this._writer = null;
        this._requestTimer = null;
        this._staleTimer = null;
        this._connected = false;
        this._connectedAt = -Infinity;
        this._closing = false;
        this._lastFrameAt = -Infinity;
        this._frameTimes = [];
        this._rawChannels = new Array(T8L_CHANNEL_COUNT).fill(1500);
        this._axes = new Array(T8L_CHANNEL_COUNT).fill(0);
        this._parser = new T8LPacketParser(channels => this._acceptChannels(channels));
    }

    async connect() {
        if (this._connected) return true;
        if (this._closing) throw new Error('T8L serial port is still disconnecting');
        if (!this._serial?.requestPort) throw new Error('Web Serial is unavailable; use Chrome or Edge');
        const port = await this._serial.requestPort({
            filters: [{ usbVendorId: T8L_USB_VENDOR_ID, usbProductId: T8L_USB_PRODUCT_ID }],
        });
        try {
            await port.open({ baudRate: T8L_BAUD_RATE });
            this._port = port;
            this._reader = port.readable.getReader();
            this._writer = port.writable.getWriter();
        } catch (error) {
            try { await port.close(); } catch (_) {}
            this._port = null;
            this._reader = null;
            this._writer = null;
            throw error;
        }
        this._connected = true;
        this._connectedAt = this._clock();
        this._lastFrameAt = -Infinity;
        this._frameTimes.length = 0;
        this._rawChannels.fill(1500);
        this._axes.fill(0);
        this._closing = false;
        this._parser.reset();
        this._requestTimer = globalThis.setInterval(() => this._requestChannels(), 20);
        this._armStaleTimer();
        this._requestChannels();
        this.dispatchEvent(new Event('connect'));
        void this._readLoop();
        return true;
    }

    async disconnect(reason = 'user-disconnect') {
        if (this._closing) return;
        this._closing = true;
        this._connected = false;
        const reader = this._reader;
        const writer = this._writer;
        const port = this._port;
        this._reader = null;
        this._writer = null;
        this._port = null;
        if (this._requestTimer !== null) globalThis.clearInterval(this._requestTimer);
        if (this._staleTimer !== null) globalThis.clearTimeout(this._staleTimer);
        this._requestTimer = null;
        this._staleTimer = null;
        this._parser.reset();
        this._lastFrameAt = -Infinity;
        this._frameTimes.length = 0;
        this._rawChannels.fill(1500);
        this._axes.fill(0);
        // Publish link loss before potentially slow stream/port cleanup so the
        // next animation frame can enter the SO3 hold failsafe immediately.
        this.dispatchEvent(new CustomEvent('disconnect', { detail: { reason } }));
        try { await reader?.cancel(); } catch (_) {}
        try { reader?.releaseLock(); } catch (_) {}
        try { writer?.releaseLock(); } catch (_) {}
        try { await port?.close(); } catch (_) {}
        this._closing = false;
    }

    snapshot(now = this._clock()) {
        const ageMs = Number.isFinite(this._lastFrameAt)
            ? Math.max(0, now - this._lastFrameAt)
            : (this._connected ? Math.max(0, now - this._connectedAt) : Infinity);
        const hasValidFrame = Number.isFinite(this._lastFrameAt);
        return Object.freeze({
            connected: this._connected,
            fresh: this._connected && hasValidFrame && ageMs <= T8L_STALE_TIMEOUT_MS,
            ageMs,
            frameRateHz: this._frameRate(now),
            rawChannels: Object.freeze([...this._rawChannels]),
            axes: Object.freeze([...this._axes]),
        });
    }

    _acceptChannels(channels) {
        if (!this._connected) return;
        const now = this._clock();
        this._rawChannels = channels.slice(0, T8L_CHANNEL_COUNT);
        this._axes = this._rawChannels.map(normalizeT8lPwm);
        this._lastFrameAt = now;
        this._frameTimes.push(now);
        this._trimFrameTimes(now);
        this._armStaleTimer();
        this.dispatchEvent(new CustomEvent('channels', { detail: this.snapshot(now) }));
    }

    _armStaleTimer() {
        if (this._staleTimer !== null) globalThis.clearTimeout(this._staleTimer);
        if (!this._connected) return;
        this._staleTimer = globalThis.setTimeout(() => {
            this._staleTimer = null;
            if (this._connected && !this.snapshot().fresh) {
                this.dispatchEvent(new Event('stale'));
            }
        }, T8L_STALE_TIMEOUT_MS + 1);
    }

    _frameRate(now) {
        this._trimFrameTimes(now);
        if (this._frameTimes.length < 2) return 0;
        const span = this._frameTimes.at(-1) - this._frameTimes[0];
        return span > 0 ? (this._frameTimes.length - 1) * 1000 / span : 0;
    }

    _trimFrameTimes(now) {
        while (this._frameTimes.length && now - this._frameTimes[0] > 1000) this._frameTimes.shift();
    }

    async _requestChannels() {
        if (!this._connected || !this._writer) return;
        try {
            await this._writer.write(OUTPUT_CHANNEL_REQUEST);
        } catch (error) {
            if (!this._closing) void this.disconnect(`write-error:${error?.message || error}`);
        }
    }

    async _readLoop() {
        try {
            while (this._connected && this._reader) {
                const { value, done } = await this._reader.read();
                if (done) break;
                if (value) this._parser.push(value);
            }
            if (this._connected) await this.disconnect('stream-ended');
        } catch (error) {
            if (!this._closing) await this.disconnect(`read-error:${error?.message || error}`);
        }
    }
}
