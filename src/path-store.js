/*
 * Copyright 2026 Manifold Tech Ltd.
 * Author: MENG Guotao <mengguotao@manifoldtech.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Per-scene gate-path persistence.
 *
 * Thin client for the `/api/path/<safeName>.json` routes defined in
 * `scripts/server.js` and `scripts/serve.py`. Callers pass a `File` object
 * plus — on save — a JSON-safe record, and we take care of the
 * filename sanitisation + size-suffix keying so identically-named scenes
 * with different sizes don't clobber each other.
 *
 * On-disk record shape (written to `asset/gate-paths/*.json` by the
 * server):
 *
 *   {
 *     schemaVersion: 1,
 *     sceneId:       "google-tiles-hong-kong",
 *     coordSystem:   "zup" | "yup",
 *     path: {
 *       closed:    true,
 *       points:    [{ x, y, z }, ...],     // >= 3 control points
 *       gateSize:  1.2,
 *       clearance: 0.8,
 *       yMin:      0.0,
 *       yMax:      5.4,
 *     } | null,
 *     bestLapMs:   42300 | null,
 *     lastUpdated: "2026-04-20T09:58:00Z",
 *   }
 *
 * `path` being null means "we remembered a coord system and/or best-lap
 * for this scene but the user hasn't drawn a course yet". That's a valid
 * intermediate state and the loader tolerates it.
 */

import { reportUserError } from './error-report.js';

const API_BASE = '/api/path/';
const SCHEMA_VERSION = 1;

/**
 * Build the stable storage key for a loaded scene File. We combine the
 * sanitized base name with the exact byte size so two differently-sized
 * scenes that happen to share a name (e.g. original vs. re-exported)
 * still land in separate files. The `.json` extension is mandatory for
 * the server-side regex.
 *
 * @param {{ name: string, size: number } | null} file
 * @returns {string | null}  safe filename like `google_tiles_hong_kong_0.json`, or null
 *   if the file argument is missing.
 */
export function keyFor(file) {
    if (!file || !file.name) return null;
    // Collapse every filesystem-hostile char to underscore. The storage
    // file's `.json` suffix comes from us.
    const safe = String(file.name).replace(/[^A-Za-z0-9._-]+/g, '_')
                                  .replace(/\.+/g, '_')
                                  .replace(/^_+|_+$/g, '');
    const size = Number.isFinite(Number(file.size)) ? Number(file.size) : 0;
    return `${safe || 'scene'}_${size}.json`;
}

/**
 * Load the stored record for a scene.
 *
 * Resolves to:
 *   - the parsed record object on 200,
 *   - `null` on 404 (first time this scene is loaded),
 *   - `null` on any other failure (network / malformed JSON), with a
 *     user-visible error banner — we never reject, because a missing path
 *     file is not an error condition worth blocking scene load.
 *
 * @param {File} file
 * @returns {Promise<object | null>}
 */
export async function loadForScene(file) {
    const key = keyFor(file);
    if (!key) return null;
    try {
        const res = await fetch(API_BASE + key, { method: 'GET' });
        if (res.status === 404) return null;
        if (!res.ok) {
            reportUserError('Gate path load failed', new Error(`${key}: HTTP ${res.status}`), {
                key: `path-load-${key}`,
                intervalMs: 10000,
            });
            return null;
        }
        const data = await res.json();
        return _validate(data, file) ? data : null;
    } catch (e) {
        reportUserError(`Gate path load failed (${key})`, e, {
            key: `path-load-${key}`,
            intervalMs: 10000,
        });
        return null;
    }
}

/**
 * Save the stored record for a scene. Overwrites any existing file.
 *
 * The `record` is normalised + stamped (schemaVersion, filename, fileSize,
 * lastUpdated) before send, so callers don't have to remember those.
 * Caller-supplied fields (coordSystem, path, bestLapMs) are preserved
 * verbatim — validation happens downstream in the consumer, not here.
 *
 * @param {File} file
 * @param {{
 *   coordSystem?: 'zup' | 'yup',
 *   path?: object | null,
 *   bestLapMs?: number | null,
 * }} record
 * @returns {Promise<boolean>}  true on success, false on failure (reported to the user)
 */
export async function saveForScene(file, record) {
    const key = keyFor(file);
    if (!key) return false;
    const body = JSON.stringify({
        schemaVersion: SCHEMA_VERSION,
        filename:      file.name,
        fileSize:      file.size,
        coordSystem:   record.coordSystem || 'zup',
        path:          record.path || null,
        bestLapMs:     _normaliseBestLap(record.bestLapMs),
        lastUpdated:   new Date().toISOString(),
    });
    try {
        const res = await fetch(API_BASE + key, {
            method:  'PUT',
            headers: { 'Content-Type': 'application/json' },
            body,
        });
        if (!res.ok) {
            reportUserError('Gate path save failed', new Error(`${key}: HTTP ${res.status}`), {
                key: `path-save-${key}`,
                intervalMs: 10000,
            });
            return false;
        }
        return true;
    } catch (e) {
        reportUserError(`Gate path save failed (${key})`, e, {
            key: `path-save-${key}`,
            intervalMs: 10000,
        });
        return false;
    }
}

/**
 * Delete the stored record for a scene. Idempotent — succeeds quietly
 * if the file is already absent.
 *
 * @param {File} file
 * @returns {Promise<boolean>}
 */
export async function deleteForScene(file) {
    const key = keyFor(file);
    if (!key) return false;
    try {
        const res = await fetch(API_BASE + key, { method: 'DELETE' });
        if (res.ok || res.status === 404) return true;
        reportUserError('Gate path delete failed', new Error(`${key}: HTTP ${res.status}`), {
            key: `path-delete-${key}`,
            intervalMs: 10000,
        });
        return false;
    } catch (e) {
        reportUserError(`Gate path delete failed (${key})`, e, {
            key: `path-delete-${key}`,
            intervalMs: 10000,
        });
        return false;
    }
}

// ---- Internal ------------------------------------------------------

/**
 * Light schema check: a tampered or future-version file shouldn't
 * silently break the loader. We require `schemaVersion`, `filename`,
 * `fileSize`; if `fileSize` doesn't match what the browser sees we still
 * accept the record (operator may have re-exported the scene with
 * unrelated tweaks) but log a warning so they know.
 */
function _validate(data, file) {
    if (!data || typeof data !== 'object') return false;
    if (Number(data.schemaVersion) !== SCHEMA_VERSION) {
        reportUserError('Gate path schema mismatch', new Error(`stored v${data.schemaVersion}, client v${SCHEMA_VERSION}`), {
            key: 'path-schema-mismatch',
            intervalMs: 10000,
        });
        return false;
    }
    if (typeof data.filename !== 'string' || !data.filename) return false;
    if (file && Number(data.fileSize) !== Number(file.size)) {
        reportUserError(
            'Gate path file size mismatch',
            new Error(`${data.filename}: stored=${data.fileSize}, current=${file.size}`),
            { key: 'path-size-mismatch', intervalMs: 10000 }
        );
    }
    return true;
}

function _normaliseBestLap(ms) {
    const n = Number(ms);
    return Number.isFinite(n) && n > 0 ? n : null;
}
