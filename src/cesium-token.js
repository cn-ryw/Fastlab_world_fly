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

export const CESIUM_ION_TOKEN_STORAGE_KEY = 'mindcloud_cesium_ion_token';
// Browser-visible Cesium client credential. Keep server-side credentials out
// of this module; this default has only the client scopes needed by the demo.
const DEFAULT_CESIUM_ION_TOKEN = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJlMTg2MGFhOS02YTdhLTQ1NWMtYjkzMi05YjQ2ODRlZjI5YTgiLCJpZCI6MjUxNzM1LCJpYXQiOjE3MzAyODI0ODN9.prWAxx4RB8teelutQQbVqdxhgRZpZ4zjw8wzM-8k1Ug';

function normalizedToken(value) {
    return typeof value === 'string' ? value.trim() : '';
}

export function storedCesiumIonToken(storage = undefined) {
    try {
        const target = storage ?? globalThis.localStorage;
        return normalizedToken(target?.getItem(CESIUM_ION_TOKEN_STORAGE_KEY));
    } catch (_) {
        return '';
    }
}

export function resolveCesiumIonToken(options = {}) {
    const explicit = normalizedToken(options.explicitToken);
    const stored = storedCesiumIonToken(options.storage);
    const resolved = explicit || stored || DEFAULT_CESIUM_ION_TOKEN;
    if (!explicit && !stored && resolved) {
        try {
            storeCesiumIonToken(resolved, options.storage);
        } catch (_) {
            // The bundled browser default still works for this session when
            // persistence is unavailable; the launcher normally provides it.
        }
    }
    return resolved;
}

export function storeCesiumIonToken(value, storage = undefined) {
    const token = normalizedToken(value);
    if (!token) throw new Error('Enter a non-empty Cesium Ion token.');

    try {
        const target = storage ?? globalThis.localStorage;
        if (!target || typeof target.setItem !== 'function') {
            throw new Error('localStorage is unavailable');
        }
        target.setItem(CESIUM_ION_TOKEN_STORAGE_KEY, token);
    } catch (_) {
        throw new Error('The browser could not persist the Cesium Ion token. Use a normal, non-private browser profile.');
    }
    return token;
}
