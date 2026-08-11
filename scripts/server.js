/*
 * Static server for the Google 3D Tiles flight app inside the 3DCityDB/Cesium image.
 *
 * Serves:
 *   /                         -> this project
 *   /ThirdParty/Cesium/...    -> Cesium bundled in tumgis/3dcitydb-web-map
 *   /js/3dcitydb-web-map.js   -> optional 3DCityDB helper from the base image
 *   /api/path/<name>.json     -> gate-path persistence API shared with scripts/serve.py
 */

const express = require('express');
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = Number(process.env.PORT || 8000);
const APP_ROOT = path.resolve(__dirname, '..');
const WEB_ROOT = '/var/www';
const PATHS_DIR = path.join(APP_ROOT, 'asset', 'gate-paths');
const MAX_PATH_BODY = 64 * 1024;
const SAFE_NAME_RE = /^[A-Za-z0-9._-]{1,200}\.json$/;

function safePathFile(name) {
    if (!name || !SAFE_NAME_RE.test(name)) return null;
    const candidate = path.normalize(path.join(PATHS_DIR, name));
    const rel = path.relative(PATHS_DIR, candidate);
    if (rel.startsWith('..') || path.isAbsolute(rel)) return null;
    return candidate;
}

function sendText(res, code, message) {
    res.status(code).type('text/plain; charset=utf-8').send(`${message}\n`);
}

function readRawBody(req, res, next) {
    let size = 0;
    const chunks = [];
    req.on('data', (chunk) => {
        size += chunk.length;
        if (size > MAX_PATH_BODY) {
            req.destroy();
            return sendText(res, 413, `body too large (> ${MAX_PATH_BODY} bytes)`);
        }
        chunks.push(chunk);
    });
    req.on('end', () => {
        req.rawBody = Buffer.concat(chunks);
        next();
    });
    req.on('error', (err) => sendText(res, 400, `read body failed: ${err.message}`));
}

app.use((req, res, next) => {
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
    // Per-path caching: immutable assets get long max-age; HTML revalidates
    const p = req.path;
    if (p === '/runtime-config.js' || p.startsWith('/api/')) {
        res.setHeader('Cache-Control', 'no-store');
    } else if (p === '/' || p === '/index.html' || p.endsWith('.html')) {
        res.setHeader('Cache-Control', 'no-cache');
    } else {
        res.setHeader('Cache-Control', 'public, max-age=86400');
    }
    next();
});

app.get('/runtime-config.js', (req, res) => {
    const config = { cesiumIonToken: process.env.CESIUM_ION_TOKEN || '' };
    res.status(200)
        .type('application/javascript; charset=utf-8')
        .send(`window.MINDCLOUD_RUNTIME_CONFIG = Object.freeze(${JSON.stringify(config)});\n`);
});

app.options('/api/path/:name', (req, res) => res.status(204).end());

app.get('/api/path/:name', (req, res) => {
    const fp = safePathFile(req.params.name);
    if (!fp) return sendText(res, 400, 'invalid path name');
    fs.readFile(fp, (err, body) => {
        if (err) {
            if (err.code === 'ENOENT') return sendText(res, 404, 'not found');
            return sendText(res, 500, `read failed: ${err.message}`);
        }
        res.status(200).type('application/json; charset=utf-8').send(body);
    });
});

app.put('/api/path/:name', readRawBody, (req, res) => {
    const fp = safePathFile(req.params.name);
    if (!fp) return sendText(res, 400, 'invalid path name');
    const body = Buffer.isBuffer(req.rawBody) ? req.rawBody : Buffer.alloc(0);
    if (!body.length) return sendText(res, 400, 'empty body');
    try {
        JSON.parse(body.toString('utf8'));
    } catch (e) {
        return sendText(res, 400, `not valid JSON: ${e.message}`);
    }
    fs.mkdirSync(PATHS_DIR, { recursive: true });
    const tmp = `${fp}.tmp`;
    fs.writeFile(tmp, body, (writeErr) => {
        if (writeErr) return sendText(res, 500, `write failed: ${writeErr.message}`);
        fs.rename(tmp, fp, (renameErr) => {
            if (renameErr) return sendText(res, 500, `rename failed: ${renameErr.message}`);
            res.status(204).end();
        });
    });
});

app.delete('/api/path/:name', (req, res) => {
    const fp = safePathFile(req.params.name);
    if (!fp) return sendText(res, 400, 'invalid path name');
    fs.unlink(fp, (err) => {
        if (err) {
            if (err.code === 'ENOENT') return sendText(res, 404, 'not found');
            return sendText(res, 500, `delete failed: ${err.message}`);
        }
        res.status(204).end();
    });
});

app.use(express.static(APP_ROOT, {
    setHeaders(res, filePath) {
        if (filePath.endsWith('.js')) {
            res.setHeader('Content-Type', 'application/javascript; charset=UTF-8');
        }
    },
}));
app.use(express.static(WEB_ROOT));

const server = app.listen(PORT, '0.0.0.0', () => {
    console.log(`Google 3D Tiles Flight container server listening on 0.0.0.0:${PORT}`);
    console.log(`Project root: ${APP_ROOT}`);
    console.log(`Cesium root: ${path.join(WEB_ROOT, 'ThirdParty', 'Cesium')}`);
});

let shuttingDown = false;
function shutdown(signal) {
    if (shuttingDown) return;
    shuttingDown = true;
    console.log(`Received ${signal}; shutting down HTTP server...`);
    server.close(() => {
        console.log('HTTP server stopped.');
        process.exit(0);
    });
    setTimeout(() => {
        console.warn('HTTP server did not stop in time; forcing exit.');
        process.exit(1);
    }, 5000).unref();
}

process.once('SIGINT', () => shutdown('SIGINT'));
process.once('SIGTERM', () => shutdown('SIGTERM'));
