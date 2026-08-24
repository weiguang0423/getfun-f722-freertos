// Static dashboard server: serves the prebuilt dist/ and replicates the
// token-gated graph endpoints from vite.config.ts (configureServer middleware).
// Avoids the Vite 6.4.3 Windows pre-optimize crash entirely.
const http = require('http');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const PLUGIN_ROOT = process.env.PLUGIN_ROOT || path.resolve(process.env.HOME || process.env.USERPROFILE, '.understand-anything-plugin');
const DIST = path.join(PLUGIN_ROOT, 'packages', 'dashboard', 'dist');
const GRAPH_DIR = process.env.GRAPH_DIR || process.cwd();
const ACCESS_TOKEN = process.env.UNDERSTAND_ACCESS_TOKEN || crypto.randomBytes(16).toString('hex');
const PORT = parseInt(process.env.PORT || '5173', 10);

function graphFileCandidates(fileName) {
  return [
    path.resolve(GRAPH_DIR, '.understand-anything', fileName),
    path.resolve(process.cwd(), '.understand-anything', fileName),
    path.resolve(process.cwd(), '..', '..', '..', '.understand-anything', fileName),
  ];
}

function projectRootFromGraphFile(candidate) {
  return path.dirname(path.dirname(candidate));
}

function sendJson(res, statusCode, payload) {
  res.statusCode = statusCode;
  res.setHeader('Content-Type', 'application/json');
  res.end(JSON.stringify(payload));
}

function sanitizeGraph(raw, candidate) {
  const projectRoot = projectRootFromGraphFile(candidate);
  if (Array.isArray(raw.nodes)) {
    raw.nodes = raw.nodes.map((node) => {
      if (typeof node.filePath !== 'string') return node;
      const abs = node.filePath;
      const rel = abs.startsWith(projectRoot)
        ? abs.slice(projectRoot.length).replace(/^[\\/]/, '')
        : path.isAbsolute(abs)
        ? path.basename(abs)
        : abs;
      return { ...node, filePath: rel };
    });
  }
  return raw;
}

const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
  '.json': 'application/json', '.svg': 'image/svg+xml', '.ico': 'image/x-icon',
  '.woff': 'font/woff', '.woff2': 'font/woff2', '.ttf': 'font/ttf', '.map': 'application/json',
};

const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://127.0.0.1:' + PORT);
  const pathname = url.pathname;
  const protectedEndpoints = ['/knowledge-graph.json', '/domain-graph.json', '/diff-overlay.json', '/meta.json', '/config.json', '/file-content.json'];

  if (protectedEndpoints.includes(pathname)) {
    if (url.searchParams.get('token') !== ACCESS_TOKEN) {
      sendJson(res, 403, { error: 'Forbidden: missing or invalid token' });
      return;
    }
    if (pathname === '/config.json') {
      const cfg = graphFileCandidates('config.json').find(fs.existsSync);
      if (cfg) { try { return sendJson(res, 200, JSON.parse(fs.readFileSync(cfg, 'utf-8'))); } catch { return sendJson(res, 500, { error: 'Failed to read config' }); } }
      return sendJson(res, 200, { autoUpdate: false, outputLanguage: 'en' });
    }
    const fileName = pathname === '/diff-overlay.json' ? 'diff-overlay.json'
      : pathname === '/meta.json' ? 'meta.json'
      : pathname === '/domain-graph.json' ? 'domain-graph.json'
      : 'knowledge-graph.json';
    const candidate = graphFileCandidates(fileName).find(fs.existsSync);
    if (!candidate) {
      if (pathname === '/knowledge-graph.json') return sendJson(res, 404, { error: 'No knowledge graph found. Run /understand first.' });
      res.statusCode = 404; return res.end();
    }
    try {
      const raw = sanitizeGraph(JSON.parse(fs.readFileSync(candidate, 'utf-8')), candidate);
      res.setHeader('Content-Type', 'application/json');
      res.end(JSON.stringify(raw));
    } catch (err) {
      console.error('[understand] failed to sanitise', err);
      sendJson(res, 500, { error: 'Failed to read graph file' });
    }
    return;
  }

  // static serving from dist
  let rel = decodeURIComponent(pathname);
  if (rel === '/' || rel === '') rel = '/index.html';
  const filePath = path.join(DIST, rel);
  if (!filePath.startsWith(DIST) || !fs.existsSync(filePath) || !fs.statSync(filePath).isFile()) {
    // SPA fallback
    const idx = path.join(DIST, 'index.html');
    if (fs.existsSync(idx)) {
      res.setHeader('Content-Type', 'text/html');
      return res.end(fs.readFileSync(idx));
    }
    res.statusCode = 404; return res.end('Not found');
  }
  const ext = path.extname(filePath).toLowerCase();
  res.setHeader('Content-Type', MIME[ext] || 'application/octet-stream');
  res.end(fs.readFileSync(filePath));
});

server.listen(PORT, '127.0.0.1', () => {
  console.log(`\n  🔑  Dashboard URL: http://127.0.0.1:${PORT}/?token=${ACCESS_TOKEN}\n`);
  console.log(`  Serving dist: ${DIST}`);
  console.log(`  Graph dir:  ${GRAPH_DIR}/.understand-anything/`);
});
