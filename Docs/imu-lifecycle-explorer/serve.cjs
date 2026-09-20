// Local preview serves only the explicitly listed teaching assets.
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const assets = new Map([
  ['/', ['index.html', 'text/html']],
  ['/index.html', ['index.html', 'text/html']],
  ['/hardware.js', ['hardware.js', 'text/javascript']],
  ['/steps.html', ['steps.html', 'text/html']],
  ['/model.js', ['model.js', 'text/javascript']],
  ['/overview.html', ['overview.html', 'text/html']]
]);
const server = http.createServer((req, res) => {
  const asset = assets.get(req.url);
  if (!asset || !['GET', 'HEAD'].includes(req.method)) { res.writeHead(404); res.end(); return; }
  try {
    const bytes = fs.readFileSync(path.join(__dirname, asset[0]));
    res.writeHead(200, {'Content-Type': asset[1] + '; charset=utf-8', 'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff'});
    res.end(req.method === 'HEAD' ? undefined : bytes);
  } catch { res.writeHead(500); res.end('Preview asset unavailable'); }
});
server.on('error', e => { console.error(e.message); process.exitCode = 1; });
server.listen(5178, '127.0.0.1', () => console.log('IMU preview: http://127.0.0.1:5178'));
