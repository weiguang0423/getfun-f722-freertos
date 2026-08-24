'use strict';
const fs = require('fs');
const inPath = process.argv[2];
const data = JSON.parse(fs.readFileSync(inPath, 'utf8'));
const files = data.files;
// deeper breakdown of Middlewares
function depth(n, top) {
  const m = {};
  for (const f of files) {
    if (f.filePath.startsWith(top + '/')) {
      const rest = f.filePath.slice((top + '/').length).split('/');
      const key = rest.slice(0, n).join('/');
      (m[key] = m[key] || []).push(f.filePath);
    }
  }
  return m;
}
console.log('=== Middlewares (depth 2) ===');
const mw = depth(2, 'Middlewares');
for (const k of Object.keys(mw).sort()) console.log('  ' + k + ': ' + mw[k].length);
console.log('\n=== Linux/RK3568 (depth 2) ===');
const rk = depth(2, 'Linux/RK3568');
for (const k of Object.keys(rk).sort()) console.log('  ' + k + ': ' + rk[k].length);
console.log('\n=== node type breakdown by id prefix ===');
const byType = {};
for (const f of files) (byType[f.type] = byType[f.type] || []).push(f.id);
for (const t of Object.keys(byType)) {
  console.log('TYPE ' + t + ' (' + byType[t].length + '):');
  byType[t].slice(0, 30).forEach(id => console.log('   ' + id));
}
