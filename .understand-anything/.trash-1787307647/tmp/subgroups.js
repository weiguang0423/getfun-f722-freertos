'use strict';
const fs = require('fs');
const inPath = process.argv[2];
const data = JSON.parse(fs.readFileSync(inPath, 'utf8'));
const files = data.files;
function subGroups(top) {
  const m = {};
  for (const f of files) {
    if (f.filePath.startsWith(top + '/')) {
      const rest = f.filePath.slice((top + '/').length);
      const seg = rest.includes('/') ? rest.split('/')[0] : '(file)';
      (m[seg] = m[seg] || []).push(f.filePath);
    }
  }
  return m;
}
for (const top of ['Drivers', 'Middlewares', 'Linux', 'APP', 'Core', 'USB_DEVICE', 'Tools', 'cmake']) {
  console.log('\n=== ' + top + ' ===');
  const sg = subGroups(top);
  for (const k of Object.keys(sg).sort()) {
    console.log('  ' + k + ': ' + sg[k].length);
  }
}
