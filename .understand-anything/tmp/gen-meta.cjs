const fs = require('fs');
const { execSync } = require('child_process');
const hash = execSync('git rev-parse HEAD').toString().trim();
const meta = {
  lastAnalyzedAt: new Date().toISOString(),
  gitCommitHash: hash,
  version: '1.0.0',
  analyzedFiles: 62,
};
fs.writeFileSync('.understand-anything/meta.json', JSON.stringify(meta, null, 2));
console.log('wrote meta.json');
