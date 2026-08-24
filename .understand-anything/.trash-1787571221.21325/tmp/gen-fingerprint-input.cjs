const fs = require('fs');
const { execSync } = require('child_process');
const scan = JSON.parse(fs.readFileSync('.understand-anything/intermediate/scan-result.json', 'utf8'));
const paths = scan.files.map(f => f.path);
const hash = execSync('git rev-parse HEAD').toString().trim();
const inp = { projectRoot: process.cwd(), sourceFilePaths: paths, gitCommitHash: hash };
fs.writeFileSync('.understand-anything/intermediate/fingerprint-input.json', JSON.stringify(inp, null, 2));
console.log('fingerprint input:', paths.length, 'files, hash', hash);
