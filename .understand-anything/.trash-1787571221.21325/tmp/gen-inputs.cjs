const fs = require('fs');
const root = process.cwd();
const b = JSON.parse(fs.readFileSync('.understand-anything/intermediate/batches.json', 'utf8'));
for (const batch of b.batches) {
  const inp = { projectRoot: root, batchFiles: batch.files, batchImportData: batch.batchImportData };
  fs.writeFileSync(`.understand-anything/tmp/ua-file-analyzer-input-${batch.batchIndex}.json`, JSON.stringify(inp));
}
console.log('wrote', b.batches.length, 'input files');
