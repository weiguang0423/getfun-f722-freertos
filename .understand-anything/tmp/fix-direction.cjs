const fs = require('fs');
const p = '.understand-anything/knowledge-graph.json';
const g = JSON.parse(fs.readFileSync(p, 'utf8'));
let fixed = 0;
for (const e of g.edges) {
  if (!('direction' in e)) {
    e.direction = 'forward';
    fixed++;
  }
}
fs.writeFileSync(p, JSON.stringify(g, null, 2));
console.log('fixed edges missing direction:', fixed);
console.log('total edges:', g.edges.length);
console.log('remaining missing:', g.edges.filter(e => !('direction' in e)).length);
