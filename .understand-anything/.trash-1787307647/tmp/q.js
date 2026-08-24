const r = require('E:/getfun-f722-freertos/GETFUN_F722_FreeRTOS/.understand-anything/tmp/ua-tour-results.json');
console.log('=== BFS start:', r.bfsTraversal.startNode);
for (const d of Object.keys(r.bfsTraversal.byDepth)) {
  console.log('--- depth', d, '(' + r.bfsTraversal.byDepth[d].length + ' nodes)');
  for (const id of r.bfsTraversal.byDepth[d]) console.log('   ', id);
}
console.log('\n=== FAN-IN TOP 20');
for (const x of r.fanInRanking) console.log('  ', x.fanIn, x.id);
console.log('\n=== FAN-OUT TOP 20');
for (const x of r.fanOutRanking) console.log('  ', x.fanOut, x.id);
console.log('\n=== LAYERS');
for (const l of r.layers.list) console.log('  ', l.name, '::', l.description);
console.log('\n=== CLUSTERS');
for (const c of r.clusters) console.log('  ', c.edgeCount, c.nodes.join(' <-> '));
