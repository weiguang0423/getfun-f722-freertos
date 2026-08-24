'use strict';
const fs = require('fs');

function main() {
  const inPath = process.argv[2];
  const outPath = process.argv[3];
  if (!inPath || !outPath) {
    console.error('Usage: node ua-tour-analyze.js <input.json> <output.json>');
    process.exit(1);
  }

  const data = JSON.parse(fs.readFileSync(inPath, 'utf8'));
  const nodes = data.nodes || [];
  const edges = data.edges || [];
  const layers = data.layers || [];

  const nodeById = {};
  for (const n of nodes) nodeById[n.id] = n;

  // --- A. Fan-In ranking ---
  const fanIn = {};
  const fanOut = {};
  for (const n of nodes) { fanIn[n.id] = 0; fanOut[n.id] = 0; }
  for (const e of edges) {
    if (nodeById[e.target]) fanIn[e.target] = (fanIn[e.target] || 0) + 1;
    if (nodeById[e.source]) fanOut[e.source] = (fanOut[e.source] || 0) + 1;
  }
  const fanInRanking = Object.keys(fanIn)
    .map(id => ({ id, fanIn: fanIn[id], name: nodeById[id].name }))
    .sort((a, b) => b.fanIn - a.fanIn || a.id.localeCompare(b.id))
    .slice(0, 20);
  const fanOutRanking = Object.keys(fanOut)
    .map(id => ({ id, fanOut: fanOut[id], name: nodeById[id].name }))
    .sort((a, b) => b.fanOut - a.fanOut || a.id.localeCompare(b.id))
    .slice(0, 20);

  // --- C. Entry point candidates ---
  const entryNames = ['index.ts','index.js','main.ts','main.js','app.ts','app.js','server.ts','server.js',
    'mod.rs','main.go','main.py','main.rs','manage.py','app.py','wsgi.py','asgi.py','run.py','__main__.py',
    'Application.java','Main.java','Program.cs','config.ru','index.php','App.swift','Application.kt','main.cpp','main.c'];
  const candScore = {};
  for (const n of nodes) {
    let score = 0;
    const name = n.name || '';
    const fp = n.filePath || '';
    const depth = (fp.match(/\//g) || []).length; // count separators
    if (n.type === 'file') {
      if (entryNames.includes(name)) score += 3;
      if (depth <= 1) score += 1;
    } else if (n.type === 'document') {
      if (name === 'README.md' && depth === 0) score += 5;
      else if (name.endsWith('.md') && depth === 0) score += 2;
    }
    if (score > 0) candScore[n.id] = score;
  }
  const fanOutVals = Object.values(fanOut).sort((a,b)=>a-b);
  const top10pct = fanOutVals[Math.floor(fanOutVals.length * 0.9)] || Infinity;
  const fanInVals = Object.values(fanIn).sort((a,b)=>a-b);
  const bottom25pct = fanInVals[Math.floor(fanInVals.length * 0.25)] || -1;
  for (const id of Object.keys(candScore)) {
    const n = nodeById[id];
    if (n.type === 'file') {
      if (fanOut[id] >= top10pct) candScore[id] += 1;
      if (fanIn[id] <= bottom25pct) candScore[id] += 1;
    }
  }
  const entryPointCandidates = Object.keys(candScore)
    .map(id => ({ id, score: candScore[id], name: nodeById[id].name, summary: nodeById[id].summary }))
    .sort((a, b) => b.score - a.score || a.id.localeCompare(b.id))
    .slice(0, 5);

  // --- D. BFS from top CODE entry point ---
  const codeEntry = entryPointCandidates.find(c => {
    const n = nodeById[c.id];
    return n && (n.type === 'file' || n.type === 'config');
  });
  const bfsTraversal = { startNode: null, order: [], depthMap: {}, byDepth: {} };
  if (codeEntry) {
    const start = codeEntry.id;
    bfsTraversal.startNode = start;
    const visited = new Set([start]);
    const queue = [{ id: start, depth: 0 }];
    const adj = {};
    for (const e of edges) {
      if (e.type === 'imports' || e.type === 'calls') {
        (adj[e.source] = adj[e.source] || []).push(e.target);
      }
    }
    while (queue.length) {
      const cur = queue.shift();
      bfsTraversal.order.push(cur.id);
      bfsTraversal.depthMap[cur.id] = cur.depth;
      (bfsTraversal.byDepth[cur.depth] = bfsTraversal.byDepth[cur.depth] || []).push(cur.id);
      for (const next of (adj[cur.id] || [])) {
        if (!visited.has(next) && nodeById[next]) {
          visited.add(next);
          queue.push({ id: next, depth: cur.depth + 1 });
        }
      }
    }
  }

  // --- E. Non-code inventory ---
  const nonCodeFiles = { documentation: [], infrastructure: [], data: [], config: [] };
  for (const n of nodes) {
    if (n.type === 'document') nonCodeFiles.documentation.push({ id: n.id, name: n.name, type: n.type, summary: n.summary });
    else if (n.type === 'config') nonCodeFiles.config.push({ id: n.id, name: n.name, type: n.type, summary: n.summary });
    else if (n.type === 'service' || n.type === 'pipeline' || n.type === 'resource')
      nonCodeFiles.infrastructure.push({ id: n.id, name: n.name, type: n.type, summary: n.summary });
    else if (n.type === 'table' || n.type === 'schema' || n.type === 'endpoint')
      nonCodeFiles.data.push({ id: n.id, name: n.name, type: n.type, summary: n.summary });
  }

  // --- F. Clusters (bidirectional pairs) ---
  const pairEdges = {};
  for (const e of edges) {
    const k = [e.source, e.target].sort().join('|');
    (pairEdges[k] = pairEdges[k] || []).push(e);
  }
  const clusters = [];
  const seen = new Set();
  for (const k of Object.keys(pairEdges)) {
    const [a, b] = k.split('|');
    const hasFwd = edges.some(e => e.source === a && e.target === b);
    const hasRev = edges.some(e => e.source === b && e.target === a);
    if (hasFwd && hasRev && !seen.has(k)) {
      seen.add(k);
      clusters.push({ nodes: [a, b], edgeCount: pairEdges[k].length });
    }
  }
  clusters.sort((x, y) => y.edgeCount - x.edgeCount || y.nodes.length - x.nodes.length);

  // --- G. Layers ---
  const layerList = layers.map(l => ({ id: l.id, name: l.name, description: l.description }));

  // --- H. Node summary index ---
  const nodeSummaryIndex = {};
  for (const n of nodes) {
    nodeSummaryIndex[n.id] = { name: n.name, type: n.type, summary: n.summary, filePath: n.filePath };
  }

  const result = {
    scriptCompleted: true,
    entryPointCandidates,
    fanInRanking,
    fanOutRanking,
    bfsTraversal,
    nonCodeFiles,
    clusters: clusters.slice(0, 10),
    layers: { count: layers.length, list: layerList },
    nodeSummaryIndex,
    totalNodes: nodes.length,
    totalEdges: edges.length
  };

  fs.writeFileSync(outPath, JSON.stringify(result, null, 2));
  console.log('Analysis written to', outPath);
  console.log('totalNodes', nodes.length, 'totalEdges', edges.length);
  console.log('top entry', entryPointCandidates.map(c=>c.id+'('+c.score+')').join(', '));
}

main();
