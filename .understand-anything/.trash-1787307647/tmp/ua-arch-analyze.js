'use strict';
const fs = require('fs');

const inputPath = process.argv[2];
const outPath = process.argv[3];

const raw = fs.readFileSync(inputPath, 'utf8');
const data = JSON.parse(raw);

const files = data.files || [];
const imports = data.imports || [];
const all = data.all || [];

// A. Directory grouping by top-level segment after common prefix
function commonPrefix(paths) {
  if (paths.length === 0) return '';
  let prefix = paths[0].split('/');
  for (const p of paths.slice(1)) {
    const segs = p.split('/');
    let i = 0;
    while (i < prefix.length && i < segs.length && prefix[i] === segs[i]) i++;
    prefix = prefix.slice(0, i);
  }
  // return prefix as path (drop trailing partial)
  return prefix.length ? prefix.join('/') + '/' : '';
}

const filePaths = files.map(f => f.filePath);
const prefix = commonPrefix(filePaths);

function topGroup(fp) {
  let rest = fp;
  if (prefix && fp.startsWith(prefix)) rest = fp.slice(prefix.length);
  const segs = rest.split('/');
  return segs[0] || '(root)';
}

const directoryGroups = {};
for (const f of files) {
  const g = topGroup(f.filePath);
  (directoryGroups[g] = directoryGroups[g] || []).push(f.id);
}

// B. Node type grouping
const nodeTypeGroups = {};
for (const f of files) {
  (nodeTypeGroups[f.type] = nodeTypeGroups[f.type] || []).push(f.id);
}

// C. Import adjacency
const fanOut = {}, fanIn = {};
for (const f of files) { fanOut[f.id] = 0; fanIn[f.id] = 0; }
for (const e of imports) {
  if (fanOut[e.source] !== undefined) fanOut[e.source]++;
  if (fanIn[e.target] !== undefined) fanIn[e.target]++;
}

// D. Cross-category edges
const crossCat = {};
for (const e of all) {
  const s = files.find(x => x.id === e.source);
  const t = files.find(x => x.id === e.target);
  if (!s || !t) continue;
  const key = `${s.type}->${t.type}:${e.type}`;
  crossCat[key] = (crossCat[key] || 0) + 1;
}
const crossCategoryEdges = Object.entries(crossCat).map(([k, count]) => {
  const [ft, rest] = k.split('->');
  const [tt, et] = rest.split(':');
  return { fromType: ft, toType: tt, edgeType: et, count };
});

// E. Inter-group imports
const interGroup = {};
for (const e of imports) {
  const s = files.find(x => x.id === e.source);
  const t = files.find(x => x.id === e.target);
  if (!s || !t) continue;
  const sg = topGroup(s.filePath), tg = topGroup(t.filePath);
  const key = `${sg}->${tg}`;
  interGroup[key] = (interGroup[key] || 0) + 1;
}
const interGroupImports = Object.entries(interGroup).map(([k, count]) => {
  const [from, to] = k.split('->');
  return { from, to, count };
}).sort((a, b) => b.count - a.count);

// F. Intra-group density
const intraGroupDensity = {};
for (const g of Object.keys(directoryGroups)) {
  let internal = 0, total = 0;
  for (const e of imports) {
    const s = files.find(x => x.id === e.source);
    const t = files.find(x => x.id === e.target);
    if (!s || !t) continue;
    const sg = topGroup(s.filePath), tg = topGroup(t.filePath);
    if (sg === g || tg === g) total++;
    if (sg === g && tg === g) internal++;
  }
  const grpIds = directoryGroups[g];
  intraGroupDensity[g] = {
    internalEdges: internal,
    totalEdges: total,
    density: total ? internal / total : 0,
    size: grpIds.length
  };
}

// K. Dependency direction (per pair)
const depDir = {};
for (const e of imports) {
  const s = files.find(x => x.id === e.source);
  const t = files.find(x => x.id === e.target);
  if (!s || !t) continue;
  const sg = topGroup(s.filePath), tg = topGroup(t.filePath);
  if (sg === tg) continue;
  depDir[`${sg}|${tg}`] = (depDir[`${sg}|${tg}`] || 0) + 1;
}
const dependencyDirection = [];
for (const k of Object.keys(depDir)) {
  const [a, b] = k.split('|');
  const ab = depDir[k];
  const ba = depDir[`${b}|${a}`] || 0;
  if (ab > ba) dependencyDirection.push({ dependent: a, dependsOn: b });
}

// fileStats
const nodeTypeCounts = {};
for (const f of files) nodeTypeCounts[f.type] = (nodeTypeCounts[f.type] || 0) + 1;
const filesPerGroup = {};
for (const g of Object.keys(directoryGroups)) filesPerGroup[g] = directoryGroups[g].length;

const result = {
  scriptCompleted: true,
  commonPrefix: prefix,
  directoryGroups,
  nodeTypeGroups,
  crossCategoryEdges,
  interGroupImports,
  intraGroupDensity,
  dependencyDirection,
  fileStats: {
    totalFileNodes: files.length,
    filesPerGroup,
    nodeTypeCounts
  },
  fileFanIn: fanIn,
  fileFanOut: fanOut
};

fs.writeFileSync(outPath, JSON.stringify(result, null, 2), 'utf8');
console.log('Wrote', outPath);
console.log('Total files:', files.length);
console.log('Directory groups:', Object.keys(directoryGroups).sort().join(', '));
console.log('Node types:', JSON.stringify(nodeTypeCounts));
console.log('filesPerGroup:', JSON.stringify(filesPerGroup, null, 0));
