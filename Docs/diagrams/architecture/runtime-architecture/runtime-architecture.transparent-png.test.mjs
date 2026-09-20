import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';

const html = readFileSync(new URL('./runtime-architecture.html', import.meta.url), 'utf8');

assert.match(html, /data-format="png-transparent"/);
assert.match(html, /querySelector\('rect\[fill="url\(#grid\)"\]'\)/);
assert.match(html, /var transparent = format === 'png-transparent';/);
assert.match(html, /serializeSvg\(scale, \{ transparent: transparent \}\)/);
assert.match(html, /if \(!transparent\) clone\.insertBefore\(bgRect, style\.nextSibling\);/);
assert.match(html, /if \(gridLayer\) gridLayer\.remove\(\);/);
assert.match(html, /-transparent\.png/);

for (const match of html.matchAll(/<script([^>]*)>([\s\S]*?)<\/script>/g)) {
  if (!/application\/json/.test(match[1])) new vm.Script(match[2]);
}

console.log('Transparent PNG background/grid removal and inline script syntax: OK');
