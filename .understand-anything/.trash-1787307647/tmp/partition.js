'use strict';
const fs = require('fs');
const inPath = process.argv[2];
const data = JSON.parse(fs.readFileSync(inPath, 'utf8'));
const files = data.files;

function layerOf(f) {
  const p = f.filePath;
  const id = f.id;
  // APP application layer
  if (p.startsWith('APP/')) return 'layer:app';
  // CubeMX generated app layer
  if (p.startsWith('Core/')) return 'layer:cubemx-app';
  if (p.startsWith('USB_DEVICE/')) return 'layer:cubemx-app';
  // HAL driver
  if (p.startsWith('Drivers/STM32F7xx_HAL_Driver/')) return 'layer:hal';
  // CMSIS (include + Device), but LICENSE stays with cmsis
  if (p.startsWith('Drivers/CMSIS/')) return 'layer:cmsis';
  // FreeRTOS kernel
  if (p.startsWith('Middlewares/Third_Party/FreeRTOS/')) return 'layer:freertos';
  // ST USB device library (+ its LICENSE)
  if (p.startsWith('Middlewares/ST/')) return 'layer:usb-device';
  // RK3568 Linux companion
  if (p.startsWith('Linux/RK3568/')) return 'layer:rk3568';
  // Tools & verification scripts
  if (p.startsWith('Tools/')) return 'layer:tools';
  // Build & config (root-level)
  if (p === 'CMakeLists.txt' || p === 'CMakePresets.json' ||
      p === 'GETFUN_F722_FreeRTOS.ioc' || p === 'STM32F722XX_FLASH.ld' ||
      p === 'startup_stm32f722xx.s' || p === '.clangd' || p === '.mxproject' ||
      p === 'move-src.txt' || p.startsWith('cmake/') ||
      p.startsWith('.understand-anything/')) return 'layer:build-config';
  // Root docs
  if (p === 'README.md' || p === 'CLAUDE.md') return 'layer:docs';
  return null;
}

const layers = {};
let unassigned = [];
for (const f of files) {
  const L = layerOf(f);
  if (!L) { unassigned.push(f); continue; }
  (layers[L] = layers[L] || []).push(f.id);
}

console.log('=== LAYER COUNTS ===');
let total = 0;
for (const k of Object.keys(layers).sort()) {
  console.log(k + ': ' + layers[k].length);
  total += layers[k].length;
}
console.log('TOTAL assigned: ' + total + ' / ' + files.length);
if (unassigned.length) {
  console.log('UNASSIGNED (' + unassigned.length + '):');
  unassigned.forEach(f => console.log('   ' + f.id + '  [' + f.type + ']  ' + f.filePath));
}
