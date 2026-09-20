const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {stages, scenarios, move} = require('./model.js');
const hardware = require('./hardware.js');
assert.equal(hardware.scenes.length, 7);
assert.equal(hardware.next(6), 6, 'hardware playback must stop at the final scene');
for (const scene of hardware.scenes) {
  for (const route of scene.routes) assert.ok(hardware.routes[route], `unknown route: ${route}`);
  for (const component of scene.active) assert.ok(['sensor','cpu','spi','dma','ram'].includes(component));
}
assert.deepEqual(hardware.scenes[2].routes.filter(id => hardware.routes[id].kind === 'tx'), ['mosi','txSpi','ramTx']);
assert.deepEqual(hardware.scenes[2].routes.filter(id => hardware.routes[id].kind === 'rx'), ['miso','rxDma','dmaRam']);
assert.ok(!hardware.scenes[2].active.includes('cpu'), 'DMA transfer must not imply per-byte CPU copying');
assert.ok(!hardware.scenes[3].routes.includes('readRx'), 'IRQ is a notification, not the sample payload');
assert.equal(stages.length, 9);
assert.equal(move(0, -1), 0, 'previous must stay at the start');
assert.equal(move(8, 1), 8, 'next must stop at the consumer');
let stage = 0;
for (let i = 0; i < 8; i++) stage = move(stage, 1);
assert.equal(stage, 8);
assert.equal(scenarios.drdy.step, 0, 'no DRDY must stop before DMA');
assert.equal(scenarios.timeout.step, 1, 'timeout must stop at the DMA transaction');
assert.equal(scenarios.stale.step, 8, 'age gate belongs to the consumer');
for (const s of stages) for (const key of ['title','owner','input','output','text','why','detail','source']) assert.ok(s[key]);
const html = fs.readFileSync(path.join(__dirname, 'index.html'), 'utf8');
for (const match of html.matchAll(/<script>([\s\S]*?)<\/script>/g)) new vm.Script(match[1]);
for (const match of html.matchAll(/(?:src|href)="([^"#]+)"/g)) {
  assert.ok(!/^https?:/.test(match[1]), 'page must work without network assets');
  assert.ok(fs.existsSync(path.join(__dirname, match[1])), `missing asset: ${match[1]}`);
}
const root = path.resolve(__dirname, '../..');
const task = fs.readFileSync(path.join(root, 'APP/Src/rtos/imu_task.c'), 'utf8');
const flight = fs.readFileSync(path.join(root, 'APP/Src/rtos/flight_task.c'), 'utf8');
assert.match(task, /IMU_TASK_PERIOD_TICKS\s+pdMS_TO_TICKS\(1U\)/);
assert.match(task, /IMU_DMA_TIMEOUT_TICKS\s+pdMS_TO_TICKS\(2U\)/);
assert.match(task, /IMU_MAX_CONSECUTIVE_ERRORS\s+3U/);
assert.match(flight, /FLIGHT_IMU_TIMEOUT_TICKS\s+pdMS_TO_TICKS\(5U\)/);
console.log('PASS: navigation boundaries, full journey, failure ownership, explanation completeness, script syntax, offline assets, source timing constants');
