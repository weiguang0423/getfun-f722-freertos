const fs = require('fs');
const g = JSON.parse(fs.readFileSync('.understand-anything/intermediate/assembled-graph.json', 'utf8'));
const fileLevel = new Set(['file', 'config', 'document', 'service', 'pipeline', 'table', 'schema', 'resource', 'endpoint']);
const fileNodes = g.nodes.filter(n => fileLevel.has(n.type));

function layerFor(p) {
  if (p.startsWith('APP/Src/rtos/') || p.startsWith('APP/Inc/rtos/')) return 'layer:app-rtos';
  if (p.startsWith('APP/Src/algorithms/') || p.startsWith('APP/Inc/algorithms/')) return 'layer:app-algorithms';
  if (p.startsWith('APP/Src/bsp/') || p.startsWith('APP/Inc/bsp/') || p.startsWith('APP/Src/drivers/') || p.startsWith('APP/Inc/drivers/')) return 'layer:app-bsp';
  if (p.startsWith('APP/Src/protocol/') || p.startsWith('APP/Inc/protocol/')) return 'layer:app-protocol';
  if (p.startsWith('APP/Src/platform/') || p.startsWith('APP/Inc/platform/')) return 'layer:app-platform';
  if (p.startsWith('APP/Src/storage/') || p.startsWith('APP/Inc/storage/')) return 'layer:app-storage';
  if (p === 'APP/Src/app_state.c' || p === 'APP/Inc/app_state.h') return 'layer:app-state';
  return 'layer:app-misc';
}

const groups = {};
for (const n of fileNodes) {
  const l = layerFor(n.filePath || n.id.replace(/^(file|config|document):/, ''));
  if (!groups[l]) groups[l] = [];
  groups[l].push(n.id);
}

const meta = {
  'layer:app-rtos': ['RTOS 任务层', 'FreeRTOS 静态任务：应用装配、IMU 采样、飞行控制、RC 解析、电池监测。'],
  'layer:app-algorithms': ['控制算法层', '姿态/校准/滤波/PID/混控/解锁/RC 等不依赖 HAL/RTOS 的纯算法模块。'],
  'layer:app-bsp': ['板级支持与驱动层', 'SPI 总线、ICM42688P 寄存器驱动、CRSF/DShot/电源/USB CDC 等外设适配。'],
  'layer:app-protocol': ['通信协议层', 'MSP 逐字节状态机与命令派发、CRSF 遥控协议编解码。'],
  'layer:app-platform': ['平台基础设施层', 'DWT 微秒时基与诊断/安全停机。'],
  'layer:app-storage': ['参数持久化层', 'Sector 6/7 双槽 Flash 参数存储与迁移。'],
  'layer:app-state': ['全局运行态', 'app_state 快照：跨任务共享的短临界区发布结构。'],
  'layer:app-misc': ['其他', '未分类文件。'],
};

const layers = Object.keys(meta).filter(l => groups[l] && groups[l].length).map(l => ({
  id: l, name: meta[l][0], description: meta[l][1], nodeIds: groups[l],
}));
if (groups['layer:app-misc'] && groups['layer:app-misc'].length) {
  layers.push({ id: 'layer:app-misc', name: meta['layer:app-misc'][0], description: meta['layer:app-misc'][1], nodeIds: groups['layer:app-misc'] });
}

fs.writeFileSync('.understand-anything/intermediate/layers.json', JSON.stringify(layers, null, 2));
console.log('layers:', layers.length);
layers.forEach(l => console.log(`  ${l.id}: ${l.nodeIds.length} nodes`));
const total = layers.reduce((a, l) => a + l.nodeIds.length, 0);
console.log('file nodes assigned:', total, '/', fileNodes.length);
