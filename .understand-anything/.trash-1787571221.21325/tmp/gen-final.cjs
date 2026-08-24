const fs = require('fs');
const { execSync } = require('child_process');
const g = JSON.parse(fs.readFileSync('.understand-anything/intermediate/assembled-graph.json', 'utf8'));
const layers = JSON.parse(fs.readFileSync('.understand-anything/intermediate/layers.json', 'utf8'));
const tour = JSON.parse(fs.readFileSync('.understand-anything/intermediate/tour.json', 'utf8'));
const final = {
  version: '1.0.0',
  project: {
    name: 'GETFUN_F722_FreeRTOS',
    languages: ['c'],
    frameworks: ['stm32-hal', 'freertos', 'cmsis-rtos'],
    description: '基于 STM32F722 + FreeRTOS 的飞控固件 APP 应用层（仅业务代码，已排除第三方/文档/构建配置）。',
    analyzedAt: new Date().toISOString(),
    gitCommitHash: execSync('git rev-parse HEAD').toString().trim(),
  },
  nodes: g.nodes,
  edges: g.edges,
  layers,
  tour,
};
fs.writeFileSync('.understand-anything/intermediate/assembled-graph.json', JSON.stringify(final, null, 2));
console.log('wrote final assembled graph:', final.nodes.length, 'nodes,', final.edges.length, 'edges,', final.layers.length, 'layers,', final.tour.length, 'tour steps');
