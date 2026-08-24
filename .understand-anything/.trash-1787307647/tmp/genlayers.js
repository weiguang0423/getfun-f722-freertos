'use strict';
const fs = require('fs');
const inPath = process.argv[2];
const outPath = process.argv[3];
const data = JSON.parse(fs.readFileSync(inPath, 'utf8'));
const files = data.files;

function layerOf(f) {
  const p = f.filePath;
  if (p.startsWith('APP/')) return 'layer:app';
  if (p.startsWith('Core/')) return 'layer:cubemx-app';
  if (p.startsWith('USB_DEVICE/')) return 'layer:cubemx-app';
  if (p.startsWith('Drivers/STM32F7xx_HAL_Driver/')) return 'layer:hal';
  if (p.startsWith('Drivers/CMSIS/')) return 'layer:cmsis';
  if (p.startsWith('Middlewares/Third_Party/FreeRTOS/')) return 'layer:freertos';
  if (p.startsWith('Middlewares/ST/')) return 'layer:usb-device';
  if (p.startsWith('Linux/RK3568/')) return 'layer:rk3568';
  if (p.startsWith('Tools/')) return 'layer:tools';
  if (p === 'CMakeLists.txt' || p === 'CMakePresets.json' ||
      p === 'GETFUN_F722_FreeRTOS.ioc' || p === 'STM32F722XX_FLASH.ld' ||
      p === 'startup_stm32f722xx.s' || p === '.clangd' || p === '.mxproject' ||
      p === 'move-src.txt' || p.startsWith('cmake/') ||
      p.startsWith('.understand-anything/')) return 'layer:build-config';
  if (p === 'README.md' || p === 'CLAUDE.md') return 'layer:docs';
  return null;
}

const layerMeta = {
  'layer:app': {
    name: 'APP 应用层',
    description: '项目自研应用层（STM32 侧）：MSP 协议服务器、IMU/校准/滤波算法、PID/Mixer/安全解锁、RC 仲裁、USB CDC 传输、参数存储、平台时基/诊断与 RTOS 任务'
  },
  'layer:cubemx-app': {
    name: 'CubeMX 应用生成层',
    description: 'CubeMX 生成的固件骨架：main.c 入口、freertos.c 任务、GPIO/SPI/UART 初始化，以及 USB_DEVICE CDC 接口（usbd_cdc_if.c 转发 RX 给 usb_cdc_transport）'
  },
  'layer:hal': {
    name: 'HAL 驱动层',
    description: 'ST STM32F7xx HAL/LL 外设驱动库（GPIO/DMA/SPI/UART/PCD/TIM 等），CubeMX 生成并随芯片 HAL 版本升级'
  },
  'layer:cmsis': {
    name: 'CMSIS 层',
    description: 'ARM CMSIS-Core 头文件、Cortex-M7 内核访问接口与 STM32F7xx 设备定义（含启动/向量与 LICENSE）'
  },
  'layer:freertos': {
    name: 'FreeRTOS 内核层',
    description: 'FreeRTOS Kernel V10.2.1 内核与 CMSIS-RTOS V2 封装（含 heap_4、静态/动态分配、软件定时器）'
  },
  'layer:usb-device': {
    name: 'ST USB 设备库层',
    description: 'ST STM32 USB Device 核心与 CDC 类库（USB 内核、CDC 类实现及 LICENSE），被 CubeMX 的 USB_DEVICE 调用'
  },
  'layer:rk3568': {
    name: 'RK3568 伴随计算机',
    description: 'RK3568 Linux 伴随计算机源码（同仓）：S7.x 各阶段 RKNN 模型转换、ARM64 板端推理 C++、手部跟踪/手势管线、虚拟 RC 序列化、交叉构建脚本、验证产物（JSON/JSONL）与文档'
  },
  'layer:tools': {
    name: '工具与验收脚本',
    description: '离线回归验证工具（Tools/*.mjs、*.ps1 等），静态读取固件源码做构建外的一致性/验收检查'
  },
  'layer:build-config': {
    name: '构建与配置',
    description: '构建与配置：根 CMakeLists.txt、CMakePresets.json、cmake/ 目录、链接脚本 STM32F722XX_FLASH.ld、启动文件 startup_stm32f722xx.s、CubeMX 项目 .ioc、.clangd/.mxproject/move-src.txt 及 understand-anything 配置'
  },
  'layer:docs': {
    name: '文档层',
    description: '仓库根级文档：README.md 项目说明与 CLAUDE.md 开发/架构约束指南'
  }
};

const layers = {};
let unassigned = [];
for (const f of files) {
  const L = layerOf(f);
  if (!L) { unassigned.push(f.id); continue; }
  (layers[L] = layers[L] || []).push(f.id);
}

if (unassigned.length) {
  console.error('UNASSIGNED: ' + unassigned.length);
  process.exit(1);
}

const out = Object.keys(layerMeta).map(id => ({
  id,
  name: layerMeta[id].name,
  description: layerMeta[id].description,
  nodeIds: layers[id] || []
})).filter(l => l.nodeIds.length > 0);

// safety: verify every file present exactly once
const seen = new Set();
for (const l of out) for (const n of l.nodeIds) seen.add(n);
if (seen.size !== files.length) {
  console.error('MISMATCH: ' + seen.size + ' vs ' + files.length);
  process.exit(1);
}

fs.writeFileSync(outPath, JSON.stringify(out, null, 2), 'utf8');
console.log('Wrote', outPath, 'with', out.length, 'layers,', files.length, 'nodes');
out.forEach(l => console.log('  ' + l.id + ' (' + l.name + '): ' + l.nodeIds.length));
