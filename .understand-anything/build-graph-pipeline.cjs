// Build a SYSTEM-LOGIC knowledge graph: 15 subsystem modules connected by
// data/control flow, laid out as a left-to-right pipeline by the dashboard's
// dagre layered layout. No file nodes, no #include edges — pure logic.
const fs = require("fs");
const path = require("path");

const OUT = path.resolve(__dirname, "knowledge-graph.json");

// --- Modules (subsystems) -------------------------------------------------
// files: source files that belong to this subsystem (shown in summary).
const MODULES = [
  {
    id: "app-init", name: "应用装配", stage: "support",
    summary: "APP 层入口：创建并初始化所有 RTOS 任务、app_state 与 USB 传输。",
    tags: ["rtos", "init", "entry-point"],
    files: ["APP/Src/rtos/app_task.c", "APP/Inc/rtos/app_task.h"],
  },
  {
    id: "imu-sampling", name: "IMU 采样", stage: "sense",
    summary: "ImuTask 是 SPI1/ICM42688P 唯一所有者，1kHz DRDY 门控 DMA 采样原始陀螺/加速度数据。",
    tags: ["imu", "sensor", "dma", "task"],
    files: ["APP/Src/rtos/imu_task.c", "APP/Inc/rtos/imu_task.h",
      "APP/Src/bsp/imu_bus.c", "APP/Inc/bsp/imu_bus.h",
      "APP/Src/drivers/icm42688p.c", "APP/Inc/drivers/icm42688p.h"],
  },
  {
    id: "time-base", name: "时间基准", stage: "sense",
    summary: "Cortex-M7 DWT 微秒时基，提供真实 dt 供采样门控、滤波与姿态积分。",
    tags: ["platform", "time", "dwt"],
    files: ["APP/Src/platform/platform_time.c", "APP/Inc/platform/platform_time.h"],
  },
  {
    id: "calibration", name: "校准", stage: "estimate",
    summary: "上电陀螺静态零偏 + 水平单面加速度校准（Welford 窗口），零偏持久化到 Flash。",
    tags: ["algorithms", "calibration", "gyro", "accel"],
    files: ["APP/Src/algorithms/gyro_calibration.c", "APP/Inc/algorithms/gyro_calibration.h",
      "APP/Src/algorithms/accel_calibration.c", "APP/Inc/algorithms/accel_calibration.h"],
  },
  {
    id: "filter", name: "低通滤波", stage: "estimate",
    summary: "三轴 PT1 低通，按真实 dt 更新（Gyro 100Hz / Accel 30Hz），首样本播种。",
    tags: ["algorithms", "filter", "pt1"],
    files: ["APP/Src/algorithms/imu_filter.c", "APP/Inc/algorithms/imu_filter.h"],
  },
  {
    id: "attitude", name: "姿态估计", stage: "estimate",
    summary: "Mahony 四元数姿态估计，消费滤波后陀螺/加速度，输出姿态四元数。",
    tags: ["algorithms", "attitude", "mahony", "quaternion"],
    files: ["APP/Src/algorithms/attitude_estimator.c", "APP/Inc/algorithms/attitude_estimator.h"],
  },
  {
    id: "rc-input", name: "RC 输入", stage: "control",
    summary: "CRSF 解析 + Linux 虚拟 RC 监控 + 物理/虚拟 RC 仲裁，产出最终 RC 快照。",
    tags: ["rc", "crsf", "arbiter", "task"],
    files: ["APP/Src/rtos/rc_task.c", "APP/Inc/rtos/rc_task.h",
      "APP/Src/bsp/crsf_uart.c", "APP/Inc/bsp/crsf_uart.h",
      "APP/Src/protocol/crsf.c", "APP/Inc/protocol/crsf.h",
      "APP/Src/bsp/linux_rc_monitor.c", "APP/Inc/bsp/linux_rc_monitor.h",
      "APP/Src/algorithms/rc_source_arbiter.c", "APP/Inc/algorithms/rc_source_arbiter.h",
      "APP/Src/algorithms/rc_input.c", "APP/Inc/algorithms/rc_input.h",
      "APP/Src/algorithms/rc_setpoint.c", "APP/Inc/algorithms/rc_setpoint.h"],
  },
  {
    id: "flight-control", name: "飞行控制", stage: "control",
    summary: "ANGLE 外环 + Rate PID + 四态安全链，消费姿态与 RC，产出电机控制量。",
    tags: ["flight", "pid", "arming", "task"],
    files: ["APP/Src/rtos/flight_task.c", "APP/Inc/rtos/flight_task.h",
      "APP/Src/algorithms/angle_outer_loop.c", "APP/Inc/algorithms/angle_outer_loop.h",
      "APP/Src/algorithms/rate_pid.c", "APP/Inc/algorithms/rate_pid.h",
      "APP/Src/algorithms/flight_arming.c", "APP/Inc/algorithms/flight_arming.h"],
  },
  {
    id: "mixer", name: "混控", stage: "control",
    summary: "Quad-X 混控，把 4 路控制量映射到 8 路电机端点。",
    tags: ["mixer", "quad-x"],
    files: ["APP/Src/algorithms/quad_x_mixer.c", "APP/Inc/algorithms/quad_x_mixer.h"],
  },
  {
    id: "motor-output", name: "电机输出", stage: "control",
    summary: "8 路电机安全输出（DShot/GPIO），只有 ARMED 才放行，Failsafe 强制低电平。",
    tags: ["motor", "safety", "dshot"],
    files: ["APP/Src/bsp/dshot_motor.c", "APP/Inc/bsp/dshot_motor.h"],
  },
  {
    id: "msp-usb", name: "MSP/USB 通信", stage: "interface",
    summary: "USB CDC 虚拟串口上的 MSP 协议服务器，向 Betaflight Configurator 提供遥测。",
    tags: ["msp", "usb", "cdc", "protocol"],
    files: ["APP/Src/bsp/usb_cdc_transport.c", "APP/Inc/bsp/usb_cdc_transport.h",
      "APP/Src/protocol/msp_transport.c", "APP/Inc/protocol/msp_transport.h",
      "APP/Src/protocol/msp_server.c", "APP/Inc/protocol/msp_server.h"],
  },
  {
    id: "param-store", name: "参数存储", stage: "interface",
    summary: "STM32F722 Sector 6/7 双槽参数存储，保存零偏/PID/Rate/Angle 等配置。",
    tags: ["storage", "flash", "eeprom"],
    files: ["APP/Src/storage/parameter_store.c", "APP/Inc/storage/parameter_store.h"],
  },
  {
    id: "app-state", name: "全局状态", stage: "interface",
    summary: "全局运行态快照（IMU/校准/滤波/姿态/RC/电池），关中断短临界区发布。",
    tags: ["state", "snapshot", "critical-section"],
    files: ["APP/Src/app_state.c", "APP/Inc/app_state.h"],
  },
  {
    id: "diag", name: "平台诊断", stage: "support",
    summary: "UART4 1Hz 基线诊断与安全停机，致命故障强制电机低电平。",
    tags: ["diag", "uart", "safety"],
    files: ["APP/Src/platform/platform_diag.c", "APP/Inc/platform/platform_diag.h"],
  },
  {
    id: "power", name: "电源监测", stage: "support",
    summary: "ADC3 电源监测 + BatteryTask，发布电池电压/电流到全局状态。",
    tags: ["power", "adc", "battery"],
    files: ["APP/Src/bsp/power_adc.c", "APP/Inc/bsp/power_adc.h",
      "APP/Src/algorithms/power_monitor.c", "APP/Inc/algorithms/power_monitor.h",
      "APP/Src/rtos/battery_task.c", "APP/Inc/rtos/battery_task.h"],
  },
];

// --- Edges (data / control flow) -----------------------------------------
// type: calls | depends_on | writes_to | reads_from
const EDGES = [
  // 装配：创建并启动各任务
  ["app-init", "imu-sampling", "calls", "创建并启动 ImuTask"],
  ["app-init", "rc-input", "calls", "创建并启动 RcTask"],
  ["app-init", "flight-control", "calls", "创建并启动 FlightTask"],
  ["app-init", "power", "calls", "创建并启动 BatteryTask"],
  ["app-init", "msp-usb", "calls", "创建并启动 MspTask"],
  ["app-init", "app-state", "calls", "初始化全局状态"],
  // 主信号链：采样 → 校准 → 滤波 → 姿态 → 飞行控制 → 混控 → 电机
  ["imu-sampling", "calibration", "depends_on", "原始样本送入校准"],
  ["calibration", "filter", "depends_on", "校准后数据送入低通"],
  ["filter", "attitude", "depends_on", "滤波后数据送入姿态估计"],
  ["attitude", "flight-control", "depends_on", "姿态送入 ANGLE 外环"],
  ["rc-input", "flight-control", "depends_on", "RC 设定值送入飞行控制"],
  ["flight-control", "mixer", "depends_on", "控制量送入混控"],
  ["mixer", "motor-output", "depends_on", "混控输出送入电机"],
  ["flight-control", "motor-output", "depends_on", "解锁门禁放行电机"],
  // 时间基准贯穿
  ["time-base", "imu-sampling", "depends_on", "提供采样真实 dt"],
  ["time-base", "filter", "depends_on", "提供滤波真实 dt"],
  ["time-base", "attitude", "depends_on", "提供姿态积分 dt"],
  // 全局状态：各模块发布，飞行控制与 MSP 读取
  ["imu-sampling", "app-state", "writes_to", "发布 IMU 快照"],
  ["calibration", "app-state", "writes_to", "发布校准状态"],
  ["filter", "app-state", "writes_to", "发布滤波数据"],
  ["power", "app-state", "writes_to", "发布电池信息"],
  ["app-state", "flight-control", "reads_from", "飞行控制读取状态"],
  ["app-state", "msp-usb", "reads_from", "MSP 读取遥测"],
  // 参数持久化
  ["calibration", "param-store", "writes_to", "保存零偏到 Flash"],
  ["param-store", "calibration", "reads_from", "启动加载零偏"],
  ["msp-usb", "param-store", "writes_to", "MSP 写配置到 Flash"],
  // 诊断安全停机
  ["diag", "motor-output", "depends_on", "安全停机强制电机低电平"],
];

// --- Layers (functional stages, for legend) ------------------------------
const STAGE_NAMES = {
  sense: "感知", estimate: "解算", control: "控制", interface: "接口/状态", support: "支撑",
};
const STAGE_DESC = {
  sense: "传感器采样与时间基准",
  estimate: "校准、低通与姿态解算",
  control: "RC、PID、混控与电机输出",
  interface: "MSP/USB、参数存储与全局状态",
  support: "应用装配、诊断与电源监测",
};

// --- Build nodes ----------------------------------------------------------
const nodes = MODULES.map((m) => ({
  id: `module:${m.id}`,
  type: "module",
  name: m.name,
  summary: m.summary + "\n文件: " + m.files.join(", "),
  tags: m.tags,
  complexity: "moderate",
  stage: m.stage,
}));

// --- Build edges ----------------------------------------------------------
const edges = EDGES.map(([s, t, type, label]) => ({
  source: `module:${s}`,
  target: `module:${t}`,
  type,
  direction: "forward",
  weight: type === "calls" ? 0.8 : type === "depends_on" ? 0.6 : 0.5,
  label,
}));

// --- Build layers (one per stage) -----------------------------------------
const stageIds = {};
for (const m of MODULES) (stageIds[m.stage] ||= []).push(`module:${m.id}`);
const layers = Object.keys(STAGE_NAMES).map((stage) => ({
  id: `layer:${stage}`,
  name: STAGE_NAMES[stage],
  description: STAGE_DESC[stage],
  nodeIds: stageIds[stage],
}));

// --- Tour (walkthrough of the main pipeline) ------------------------------
// Schema requires each tour step to be an object: { order, title, description, nodeIds }
const TOUR_ORDER = [
  "app-init", "imu-sampling", "time-base", "calibration", "filter",
  "attitude", "rc-input", "flight-control", "mixer", "motor-output",
  "app-state", "msp-usb", "param-store", "power", "diag",
];
const tour = TOUR_ORDER.map((id, i) => {
  const m = MODULES.find((x) => x.id === id);
  return {
    order: i,
    title: m.name,
    description: m.summary,
    nodeIds: [`module:${id}`],
  };
});

const graph = {
  version: "1.0.0",
  project: {
    name: "GETFUN_F722_FreeRTOS",
    languages: ["c"],
    frameworks: ["stm32-hal", "freertos", "cmsis-ros"],
    description: "基于 STM32F722 + FreeRTOS 的飞控固件 APP 应用层，按系统逻辑（数据流/控制流）组织的 15 子系统管道图谱。",
    analyzedAt: new Date().toISOString(),
    gitCommitHash: "a504227765e8c80120e29ac75c7e5148d47239ba",
  },
  nodes,
  edges,
  layers,
  tour,
};

fs.writeFileSync(OUT, JSON.stringify(graph, null, 2), "utf-8");

// quick stats
const nodeTypes = {};
const edgeTypes = {};
for (const n of nodes) nodeTypes[n.type] = (nodeTypes[n.type] || 0) + 1;
for (const e of edges) edgeTypes[e.type] = (edgeTypes[e.type] || 0) + 1;
console.log(`nodes: ${nodes.length} edges: ${edges.length} layers: ${layers.length} tour: ${tour.length}`);
console.log("nodeTypes:", JSON.stringify(nodeTypes));
console.log("edgeTypes:", JSON.stringify(edgeTypes));
console.log("written:", OUT);
