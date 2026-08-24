// Phase 2 graph builder: reads extract results + batches, emits batch-<i>.json
// Focus: APP/ C code only (docs/build/config already excluded by .understandignore)
const fs = require('fs');
const path = require('path');
const root = process.cwd();
const batches = JSON.parse(fs.readFileSync('.understand-anything/intermediate/batches.json', 'utf8'));

// ---- curated Chinese descriptions for APP modules ----
const DESC = {
  'APP/Src/rtos/app_task.c': ['应用任务初始化入口', '提供 app_tasks_init()：初始化 app_state 与 usb_cdc_transport，并静态创建 ImuTask 与 MspTask，是 APP 层 RTOS 任务的装配点。', ['entry-point', 'rtos', 'initialization']],
  'APP/Src/rtos/imu_task.c': ['IMU 采样单写者任务', 'ImuTask：SPI1/ICM42688P 唯一所有者，负责 1kHz DRDY 门控 DMA 采样、时间扩展、SI/CW90 变换、校准、PT1 低通、参数提交与 app_state 发布。', ['rtos', 'imu', 'sensor', 'task']],
  'APP/Src/rtos/flight_task.c': ['飞行控制任务', 'FlightTask：以 1kHz 消费动态 RC/PID/Angle 参数，ANGLE 外环把姿态角误差转为 Rate 设定、Yaw 透传，四态安全链仅在 ARMED 时把混控映射到电机端点。', ['rtos', 'flight-control', 'task']],
  'APP/Src/rtos/rc_task.c': ['RC 解析任务', 'RcTask：物理 CRSF 解析与最终 RC 快照单写者，消费 USART6 候选并只把仲裁后的 mapped_channel_us 交给控制链。', ['rtos', 'rc', 'task']],
  'APP/Src/rtos/battery_task.c': ['电池监测任务', 'BatteryTask：周期性读取电源监测结果并发布电池电压/电流/剩余量到 app_state。', ['rtos', 'battery', 'task']],
  'APP/Src/protocol/msp_transport.c': ['MSP 协议层', '纯协议层（无 RTOS 依赖）：msp_parser_t 逐字节状态机解析 $M< V1 / $X< V2，msp_transport_build_response() 构造回包帧。', ['protocol', 'msp', 'parser']],
  'APP/Src/protocol/msp_server.c': ['MSP 命令派发', 'MSP 命令派发：按 command 路由，返回 FC_VARIANT/BOARD_INFO，并实现 PID/Modes/Motors/Configuration 最小标准读写，SET 经私有事务提交 EEPROM，写操作受 Armed/安全门禁约束。', ['protocol', 'msp', 'server']],
  'APP/Src/protocol/crsf.c': ['CRSF 协议', 'Crossfire 遥控协议编解码：帧解析、通道解算与 CRC 校验，供物理 RC 链路使用。', ['protocol', 'crsf', 'rc']],
  'APP/Src/bsp/imu_bus.c': ['SPI1 总线适配', 'SPI1 + PA4 CS + DMA2 总线适配层，封装 ICM42688P 的阻塞事务与 14 字节 DMA 样本传输。', ['bsp', 'spi', 'driver']],
  'APP/Src/drivers/icm42688p.c': ['ICM42688P 寄存器驱动', 'ICM42688P 寄存器级驱动：初始化、量程/ODR 配置、FIFO/DRDY 读取与原始样本转换。', ['driver', 'imu', 'sensor']],
  'APP/Src/algorithms/gyro_calibration.c': ['陀螺静态零偏校准', '上电陀螺静态零偏状态机：预热、Welford 均值/方差窗口，运动或异常时重置，READY 后冻结并扣除三轴零偏。', ['algorithm', 'calibration', 'gyro']],
  'APP/Src/algorithms/accel_calibration.c': ['加速度校准', '水平单面加速度校准：预热、Welford 窗口与运动/水平/方差/偏置门限，候选值持久化成功前不切换。', ['algorithm', 'calibration', 'accel']],
  'APP/Src/algorithms/imu_filter.c': ['IMU 低通滤波', '三轴 PT1 低通：按真实 dt 更新，Gyro 100Hz / Accel 30Hz，首样本播种，dt 超界或非有限值重置。', ['algorithm', 'filter', 'pt1']],
  'APP/Src/platform/platform_time.c': ['DWT 微秒时基', 'Cortex-M7 DWT 微秒时基：ISR 只读 CYCCNT，ImuTask 单写者做 32 位回绕扩展与周期余数累计。', ['platform', 'time', 'dwt']],
  'APP/Src/platform/platform_diag.c': ['平台诊断与安全停机', '平台基线诊断与安全停机：UART4 1Hz 输出摘要，致命故障或参数保存前强制 Motor 1~8 低电平。', ['platform', 'diag', 'safety']],
  'APP/Src/storage/parameter_store.c': ['参数持久化存储', 'STM32F722 Sector 6/7 双槽参数存储：保存 v2（偏置/RC/Rate PID/Angle/motor_idle/名称），兼容 v1 迁移，只擦除非活动槽。', ['storage', 'flash', 'parameter']],
  'APP/Src/algorithms/angle_outer_loop.c': ['ANGLE 外环', '姿态角外环：Roll/Pitch 角误差转 Rate 设定、Yaw 透传，模式/参数切换复位 PID。', ['algorithm', 'attitude', 'angle']],
  'APP/Src/algorithms/rate_pid.c': ['Rate PID 控制器', '角速率 PID：对三轴角速率误差做 PID 计算并输出混控前控制量。', ['algorithm', 'pid', 'rate']],
  'APP/Src/algorithms/quad_x_mixer.c': ['Quad-X 混控器', 'Quad-X 混控矩阵：把控制量映射到 4 路电机输出，含油门与方向分配。', ['algorithm', 'mixer', 'quad-x']],
  'APP/Src/algorithms/flight_arming.c': ['解锁安全链', '四态安全解锁链：综合 IMU/校准/参数/RC 门禁，只有 ARMED 才允许电机输出。', ['algorithm', 'arming', 'safety']],
  'APP/Src/algorithms/rc_input.c': ['RC 输入', 'RC 输入抽象：把物理/虚拟候选通道映射为控制链使用的 mapped_channel_us。', ['algorithm', 'rc', 'input']],
  'APP/Src/algorithms/rc_setpoint.c': ['RC 设定值', 'RC 设定值生成：由映射通道解算目标姿态角速率/油门设定。', ['algorithm', 'rc', 'setpoint']],
  'APP/Src/algorithms/rc_source_arbiter.c': ['RC 源仲裁', '物理/虚拟 RC 仲裁：AUX3 授权，物理 AUX 保持所有权，超时/会话重启/inhibit/DISARM 立即退出且禁止自动重入。', ['algorithm', 'rc', 'arbiter']],
  'APP/Src/bsp/linux_rc_monitor.c': ['Linux RC 监视', 'USART6 ISR 固定帧接收器：校验 CRC/格式/范围/心跳，向 RcTask 提供原子候选快照。', ['bsp', 'rc', 'linux', 'uart']],
  'APP/Src/bsp/crsf_uart.c': ['CRSF UART 驱动', 'CRSF 串口驱动：物理 RC 链路字节收发与帧边界检测。', ['bsp', 'crsf', 'uart']],
  'APP/Src/bsp/dshot_motor.c': ['DShot 电机驱动', 'DShot 电调协议输出：把电机端点值编码为 DShot 帧并通过定时器 DMA 发送。', ['bsp', 'motor', 'dshot']],
  'APP/Src/bsp/power_adc.c': ['电源 ADC 驱动', 'ADC3 电源采样驱动：读取电池电压/电流原始值。', ['bsp', 'adc', 'power']],
  'APP/Src/algorithms/power_monitor.c': ['电源监测', '电源监测算法：由 ADC 原始值换算电压/电流/剩余量并做滤波。', ['algorithm', 'power', 'monitor']],
  'APP/Src/app_state.c': ['全局运行态快照', 'app_state 全局快照：聚合 IMU 时间/dt、未滤波/滤波数据、校准、参数槽、解锁抑制与姿态/电池信息，关中断短临界区发布。', ['state', 'snapshot', 'shared']],
  'APP/Src/bsp/usb_cdc_transport.c': ['USB CDC 传输', 'USB CDC 收发适配：1024 字节 RX 环形缓冲 + 任务通知，TX 轮询等待 CDC_Transmit_FS 完成。', ['bsp', 'usb', 'cdc', 'transport']],
  'APP/Inc/app_state.h': ['运行态快照头文件', '定义 app_state_snapshot_t 全局运行态结构与发布/读取接口。', ['type-definition', 'state']],
  'APP/Inc/rtos/app_task.h': ['应用任务头文件', '声明 app_tasks_init() 与静态任务句柄。', ['type-definition', 'rtos']],
  'APP/Inc/rtos/imu_task.h': ['IMU 任务头文件', '声明 ImuTask 入口与初始化接口。', ['type-definition', 'rtos']],
  'APP/Inc/rtos/flight_task.h': ['飞行任务头文件', '声明 FlightTask 入口。', ['type-definition', 'rtos']],
  'APP/Inc/rtos/rc_task.h': ['RC 任务头文件', '声明 RcTask 入口。', ['type-definition', 'rtos']],
  'APP/Inc/rtos/battery_task.h': ['电池任务头文件', '声明 BatteryTask 入口。', ['type-definition', 'rtos']],
  'APP/Inc/protocol/msp_transport.h': ['MSP 协议头文件', '定义 msp_parser_t 状态机与帧构造接口。', ['type-definition', 'protocol']],
  'APP/Inc/protocol/msp_server.h': ['MSP 服务器头文件', '声明 MSP 命令派发接口。', ['type-definition', 'protocol']],
  'APP/Inc/protocol/crsf.h': ['CRSF 协议头文件', '定义 CRSF 帧结构与通道解算接口。', ['type-definition', 'protocol']],
  'APP/Inc/bsp/imu_bus.h': ['SPI 总线头文件', '声明 SPI1/ICM42688P 总线接口。', ['type-definition', 'bsp']],
  'APP/Inc/drivers/icm42688p.h': ['IMU 驱动头文件', '定义 ICM42688P 寄存器与配置接口。', ['type-definition', 'driver']],
  'APP/Inc/bsp/crsf_uart.h': ['CRSF UART 头文件', '声明 CRSF 串口接口。', ['type-definition', 'bsp']],
  'APP/Inc/bsp/dshot_motor.h': ['DShot 电机头文件', '声明 DShot 电机输出接口。', ['type-definition', 'bsp']],
  'APP/Inc/bsp/linux_rc_monitor.h': ['Linux RC 监视头文件', '声明 USART6 候选快照接口。', ['type-definition', 'bsp']],
  'APP/Inc/bsp/power_adc.h': ['电源 ADC 头文件', '声明 ADC3 采样接口。', ['type-definition', 'bsp']],
  'APP/Inc/bsp/usb_cdc_transport.h': ['USB CDC 传输头文件', '声明环形缓冲收发接口。', ['type-definition', 'bsp']],
  'APP/Inc/platform/platform_diag.h': ['平台诊断头文件', '声明诊断输出与安全停机接口。', ['type-definition', 'platform']],
  'APP/Inc/platform/platform_time.h': ['时基头文件', '声明 DWT 微秒时基接口。', ['type-definition', 'platform']],
  'APP/Inc/storage/parameter_store.h': ['参数存储头文件', '定义双槽参数结构与读写接口。', ['type-definition', 'storage']],
  'APP/Inc/algorithms/accel_calibration.h': ['加速度校准头文件', '声明加速度校准状态机接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/angle_outer_loop.h': ['ANGLE 外环头文件', '声明姿态角外环接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/attitude_estimator.h': ['姿态估计头文件', '声明 Mahony 四元数姿态估计接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/flight_arming.h': ['解锁头文件', '声明四态安全解锁接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/gyro_calibration.h': ['陀螺校准头文件', '声明陀螺零偏校准接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/imu_filter.h': ['低通头文件', '声明 PT1 低通接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/power_monitor.h': ['电源监测头文件', '声明电源监测接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/quad_x_mixer.h': ['混控头文件', '声明 Quad-X 混控接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/rate_pid.h': ['Rate PID 头文件', '声明角速率 PID 接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/rc_input.h': ['RC 输入头文件', '声明 RC 输入映射接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/rc_setpoint.h': ['RC 设定值头文件', '声明设定值生成接口。', ['type-definition', 'algorithm']],
  'APP/Inc/algorithms/rc_source_arbiter.h': ['RC 仲裁头文件', '声明 RC 源仲裁接口。', ['type-definition', 'algorithm']],
};

// ---- load all extract results ----
const results = [];
for (const batch of batches.batches) {
  const rp = `.understand-anything/tmp/ua-file-extract-results-${batch.batchIndex}.json`;
  if (fs.existsSync(rp)) {
    const r = JSON.parse(fs.readFileSync(rp, 'utf8'));
    for (const res of r.results) results.push(res);
  }
}

const fnMap = new Map();
const fileFns = new Map();
for (const res of results) {
  const set = new Set();
  for (const fn of (res.functions || [])) set.add(fn.name);
  fileFns.set(res.path, set);
  for (const fn of (res.functions || [])) {
    const exp = (res.exports || []).some(e => e.name === fn.name);
    if (!fnMap.has(fn.name)) fnMap.set(fn.name, []);
    fnMap.get(fn.name).push({ file: res.path, exported: exp });
  }
}

const callNames = new Set();
for (const res of results) {
  for (const cg of (res.callGraph || [])) { callNames.add(cg.caller); callNames.add(cg.callee); }
}
function fnSignificant(res, fn) {
  const lines = (fn.endLine || 0) - (fn.startLine || 0);
  const exp = (res.exports || []).some(e => e.name === fn.name);
  return lines >= 10 || exp || callNames.has(fn.name);
}

const nodes = [];
const edges = [];
const seenNode = new Set();
function addNode(n) {
  if (seenNode.has(n.id)) return;
  seenNode.add(n.id);
  nodes.push(n);
}
function complexityOf(res) {
  const ne = res.nonEmptyLines || res.totalLines || 0;
  if (ne < 50) return 'simple';
  if (ne <= 200) return 'moderate';
  return 'complex';
}

// file nodes (all APP/ files are code)
for (const res of results) {
  const p = res.path;
  const d = DESC[p] || null;
  const name = p.split('/').pop();
  const summary = d ? d[1] : `${name}：C 源文件，含 ${((res.functions||[]).length)} 个函数。`;
  const tags = d ? d[2] : ['module', 'code'];
  addNode({ id: `file:${p}`, type: 'file', name, filePath: p, summary, tags, complexity: complexityOf(res) });
}

// function nodes + contains + exports
for (const res of results) {
  const p = res.path;
  for (const fn of (res.functions || [])) {
    if (!fnSignificant(res, fn)) continue;
    const id = `function:${p}:${fn.name}`;
    const lines = (fn.endLine || 0) - (fn.startLine || 0);
    addNode({
      id, type: 'function', name: fn.name, filePath: p,
      summary: `${fn.name}()：${DESC[p] ? DESC[p][0] : p.split('/').pop()} 中的函数（约 ${lines} 行）。`,
      tags: ['function', 'code'],
      complexity: lines >= 10 ? (lines > 50 ? 'moderate' : 'simple') : 'simple',
    });
    edges.push({ source: `file:${p}`, target: id, type: 'contains', direction: 'forward', weight: 1.0 });
  }
  for (const exp of (res.exports || [])) {
    const id = `function:${p}:${exp.name}`;
    if (seenNode.has(id)) edges.push({ source: `file:${p}`, target: id, type: 'exports', direction: 'forward', weight: 0.8 });
  }
}

// imports edges from #include "..." (resolve to APP/Inc/<path>)
const incRe = /#include\s+"([^"]+)"/g;
for (const res of results) {
  if (!res.path.startsWith('APP/')) continue;
  let content;
  try { content = fs.readFileSync(res.path, 'utf8'); } catch { continue; }
  let m;
  const seen = new Set();
  while ((m = incRe.exec(content)) !== null) {
    const inc = m[1];
    if (seen.has(inc)) continue;
    seen.add(inc);
    const candidates = [`APP/Inc/${inc}`, inc, path.dirname(res.path) + '/' + inc];
    let tgt = null;
    for (const c of candidates) { if (fs.existsSync(c)) { tgt = c; break; } }
    if (tgt) edges.push({ source: `file:${res.path}`, target: `file:${tgt}`, type: 'imports', direction: 'forward', weight: 0.7 });
  }
}

// calls edges (cross-file) from callGraph
for (const res of results) {
  const p = res.path;
  for (const cg of (res.callGraph || [])) {
    const callee = cg.callee;
    const sameFile = (fileFns.get(p) || new Set()).has(callee);
    if (sameFile) continue;
    const candidates = fnMap.get(callee);
    if (!candidates || candidates.length === 0) continue;
    let target = candidates.find(c => c.file !== p && c.exported);
    if (!target) target = candidates.find(c => c.file !== p);
    if (!target) continue;
    const srcId = `function:${p}:${cg.caller}`;
    const tgtId = `function:${target.file}:${callee}`;
    if (seenNode.has(srcId) && seenNode.has(tgtId)) {
      edges.push({ source: srcId, target: tgtId, type: 'calls', direction: 'forward', weight: 0.8 });
    }
  }
}

// ---- write per-batch output ----
const batchFiles = new Map();
for (const batch of batches.batches) {
  batchFiles.set(batch.batchIndex, new Set(batch.files.map(f => f.path)));
}
function nodeBatch(n) {
  for (const [bi, set] of batchFiles) if (set.has(n.filePath)) return bi;
  return batches.batches[0].batchIndex;
}
const out = new Map();
for (const batch of batches.batches) out.set(batch.batchIndex, { nodes: [], edges: [] });
for (const n of nodes) out.get(nodeBatch(n)).nodes.push(n);
for (const e of edges) {
  let sp = e.source;
  if (sp.startsWith('function:')) sp = sp.slice('function:'.length).split(':').slice(0, -1).join(':');
  else if (sp.startsWith('file:')) sp = sp.slice('file:'.length);
  let bi = batches.batches[0].batchIndex;
  for (const [b, set] of batchFiles) if (set.has(sp)) { bi = b; break; }
  out.get(bi).edges.push(e);
}
for (const [bi, obj] of out) {
  fs.writeFileSync(`.understand-anything/intermediate/batch-${bi}.json`, JSON.stringify(obj, null, 2));
}
console.log('nodes:', nodes.length, 'edges:', edges.length);
for (const [bi, obj] of out) console.log(`batch ${bi}: nodes=${obj.nodes.length} edges=${obj.edges.length}`);
