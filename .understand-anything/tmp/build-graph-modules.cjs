// Module-centric graph builder: groups APP/ files into logical subsystems
// and connects subsystems by actual data/control flow (system logic),
// NOT by C #include dependencies. Replaces the file-spaghetti view.
const fs = require('fs');
const { execSync } = require('child_process');

const scan = JSON.parse(fs.readFileSync('.understand-anything/intermediate/scan-result.json', 'utf8'));
const fileSizes = {};
scan.files.forEach(f => { fileSizes[f.path] = f.sizeLines || 0; });

// file descriptions (curated from CLAUDE.md)
const DESC = {
  'APP/Src/rtos/app_task.c': ['应用任务初始化入口', 'app_tasks_init()：初始化 app_state 与 usb_cdc_transport，并静态创建 ImuTask / MspTask / RcTask / FlightTask / BatteryTask。', ['entry-point', 'rtos', 'initialization']],
  'APP/Src/rtos/imu_task.c': ['IMU 采样单写者任务', 'ImuTask：SPI1/ICM42688P 唯一所有者，1kHz DRDY 门控 DMA 采样、时间扩展、SI/CW90、校准、PT1 低通、参数提交与 app_state 发布。', ['rtos', 'imu', 'sensor', 'task']],
  'APP/Src/rtos/flight_task.c': ['飞行控制任务', 'FlightTask：1kHz 消费动态 RC/PID/Angle，ANGLE 外环转 Rate 设定、Yaw 透传，四态安全链仅 ARMED 时映射混控到电机。', ['rtos', 'flight-control', 'task']],
  'APP/Src/rtos/rc_task.c': ['RC 解析任务', 'RcTask：物理 CRSF 解析与最终 RC 快照单写者，消费 USART6 候选并只把仲裁后 mapped_channel_us 交控制链。', ['rtos', 'rc', 'task']],
  'APP/Src/rtos/battery_task.c': ['电池监测任务', 'BatteryTask：周期读取电源监测结果并发布电池电压/电流/剩余量到 app_state。', ['rtos', 'battery', 'task']],
  'APP/Src/protocol/msp_transport.c': ['MSP 协议层', '纯协议层：msp_parser_t 逐字节状态机解析 $M< V1 / $X< V2，构造回包帧。', ['protocol', 'msp', 'parser']],
  'APP/Src/protocol/msp_server.c': ['MSP 命令派发', 'MSP 命令派发：按 command 路由，实现 PID/Modes/Motors/Configuration 最小标准读写，写操作受 Armed/安全门禁约束。', ['protocol', 'msp', 'server']],
  'APP/Src/protocol/crsf.c': ['CRSF 协议', 'Crossfire 遥控协议编解码：帧解析、通道解算与 CRC 校验。', ['protocol', 'crsf', 'rc']],
  'APP/Src/bsp/imu_bus.c': ['SPI1 总线适配', 'SPI1 + PA4 CS + DMA2 总线适配层，封装 ICM42688P 阻塞事务与 14 字节 DMA 样本传输。', ['bsp', 'spi', 'driver']],
  'APP/Src/drivers/icm42688p.c': ['ICM42688P 寄存器驱动', 'ICM42688P 寄存器级驱动：初始化、量程/ODR 配置、FIFO/DRDY 读取与原始样本转换。', ['driver', 'imu', 'sensor']],
  'APP/Src/algorithms/gyro_calibration.c': ['陀螺静态零偏校准', '上电陀螺静态零偏状态机：预热、Welford 均值/方差窗口，运动或异常时重置，READY 后冻结并扣除三轴零偏。', ['algorithm', 'calibration', 'gyro']],
  'APP/Src/algorithms/accel_calibration.c': ['加速度校准', '水平单面加速度校准：预热、Welford 窗口与运动/水平/方差/偏置门限，候选值持久化成功前不切换。', ['algorithm', 'calibration', 'accel']],
  'APP/Src/algorithms/imu_filter.c': ['IMU 低通滤波', '三轴 PT1 低通：按真实 dt 更新，Gyro 100Hz / Accel 30Hz，首样本播种，dt 超界或非有限值重置。', ['algorithm', 'filter', 'pt1']],
  'APP/Src/platform/platform_time.c': ['DWT 微秒时基', 'Cortex-M7 DWT 微秒时基：ISR 只读 CYCCNT，ImuTask 单写者做 32 位回绕扩展与周期余数累计。', ['platform', 'time', 'dwt']],
  'APP/Src/platform/platform_diag.c': ['平台诊断与安全停机', '平台基线诊断与安全停机：UART4 1Hz 输出摘要，致命故障或参数保存前强制 Motor 1~8 低电平。', ['platform', 'diag', 'safety']],
  'APP/Src/storage/parameter_store.c': ['参数持久化存储', 'STM32F722 Sector 6/7 双槽参数存储：保存 v2（偏置/RC/Rate PID/Angle/motor_idle/名称），兼容 v1 迁移，只擦非活动槽。', ['storage', 'flash', 'parameter']],
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

// ---- logical subsystems (modules) ----
const MODULES = [
  { id: 'app-init', name: '应用装配', desc: 'APP 层入口：创建并初始化所有 RTOS 任务、app_state 与 USB 传输。', tags: ['rtos', 'init', 'entry-point'], files: ['APP/Src/rtos/app_task.c', 'APP/Inc/rtos/app_task.h'] },
  { id: 'imu-sampling', name: 'IMU 采样', desc: 'ImuTask 是 SPI1/ICM42688P 唯一所有者，1kHz DRDY 门控 DMA 采样原始陀螺/加速度数据。', tags: ['imu', 'sensor', 'dma', 'task'], files: ['APP/Src/rtos/imu_task.c', 'APP/Inc/rtos/imu_task.h', 'APP/Src/bsp/imu_bus.c', 'APP/Inc/bsp/imu_bus.h', 'APP/Src/drivers/icm42688p.c', 'APP/Inc/drivers/icm42688p.h'] },
  { id: 'time-base', name: '时间基准', desc: 'Cortex-M7 DWT 微秒时基，提供真实 dt 供采样门控、滤波与姿态积分。', tags: ['platform', 'time', 'dwt'], files: ['APP/Src/platform/platform_time.c', 'APP/Inc/platform/platform_time.h'] },
  { id: 'calibration', name: '校准', desc: 'SI 换算与 CW90 之后执行陀螺/加速度零偏校准状态机，候选值持久化成功前不切换。', tags: ['algorithm', 'calibration'], files: ['APP/Src/algorithms/gyro_calibration.c', 'APP/Inc/algorithms/gyro_calibration.h', 'APP/Src/algorithms/accel_calibration.c', 'APP/Inc/algorithms/accel_calibration.h'] },
  { id: 'filter', name: '低通滤波', desc: '三轴 PT1 低通，按真实 dt 更新，Gyro 100Hz / Accel 30Hz，专供后续姿态链。', tags: ['algorithm', 'filter', 'pt1'], files: ['APP/Src/algorithms/imu_filter.c', 'APP/Inc/algorithms/imu_filter.h'] },
  { id: 'attitude', name: '姿态估计', desc: 'Mahony 四元数融合陀螺与加速度，输出机体姿态；dt 无效或 IMU 门禁未过时禁止积分。', tags: ['algorithm', 'attitude', 'mahony'], files: ['APP/Src/algorithms/attitude_estimator.c', 'APP/Inc/algorithms/attitude_estimator.h'] },
  { id: 'rc-input', name: 'RC 输入', desc: '物理 CRSF 解析 + Linux 虚拟候选监视 + 人工授权仲裁，生成控制链使用的设定值。', tags: ['algorithm', 'rc', 'arbiter'], files: ['APP/Src/rtos/rc_task.c', 'APP/Inc/rtos/rc_task.h', 'APP/Src/bsp/crsf_uart.c', 'APP/Inc/bsp/crsf_uart.h', 'APP/Src/protocol/crsf.c', 'APP/Inc/protocol/crsf.h', 'APP/Src/bsp/linux_rc_monitor.c', 'APP/Inc/bsp/linux_rc_monitor.h', 'APP/Src/algorithms/rc_source_arbiter.c', 'APP/Inc/algorithms/rc_source_arbiter.h', 'APP/Src/algorithms/rc_input.c', 'APP/Inc/algorithms/rc_input.h', 'APP/Src/algorithms/rc_setpoint.c', 'APP/Inc/algorithms/rc_setpoint.h'] },
  { id: 'flight-control', name: '飞行控制', desc: 'FlightTask 1kHz 消费 RC/PID/Angle：ANGLE 外环转 Rate 设定，Rate PID 计算控制量，四态解锁链门禁。', tags: ['flight-control', 'pid', 'arming'], files: ['APP/Src/rtos/flight_task.c', 'APP/Inc/rtos/flight_task.h', 'APP/Src/algorithms/angle_outer_loop.c', 'APP/Inc/algorithms/angle_outer_loop.h', 'APP/Src/algorithms/rate_pid.c', 'APP/Inc/algorithms/rate_pid.h', 'APP/Src/algorithms/flight_arming.c', 'APP/Inc/algorithms/flight_arming.h'] },
  { id: 'mixer', name: '混控', desc: 'Quad-X 混控矩阵，把三轴控制量 + 油门映射到 4 路电机输出端点。', tags: ['algorithm', 'mixer', 'quad-x'], files: ['APP/Src/algorithms/quad_x_mixer.c', 'APP/Inc/algorithms/quad_x_mixer.h'] },
  { id: 'motor-output', name: '电机输出', desc: 'DShot 电调协议输出，把电机端点值编码为 DShot 帧并经定时器 DMA 发送给电调。', tags: ['bsp', 'motor', 'dshot'], files: ['APP/Src/bsp/dshot_motor.c', 'APP/Inc/bsp/dshot_motor.h'] },
  { id: 'msp-usb', name: 'MSP/USB 通信', desc: 'USB CDC 环形缓冲收发 + MSP 逐字节状态机解析与命令派发，使 Configurator 可读状态/写配置。', tags: ['protocol', 'msp', 'usb', 'comms'], files: ['APP/Src/bsp/usb_cdc_transport.c', 'APP/Inc/bsp/usb_cdc_transport.h', 'APP/Src/protocol/msp_transport.c', 'APP/Inc/protocol/msp_transport.h', 'APP/Src/protocol/msp_server.c', 'APP/Inc/protocol/msp_server.h'] },
  { id: 'param-store', name: '参数存储', desc: 'Sector 6/7 双槽 Flash 参数存储与迁移，保存偏置/RC/PID/Angle/motor_idle/名称。', tags: ['storage', 'flash', 'parameter'], files: ['APP/Src/storage/parameter_store.c', 'APP/Inc/storage/parameter_store.h'] },
  { id: 'app-state', name: '全局状态', desc: 'app_state 共享快照：跨任务短临界区发布 IMU/校准/滤波/姿态/电池信息，供飞行与 MSP 读取。', tags: ['state', 'snapshot', 'shared'], files: ['APP/Src/app_state.c', 'APP/Inc/app_state.h'] },
  { id: 'diag', name: '平台诊断', desc: '平台基线诊断与安全停机：UART4 输出摘要，致命故障/保存前强制电机低电平。', tags: ['platform', 'diag', 'safety'], files: ['APP/Src/platform/platform_diag.c', 'APP/Inc/platform/platform_diag.h'] },
  { id: 'power', name: '电源监测', desc: 'ADC3 采样 + 电源监测算法 + 电池任务，发布电压/电流/剩余量到 app_state。', tags: ['power', 'adc', 'battery'], files: ['APP/Src/bsp/power_adc.c', 'APP/Inc/bsp/power_adc.h', 'APP/Src/algorithms/power_monitor.c', 'APP/Inc/algorithms/power_monitor.h', 'APP/Src/rtos/battery_task.c', 'APP/Inc/rtos/battery_task.h'] },
];

const moduleById = {};
MODULES.forEach(m => { moduleById[m.id] = m; });

// ---- nodes ----
const nodes = [];
const seen = new Set();
function add(n) { if (!seen.has(n.id)) { seen.add(n.id); nodes.push(n); } }

// module nodes
for (const m of MODULES) {
  add({ id: `module:${m.id}`, type: 'module', name: m.name, filePath: m.files[0], summary: m.desc, tags: m.tags, complexity: 'moderate' });
}
// file nodes
for (const f of scan.files) {
  const p = f.path;
  const d = DESC[p] || null;
  const name = p.split('/').pop();
  const summary = d ? d[1] : `${name}：C 源文件。`;
  const tags = d ? d[2] : ['module', 'code'];
  const lines = fileSizes[p] || 0;
  const complexity = lines < 50 ? 'simple' : (lines <= 200 ? 'moderate' : 'complex');
  add({ id: `file:${p}`, type: 'file', name, filePath: p, summary, tags, complexity });
}

// ---- edges ----
const edges = [];
// contains: module -> its files
for (const m of MODULES) {
  for (const fp of m.files) {
    edges.push({ source: `module:${m.id}`, target: `file:${fp}`, type: 'contains', direction: 'forward', weight: 1.0 });
  }
}
// data/control flow between modules (system logic)
const FLOW = [
  ['app-init', 'imu-sampling', 'calls', '创建并启动 ImuTask'],
  ['app-init', 'msp-usb', 'calls', '创建并启动 MspTask'],
  ['app-init', 'rc-input', 'calls', '创建并启动 RcTask'],
  ['app-init', 'flight-control', 'calls', '创建并启动 FlightTask'],
  ['app-init', 'power', 'calls', '创建并启动 BatteryTask'],
  ['app-init', 'app-state', 'calls', '初始化全局状态'],
  ['imu-sampling', 'calibration', 'depends_on', '原始样本送入校准'],
  ['calibration', 'filter', 'depends_on', '校准后数据送入低通'],
  ['filter', 'attitude', 'depends_on', '滤波后数据送入姿态估计'],
  ['time-base', 'imu-sampling', 'depends_on', '提供采样真实 dt'],
  ['time-base', 'filter', 'depends_on', '提供滤波真实 dt'],
  ['time-base', 'attitude', 'depends_on', '提供姿态积分 dt'],
  ['attitude', 'flight-control', 'depends_on', '姿态送入 ANGLE 外环'],
  ['rc-input', 'flight-control', 'depends_on', 'RC 设定值送入飞行控制'],
  ['flight-control', 'mixer', 'depends_on', '控制量送入混控'],
  ['mixer', 'motor-output', 'depends_on', '混控输出送入电机'],
  ['flight-control', 'motor-output', 'depends_on', '解锁门禁放行电机'],
  ['imu-sampling', 'app-state', 'writes_to', '发布 IMU 快照'],
  ['calibration', 'app-state', 'writes_to', '发布校准状态'],
  ['filter', 'app-state', 'writes_to', '发布滤波数据'],
  ['app-state', 'flight-control', 'reads_from', '飞行控制读取状态'],
  ['app-state', 'msp-usb', 'reads_from', 'MSP 读取遥测'],
  ['msp-usb', 'app-state', 'writes_to', 'MSP 写配置'],
  ['calibration', 'param-store', 'writes_to', '保存零偏到 Flash'],
  ['param-store', 'calibration', 'reads_from', '启动时加载零偏'],
  ['power', 'app-state', 'writes_to', '发布电池信息'],
  ['diag', 'motor-output', 'depends_on', '安全停机强制电机低电平'],
];
const W = { calls: 0.8, depends_on: 0.6, writes_to: 0.5, reads_from: 0.5 };
for (const [s, t, type, label] of FLOW) {
  edges.push({ source: `module:${s}`, target: `module:${t}`, type, direction: 'forward', weight: W[type] || 0.6, label });
}

// ---- layers: one per module (module node + its files) ----
const layers = MODULES.map(m => ({
  id: `layer:${m.id}`,
  name: m.name,
  description: m.desc,
  nodeIds: [`module:${m.id}`, ...m.files.map(f => `file:${f}`)],
}));

// ---- tour: walk the pipeline ----
const TOUR_ORDER = [
  ['app-init', '应用装配：任务创建与初始化'],
  ['imu-sampling', 'IMU 采样：1kHz DRDY 门控 DMA 采集原始数据'],
  ['time-base', '时间基准：DWT 微秒时基提供真实 dt'],
  ['calibration', '校准：陀螺/加速度零偏状态机'],
  ['filter', '低通滤波：三轴 PT1 低通'],
  ['attitude', '姿态估计：Mahony 四元数融合'],
  ['rc-input', 'RC 输入：物理/虚拟源仲裁与设定值'],
  ['flight-control', '飞行控制：ANGLE 外环 + Rate PID + 四态解锁'],
  ['mixer', '混控：Quad-X 矩阵映射'],
  ['motor-output', '电机输出：DShot 帧发送'],
  ['app-state', '全局状态：跨任务共享快照'],
  ['msp-usb', 'MSP/USB 通信：Configurator 遥测与配置'],
  ['param-store', '参数存储：双槽 Flash 持久化'],
  ['power', '电源监测：电池电压/电流采集'],
  ['diag', '平台诊断：安全停机与摘要'],
];
const tour = TOUR_ORDER.map(([id, title], i) => ({
  order: i + 1,
  title,
  description: moduleById[id].desc,
  nodeIds: [`module:${id}`],
}));

const final = {
  version: '1.0.0',
  project: {
    name: 'GETFUN_F722_FreeRTOS',
    languages: ['c'],
    frameworks: ['stm32-hal', 'freertos', 'cmsis-rtos'],
    description: '基于 STM32F722 + FreeRTOS 的飞控固件 APP 应用层，按系统逻辑（数据流/控制流）组织的子系统图谱。',
    analyzedAt: new Date().toISOString(),
    gitCommitHash: execSync('git rev-parse HEAD').toString().trim(),
  },
  nodes, edges, layers, tour,
};

fs.writeFileSync('.understand-anything/intermediate/assembled-graph.json', JSON.stringify(final, null, 2));
const stats = {
  totalNodes: nodes.length, totalEdges: edges.length, totalLayers: layers.length, tourSteps: tour.length,
  nodeTypes: nodes.reduce((a, n) => { a[n.type] = (a[n.type] || 0) + 1; return a; }, {}),
  edgeTypes: edges.reduce((a, e) => { a[e.type] = (a[e.type] || 0) + 1; return a; }, {}),
};
console.log('nodes:', nodes.length, 'edges:', edges.length, 'layers:', layers.length, 'tour:', tour.length);
console.log('nodeTypes:', JSON.stringify(stats.nodeTypes));
console.log('edgeTypes:', JSON.stringify(stats.edgeTypes));
