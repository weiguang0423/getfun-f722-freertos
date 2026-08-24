const fs = require('fs');
const g = JSON.parse(fs.readFileSync('.understand-anything/intermediate/assembled-graph.json', 'utf8'));
const ids = new Set(g.nodes.map(n => n.id));
function f(p) { return `file:${p}`; }
function keep(arr) { return arr.filter(x => ids.has(x)); }

const steps = [
  { order: 1, title: '应用入口与任务装配', description: 'app_tasks_init() 由 CubeMX 生成的 freertos.c 调用，初始化 app_state 与 USB 传输，并静态创建 ImuTask 与 MspTask，是 APP 层 RTOS 任务的装配点。', nodeIds: keep([f('APP/Src/rtos/app_task.c')]) },
  { order: 2, title: 'IMU 采样链路', description: 'ImuTask 是 SPI1/ICM42688P 唯一所有者：imu_bus 适配 SPI+DMA，icm42688p 驱动寄存器，1kHz DRDY 门控 DMA 采样原始数据。', nodeIds: keep([f('APP/Src/rtos/imu_task.c'), f('APP/Src/bsp/imu_bus.c'), f('APP/Src/drivers/icm42688p.c')]) },
  { order: 3, title: '时间基准与精确采样', description: 'platform_time 用 Cortex-M7 DWT 微秒时基，ISR 只读 CYCCNT，ImuTask 单写者做 32 位回绕扩展，提供真实 dt 供后续滤波与姿态积分。', nodeIds: keep([f('APP/Src/platform/platform_time.c')]) },
  { order: 4, title: '校准：陀螺与加速度零偏', description: 'SI 换算与 CW90 之后执行校准：gyro_calibration 做上电静态零偏状态机，accel_calibration 做水平单面加速度校准，候选值持久化成功前不切换。', nodeIds: keep([f('APP/Src/algorithms/gyro_calibration.c'), f('APP/Src/algorithms/accel_calibration.c')]) },
  { order: 5, title: '低通滤波', description: 'imu_filter 对校准后数据做三轴 PT1 低通，Gyro 100Hz / Accel 30Hz，按真实 dt 更新，首样本播种、异常重置。', nodeIds: keep([f('APP/Src/algorithms/imu_filter.c')]) },
  { order: 6, title: '姿态估计', description: 'attitude_estimator 用 Mahony 四元数融合陀螺与加速度，输出姿态；dt 无效或 IMU 门禁未过时禁止积分。', nodeIds: keep([f('APP/Src/algorithms/attitude_estimator.c')]) },
  { order: 7, title: 'RC 输入与源仲裁', description: 'RcTask 解析物理 CRSF，linux_rc_monitor 经 USART6 收虚拟候选，rc_source_arbiter 做人工授权仲裁，rc_input/rc_setpoint 把通道映射为控制设定。', nodeIds: keep([f('APP/Src/rtos/rc_task.c'), f('APP/Src/bsp/linux_rc_monitor.c'), f('APP/Src/algorithms/rc_source_arbiter.c'), f('APP/Src/algorithms/rc_input.c'), f('APP/Src/algorithms/rc_setpoint.c')]) },
  { order: 8, title: '飞行控制与混控', description: 'FlightTask 1kHz 消费 RC/PID/Angle：angle_outer_loop 把姿态角误差转 Rate 设定，rate_pid 计算控制量，quad_x_mixer 映射到电机，flight_arming 四态安全链仅在 ARMED 放行。', nodeIds: keep([f('APP/Src/rtos/flight_task.c'), f('APP/Src/algorithms/angle_outer_loop.c'), f('APP/Src/algorithms/rate_pid.c'), f('APP/Src/algorithms/quad_x_mixer.c'), f('APP/Src/algorithms/flight_arming.c')]) },
  { order: 9, title: '电机输出', description: 'dshot_motor 把混控后的电机端点值编码为 DShot 帧，经定时器 DMA 发送给电调。', nodeIds: keep([f('APP/Src/bsp/dshot_motor.c')]) },
  { order: 10, title: '参数持久化', description: 'parameter_store 把偏置/RC/PID/Angle/motor_idle 等存入 Sector 6/7 双槽 Flash，只擦非活动槽，兼容 v1 迁移。', nodeIds: keep([f('APP/Src/storage/parameter_store.c')]) },
  { order: 11, title: 'MSP 协议与 USB 传输', description: 'usb_cdc_transport 在 USB CDC 上做环形缓冲收发，msp_transport 逐字节状态机解析，msp_server 按命令派发并回包，使 Configurator 可读状态/写配置。', nodeIds: keep([f('APP/Src/bsp/usb_cdc_transport.c'), f('APP/Src/protocol/msp_transport.c'), f('APP/Src/protocol/msp_server.c')]) },
  { order: 12, title: '全局状态与诊断', description: 'app_state 以关中断短临界区发布跨任务共享快照；platform_diag 经 UART4 输出摘要，并在致命故障/保存前强制电机低电平。', nodeIds: keep([f('APP/Src/app_state.c'), f('APP/Src/platform/platform_diag.c')]) },
];

const clean = steps.filter(s => s.nodeIds.length > 0);
fs.writeFileSync('.understand-anything/intermediate/tour.json', JSON.stringify(clean, null, 2));
console.log('tour steps:', clean.length);
clean.forEach(s => console.log(`  ${s.order}. ${s.title} (${s.nodeIds.length} nodes)`));
