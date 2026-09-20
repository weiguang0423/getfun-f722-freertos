# IMU 硬件数据传递交互页面

## 当前版本：硬件剖面示意

用户明确要求展示外设、CPU、DMA、内存之间的数据传递，主入口 `index.html` 已改为固定硬件位置的交互图。`hardware.js` 绘制 IMU 芯片、STM32 内部 CPU / SPI1 / DMA2 / SRAM / 条件启用的 D-Cache，按章节高亮实际传递方向。`hardware.html` 是相同页面的独立入口。

黄色表示 TX 命令与填充字节，蓝色表示 RX 返回数据，紫色虚线表示配置与中断，绿色表示 CPU 读写内存。字节符号、寄存器格子和时序均为教学示意，不是实测值、精确硬件周期或物理芯片布局。内部互连总线已简化。

当前启动源码只显式启用 I-Cache；D-Cache 以灰色条件能力展示，未将缓存维护误画成当前必经数据通路。

浏览器验证通过：1440×1000 桌面、390×844 窄屏无页面横向溢出；7 个章节按钮、同步解说、首尾导航、重置、播放、暂停、播放自动到达第 7 段并停止、CPU/内存局部放大均已操作检查。捕获的控制台错误和警告为空。已经查看桌面与窄屏截图；这不构成硬件验证。

本机预览：运行 `node Docs/imu-lifecycle-explorer/serve.cjs`，访问 `http://127.0.0.1:5178`。服务只监听本机回环地址，仅提供列出的教学文件，不提供目录浏览、源码目录或任意文件访问。也可直接在普通浏览器中打开 `index.html`。

`verify.cjs` 另外检查硬件章节路径引用、TX/RX 所有权、中断不携带采样负载与终点边界。原先的步骤版保存在 `steps.html`，原有独立总览图没有改动。

## 初版记录：文字步骤与独立图

初版现位于 `steps.html`。保留同目录的 `model.js` 和 `overview.html`，无需安装依赖或连接硬件。

- 点击 9 个节点，阅读处理者、输入、输出、数据形态、目的和源码位置。
- 支持播放、暂停、上一步、下一步、重置和 3 / 6 / 10 秒步进速度。
- 点击 DRDY 未就绪、DMA 超时、样本过期，查看对应处理策略。
- 独立总览入口打开 Archify 数据流图；正文交互页面为原生 HTML / CSS / JavaScript。

播放时间是教学时间，不是硬件耗时。页面没有接收传感器数据，没有执行固件，也不代表硬件或飞行验证。

## 源码范围

2026-09-16 核对工作区源码；HEAD 为 `92da0a54c84b4ae902d44fa4acd3cea7a4d1358b`。保留了工作区已有改动与既有图表。

- `APP/Src/rtos/imu_task.c`：初始化、DRDY 门控、DMA 等待、真实 dt、转换、校准、滤波和发布顺序。
- `APP/Src/bsp/imu_bus.c`：双槽、Cache 维护、CS、DMA 起止、ISR 通知链。
- `APP/Src/drivers/icm42688p.c`：14 B 负载、寄存器 DRDY、大端 int16 解码。
- `Core/Src/spi.c`：RX Stream0 / TX Stream3，DMA_NORMAL。
- `APP/Src/algorithms/imu_filter.c` 和对应头文件：PT1 系数、100 / 30 Hz、500–2000 μs 时间间隔。
- `APP/Src/app_state.c`：最新状态复制、IMU 和姿态分别发布。
- `APP/Src/rtos/flight_task.c`：sample_count 驱动、5 ms 年龄门限、控制输入。

## 验证边界

`node Docs/imu-lifecycle-explorer/verify.cjs` 检查导航边界、完整步进、异常阶段定位、解说字段完整性、脚本语法、本地资源与源码中的时间常数。这不等于浏览器交互测试。

初版交付时浏览器安全策略拒绝直接访问本地 HTML，初版未完成浏览器交互、控制台、响应式尺寸或视觉检查。当前硬件版已通过仅提供教学资产的本机预览完成上文列出的测试；该结果不追溯证明旧步骤版或独立图。

## 独立图交付记录

以下结果仅适用于 `overview.html`，不作为自定义交互页面的视觉证明。

```text
diagram_type: dataflow
output: C:/Users/13646/Desktop/Study/getfun-f722-freertos/Docs/imu-lifecycle-explorer/overview.html
specification_sha256: ff0afc11a15b45eb32398051f1369ad5cf213999ca4254a3056233c80029eb6f
artifact_sha256: 3a487b25c5b42a29a8afc9f351e7c6782691cb86bbb64b8d54b3af9b4b30a13e
validation: 9/9 showcase, 0 errors, 0 warnings
browser_evidence: not run (browser security policy blocked local HTML access)
visual_review: not performed
correction_rounds: 2
```

交付命令成功退出并原子生成该独立图。未运行 `visual-check`，因此不声称其 passed、failed 或工具定义的 skipped。
