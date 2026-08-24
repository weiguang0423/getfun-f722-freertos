# STM32 firmware core

- Entry: `Core/Src/main.c:main`: MPU -> I-cache -> HAL -> 216 MHz clock -> `linux_rc_monitor_init` -> GPIO/DMA/SPI1/USART2/UART4/USART6 -> motor-safe + diagnostics -> CMSIS-RTOS kernel init -> `MX_FREERTOS_Init` -> scheduler.
- `Core/Src/freertos.c:MX_FREERTOS_Init` creates CubeMX `InitTask` and calls `APP/Src/rtos/app_task.c:app_tasks_init` inside a USER CODE block.
- `app_tasks_init` initializes `app_state` and USB transport; statically creates ImuTask, RcTask, BatteryTask, FlightTask, MspTask.
- Main data owners: ImuTask exclusively owns SPI1/ICM42688P, DWT time extension, filter/attitude and parameter writes; RcTask exclusively publishes final RC; FlightTask consumes consistent snapshots at 1 kHz and exclusively controls armed motor output; MspTask owns byte-stream MSP handling.
- `app_state_snapshot_t` is shared runtime state; short PRIMASK-disable + DMB critical sections, never blocking inside publication.
- Sensor path: SPI1 DMA -> `imu_bus` -> `icm42688p` -> SI/CW90 -> gyro+accel calibration -> raw MSP branch and PT1 -> Mahony -> app state.
- Physical RC path: UART2 DMA CRSF -> parser -> `rc_input` -> RcTask. Linux path: USART6 byte ISR -> `linux_rc_monitor` fixed-frame validation -> candidate snapshot -> `rc_source_arbiter_update`; only R/P/Y can replace physical channels.
- Control path: RC snapshot -> setpoint/Angle outer loop -> Rate PID -> Quad-X mixer -> ARM/PREARM/Failsafe -> DShot600. Motor test and armed output are mutually exclusive; faults force low.
- Flash program region is first 256 KiB; sectors 6/7 are parameter A/B slots (`STM32F722XX_FLASH.ld`).
- APP sources must be listed in root `CMakeLists.txt`; CubeMX source lists live in `cmake/stm32cubemx/CMakeLists.txt`.