# Project core

- One product, two runtimes: STM32F722/FreeRTOS flight-controller firmware plus RK3568/Linux companion-computer user-space software.
- `Docs/00_项目开发路线与统一进度.md` is the only live source for current milestone/status. `README.md` is an overview and may lag; hardware/electrical facts belong to `Docs/01_*`.
- Generated/infrastructure boundary: `Core/`, `USB_DEVICE/`, `Drivers/`, `Middlewares/`, `cmake/stm32cubemx/`; self-authored flight logic: `APP/`; Linux originals: `Linux/RK3568/`.
- Safety boundary: Linux produces only bounded candidate R/P/Y RC input. STM32 retains physical Throttle/all AUX, manual authorization/takeover, freshness checks, ARM/Failsafe, PID/Mixer, and DShot authority.
- End-to-end S7 path: IMX335 `/dev/video0` -> V4L2 MMAP NV12 -> RKNN detector+landmark -> gesture state machine -> bounded 44-byte virtual-RC frame -> RK3568 UART (`/dev/ttyS9` when verified) -> STM32 USART6 -> candidate validation -> RC source arbiter -> `RcTask` -> existing `FlightTask` safety/control chain.
- Betaflight UI path is separate: PC/App -> USB CDC -> MSP parser/server -> `app_state` snapshots/config transaction.
- Read STM32 map and invariants in `mem:stm32/core`; Linux stage/runtime map in `mem:linux/core`; Linux host/build/board boundary in `mem:linux/build_deploy`.
- Toolchain/dependency details: `mem:tech_stack`. Edit rules: `mem:conventions`. Commands and completion gates: `mem:suggested_commands`, `mem:task_completion`.