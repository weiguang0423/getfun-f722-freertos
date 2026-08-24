# Project conventions

- New `.c/.h/.md` files require a leading purpose/scope/data-flow/constraints header; keep it synchronized.
- C application naming is snake_case; public structs/enums use `_t`; constants/macros are uppercase and domain-prefixed. Linux C++ stage APIs live in namespaces such as `s7_5`/`s7_6` and use RAII for resources.
- CubeMX `.ioc` is source of truth for pins/clocks/peripherals. In generated files edit only matching `USER CODE BEGIN/END` blocks; do not hand-maintain duplicate handles/IRQ/init logic. Checkpoint before regeneration and inspect diff afterward.
- Put self-authored algorithms, state machines, protocol, task and safety logic in `APP/`; generated layers provide HAL handles/interrupt ingress only.
- Isolate third-party internals behind project adapters. Pin versions/licenses; no build-time fetch of floating latest. Community components belong under `ThirdParty/<name>/`.
- Critical firmware paths (FlightTask, IMU sampling, DShot, safety state machines) forbid dynamic allocation, unbounded loops, uncontrolled blocking and hidden threads.
- ISR callbacks do minimal bounded work and never block; ownership/single-writer rules are architectural invariants.
- Fail closed: stale/invalid sensor, RC, Linux candidate, model/capture/serial failure must clear or invalidate output rather than reuse old actions.
- Linux machine-readable results go to stdout as JSONL; human diagnostics go to stderr; failures have explicit reason/exit status.
- A milestone is not complete from code alone: status/evidence and hardware gates follow `Docs/00_项目开发路线与统一进度.md`.