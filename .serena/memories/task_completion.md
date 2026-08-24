# Task completion gates

- Check `Docs/00_项目开发路线与统一进度.md` first for the active milestone, dependencies, exclusions and required real-hardware evidence.
- Firmware change: run relevant `Tools/verify_*.mjs`; configure/build both Debug and Release; inspect memory usage/map and warnings. Do not claim hardware behavior without board evidence.
- Linux pure logic: compile/run the stage-specific host self-test(s), then cross-build with the frozen toolchain. Run executable `--self-test` on RK3568 before live use.
- Linux camera/display/UART changes require appropriate board checks: actual V4L2 format/timestamps, RKNN runtime compatibility, DRM ownership/display behavior, or verified device node/electrical UART path. `/dev/null` is not UART acceptance.
- End-to-end virtual RC requires aligned Linux JSONL and STM32 UART/control diagnostics plus `verify_s7_8_trace.mjs`/`verify_s7_8_e2e.mjs`; propellers remain off until explicitly authorized milestone gates pass.
- For a milestone transition, update `Docs/00` and its plan/acceptance evidence in the same change; only developer-confirmed real-hardware tests may be marked accepted/frozen.
- New APP source must be registered in root `CMakeLists.txt`; generated-source changes belong to `.ioc`/CubeMX lists and USER CODE regions.
- Before handoff: `git diff`/`git status` to separate task changes from user/untracked state. Never include SDKs, models, binaries, caches or personal tooling.