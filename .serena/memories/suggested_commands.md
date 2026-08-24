# Suggested commands

## STM32 (PowerShell from repo root)
- Configure/build Debug: `cmake --preset Debug`; `cmake --build build/Debug`.
- Configure/build Release: `cmake --preset Release`; `cmake --build build/Release`.
- Run the applicable host verifier: `node Tools/verify_<area>.mjs` (most module verifiers are argument-free).
- S7.8 log checks: `node Tools/verify_s7_8_trace.mjs LINUX_TRACE.jsonl`; `node Tools/verify_s7_8_e2e.mjs LINUX_LOG.jsonl FLIGHT_LOG.log`.

## RK3568 flow (WSL/Linux shell unless stated)
- PC reference self-test: `python Linux/RK3568/s7_2/test_s7_2_reference.py`.
- Cross-build a stage: `sh Linux/RK3568/s7_3/build_board.sh` (or s7_4/s7_5/s7_6 equivalent).
- S7.5 host logic: compile/run command is in `Linux/RK3568/s7_5/README.md`.
- S7.6 host mapping and serial-file-loop tests: exact g++ commands are in `Linux/RK3568/s7_6/README.md`.
- Board binary sanity: `./s7_6_live --self-test`; UART path: `./s7_6_live --uart-test /dev/ttyS9 20 cycle` after device/pin verification.
- Inspect board resources: `ls -l /dev/video* /dev/dri/* /dev/ttyS*`; camera details via `v4l2-ctl -d /dev/video0 --all`.

## Windows discovery equivalents
- File search: `rg --files`; text search: `rg -n PATTERN PATH`; status: `git status --short --branch`.