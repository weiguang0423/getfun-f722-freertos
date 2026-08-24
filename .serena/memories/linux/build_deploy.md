# RK3568 build/deploy boundary

- Windows checkout is canonical source. WSL/Linux performs model conversion and ARM64 cross-build. RK3568 stores runnable deployment copies only; never leave unique source or acceptance evidence only inside WSL/board.
- Do not commit SDKs, virtualenvs, OpenCV build trees, `.rknn`, ARM64 executables, test corpora or one-off deployment bundles. Release deliverables should bind rebuilt artifacts to a Git tag with SHA-256.
- Environment knobs used by scripts: `SDK`, `OPENCV`, `DRMHDR`; S7.6 also `RK3568_CC`. Paths in evidence JSON are historical records, not interfaces.
- S7.3-S7.5 scripts use SDK/RKNN headers and minimal OpenCV core/imgproc/imgcodecs. Board supplies `librknnrt.so`; do not replace it casually. Private OpenCV `.405` libs sit beside executable under `lib/`, found through `$ORIGIN/lib`/`LD_LIBRARY_PATH`.
- S7.6 freezes `aarch64-linux-gnu-g++-10` by default and statically links libstdc++/libgcc because board Buildroot glibc is 2.35; the SDK host symlink may select gcc-13 and produce a GLIBC_2.36+ binary that cannot run.
- Device nodes are runtime resources, not source paths: camera typically `/dev/video0`, DRM `/dev/dri/card0`, candidate UART typically `/dev/ttyS9`; always verify actual board nodes/pins/3.3 V wiring before use.
- Before direct DRM output stop Weston; reboot normally restores it. `/dev/null` validates software flow but is not UART hardware acceptance.