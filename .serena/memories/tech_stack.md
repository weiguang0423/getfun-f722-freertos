# Tech stack

- STM32: C11 + ARM assembly; STM32F722RETx Cortex-M7 216 MHz; `arm-none-eabi-gcc`; CMake >=3.22 + Ninja; CubeMX `.ioc`; Debug O0/g3, Release Os/g0.
- Firmware infrastructure: CMSIS Core 5.1, STM32F7 CMSIS Device 1.2.10, STM32F7 HAL 1.3.3, FreeRTOS Kernel 10.2.1 with heap_4, CMSIS-RTOS V2, ST USB Device CDC.
- Linux target: RK3568 ARM64 Buildroot/glibc 2.35 user space; C++17; POSIX/Linux V4L2, DRM/KMS uapi, termios/poll/signals.
- ML/image: fixed RKNN-Toolkit2/Runtime 1.3.0 lineage and RKNPU driver 0.8.2 baseline; FP16 models selected; board-side minimal OpenCV 4.5.5.
- PC reference/conversion: Python 3.12 + `ai-edge-litert==2.1.6` + `opencv-python-headless==4.13.0.92` for S7.2; SDK RKNN Toolkit2 environment uses Python 3.8 for conversion.
- Verification helpers: Node.js `.mjs`, host C/C++ compiler, Python scripts; no repository-wide package manifest, linter, or unit-test runner.