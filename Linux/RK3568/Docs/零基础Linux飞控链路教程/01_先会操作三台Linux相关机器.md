---
title: 从零开始操作WSL与RK3568
---

# 先会操作三台 Linux 相关机器

> **当前学习位置：** `整条链路地图` → **分清 Windows、WSL、RK3568 并安全登录** → `交叉编译与部署`
>
> 本篇只建立操作环境和验证 S7.1 基线。完成后，你能说清一条命令运行在哪里，能进入 WSL 和板端 Linux；不会修改系统、安装最新版库或接入飞控。

## 为什么不能把“Linux”当成一台机器

本项目至少有两个 Linux：电脑里的 WSL 和 RK3568 板端。它们的 CPU、系统库和任务不同。

```text
Windows
 ├─ WSL Ubuntu：准备 SDK、执行交叉编译
 └─ RK3568 Buildroot：运行摄像头/NPU/串口程序
```

WSL 常把 Windows 的 `E:` 盘挂载成 `/mnt/e`。因此，Windows 中的仓库：

```text
E:\getfun-f722-freertos\GETFUN_F722_FreeRTOS
```

在 WSL 中通常写作：

```text
/mnt/e/getfun-f722-freertos/GETFUN_F722_FreeRTOS
```

反斜杠 `\` 是 Windows 路径分隔符；正斜杠 `/` 是 Linux 路径分隔符。不要把两种写法混用。

> **概念说明：终端、Shell 与命令**
>
> 终端是你输入文字的窗口。Shell 是读懂这些文字并启动程序的解释器。PowerShell、WSL 的 `sh`/`bash` 都是 Shell，只是语法不同。后文中 `pwd`、`ls` 等是 Linux 命令；请只在标有“WSL”或“RK3568 板端”的地方运行。

## 先在 Windows 确认仓库，再进入 WSL

**运行地点：Windows PowerShell。** 打开 PowerShell 后输入：

```powershell
Set-Location E:\getfun-f722-freertos\GETFUN_F722_FreeRTOS
Get-Location
wsl --status
```

`Set-Location` 的作用是进入仓库目录；`Get-Location` 显示当前位置；`wsl --status` 只读取 WSL 状态，不会改动任何内容。

接着启动你的默认 WSL 发行版：

```powershell
wsl
```

看到类似 `user@电脑名:~$` 的提示符后，说明你已经在 WSL。先用下面三条命令建立方向感：

```sh
pwd
ls /mnt/e/getfun-f722-freertos/GETFUN_F722_FreeRTOS/Linux/RK3568
uname -m
```

| 命令 | 它回答的问题 | 预期意义 |
|---|---|---|
| `pwd` | 我现在在哪个目录？ | 通常是你的 WSL 家目录，例如 `/home/user123` |
| `ls .../RK3568` | 项目 Linux 文件是否能从 WSL 看到？ | 应看到 `s7_2`、`s7_3`、`s7_4`、`s7_5`、`s7_6` 等 |
| `uname -m` | 这台机器的 CPU 架构是什么？ | WSL PC 通常显示 `x86_64`，不是 RK3568 的 `aarch64` |

这里的 `ls` 相当于“列出目录内容”，但它不会递归删除、移动或修改文件。第一次学习 Linux 时，优先使用这类只读命令。

## 认识 Linux 的四个最常用安全命令

后续教程会用到的 Linux 基础很少。先只掌握下面四个：

| 命令 | 日常语言含义 | 常见例子 |
|---|---|---|
| `pwd` | 显示“我站在哪” | `pwd` |
| `ls` | 看一个目录里有什么 | `ls Linux/RK3568/s7_4` |
| `cd` | 进入另一个目录 | `cd /mnt/e/.../GETFUN_F722_FreeRTOS` |
| `cat` | 原样显示一个小文本文件 | `cat /proc/version` |

命令末尾没有 `>` 时，一般只是把结果显示到屏幕。以后看到 `> results.jsonl`，它表示把程序输出写入文件；这会覆盖同名文件，所以教程会在使用前明确说清保存位置。

## 怎样登录 RK3568 板端

RK3568 需要先通电并启动 Linux。项目已记录的两条进入方式如下。

### 网络通时，用 SSH

**运行地点：WSL。** 当前项目曾实测直连地址为 `169.254.151.86`；网络地址可能随你的网线和网卡状态改变，所以先确认板端实际地址，而不是把这一个旧值当成永久配置。

```sh
ssh root@169.254.151.86
```

SSH 的含义是“通过网络打开另一台机器的命令行”。首次连接会要求确认主机指纹；密码输入时屏幕不回显字符是正常现象。成功后，提示符会变成类似：

```text
root@ATK-DLRK3568:~#
```

这时你输入的命令已经在**开发板**执行，不再是在 WSL 执行。

### 网络不通时，用串口

项目 S7.1 记录的 Windows 串口为 `COM8`、`1500000 8N1`。`8N1` 的意思是每个字节有 8 个数据位（8）、无校验位（N）、1 个停止位（1）。

用 PuTTY 或其他串口终端连接时，先选：

```text
Serial line: COM8
Speed:       1500000
Data:        8
Parity:      None
Stop bits:   1
```

重启后如果你看到 `debug>`，它不是正常 Linux Shell；输入下面这个单词后回车：

```text
console
```

再看到 Linux 登录提示或 `#` 才继续执行 Linux 命令。

## 在板端只读检查 S7.1 环境

> **当前学习位置：** `已经进入 RK3568` → **确认摄像头、系统与 NPU 的现有基线** → `不改动环境地进入部署阶段`

以下命令都在 **RK3568 板端 Shell** 运行：

```sh
uname -a
cat /proc/version
ls -l /dev/video0
v4l2-ctl --list-devices
cat /sys/module/rknpu/version
sha256sum /usr/lib/librknnrt.so
```

它们分别查看内核、摄像头节点、视频设备、RKNPU 驱动版本和板端 RKNN Runtime 文件的 SHA-256 校验值。SHA-256 可以理解为文件的“指纹”：同一字节内容必然得到同一串结果；不同结果就表示文件不是同一份。

根据项目 S7.1 记录，曾实测到的基线是：

| 项目 | 已记录的实测值 | 你重新操作时应怎样理解 |
|---|---|---|
| 板端系统 | Buildroot，Linux `4.19.232`，`aarch64` | 不要求你的输出逐字符相同，但应确认确实是 RK3568 ARM64 板端 |
| 摄像头入口 | RKISP 主路径 `/dev/video0`，IMX335 | 节点不存在时先解决摄像头/驱动问题，不能开始推理 |
| RKNN Runtime | 板端自带 `librknnrt.so`，版本 1.3.0 | 不要用 SDK 中另一份库直接覆盖它 |
| RKNPU 驱动 | `0.8.2` | 它是板端驱动基线，不是“越新越好”的安装任务 |

这里特别重要的一点是：SDK 中的 `librknnrt.so` 与板端镜像自带 Runtime 的 SHA-256 曾不同。项目选择保留板端可实际运行的 Runtime，而不是为了“版本统一”直接替换驱动库。对初学者而言，这是一条很好的 Linux 部署原则：**先记录能工作的版本组合，修改版本必须单独验证和留回退路径。**

## 最小摄像头检查为什么不等于手势识别

板端的下面命令只检查摄像头能连续给出 NV12 图像：

```sh
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=800,height=600,pixelformat=NV12 \
  --stream-mmap=3 --stream-count=300 --stream-to=/dev/null
```

先解释首次出现的符号：

- `v4l2-ctl` 是 Linux 的视频设备控制工具；V4L2 是 Linux 的摄像头接口标准。
- `-d /dev/video0` 指定使用哪一个视频设备。
- `NV12` 是一种图像内存格式，不是图片文件格式。
- `--stream-mmap=3` 让驱动和程序共享 3 块内存缓冲，避免每帧再复制一遍。
- `/dev/null` 是 Linux 的“黑洞文件”；把数据写进去会丢弃，因此不会在磁盘留下巨大原始视频。

项目记录中，这条短采集以 `30.00 fps`、退出码 `0` 完成。它只说明摄像头可以采集，不说明模型、关键点、手势或飞控链路已经成功。下一篇才会说明怎样产生与板端匹配的 ARM64 程序。

## 本篇完成判据与常见卡点

完成本篇时，你应该能做到：

- 在提示符变化后辨认自己是在 PowerShell、WSL 还是 RK3568 上操作。
- 在 WSL 中看到项目 `Linux/RK3568` 目录，并用 `uname -m` 看出它不是 ARM64 板端。
- 至少用 SSH 或串口其中一种方式进入板端，并只读确认 `/dev/video0` 与 RKNPU 基线。

| 现象 | 最先检查什么 | 不要做什么 |
|---|---|---|
| `No such file or directory` | 当前地点和路径格式是否匹配；在 WSL 用 `/mnt/e/...` | 不要在不确定位置执行删除或重装命令 |
| SSH 连不上 | 板子是否启动、网线和当前 IP；改用串口 | 不要把旧 IP 当作板端永久地址 |
| 串口全空白 | COM 号、1500000 波特率、GND 与串口线 | 不要随意短接未知引脚 |
| `/dev/video0` 不存在 | 摄像头排线、供电、内核设备枚举 | 不要先怀疑模型或重装 RKNN |

下一篇：[02_把已有模型和程序部署到RK3568](02_把已有模型和程序部署到RK3568.md)。
