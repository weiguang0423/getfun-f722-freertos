<!--
文件用途：面向不熟悉 Linux 命令行操作者的 S7.4 屏显核验保姆级执行指南。
覆盖范围：WSL 打开与 SDK 检查（precheck_wsl.sh）、板端登录（SSH/串口两种方式）、
板端屏显检查（precheck_board.sh）、构建与屏显冒烟的完整命令。
关键约束：每一步都标明"在哪里操作"、"输入什么"、"预期看到什么"；输出请原样贴回。
-->

# S7.4 屏显核验执行指南（手把手）

> 你不需要理解原理，只需要**照着输入命令、把输出复制回来**。
> 所有"输入什么"里的命令都可以整行复制粘贴，注意不要漏掉空格。

## 0. 先准备三样东西

| 东西 | 状态检查 |
|---|---|
| 开发板通电、开机完成 | 板子电源灯亮，开机后等 30 秒以上 |
| 屏幕接在开发板上 | 排线插好（MIPI/HDMI 等，按板子说明书） |
| 摄像头接在开发板上 | 之前 S7.4 用过的 IMX335 |
| 网线直连电脑 | 电脑任务栏网络图标没有红叉 |

## 1. 认识三个"操作地点"

| 地点 | 是什么 | 怎么打开 |
|---|---|---|
| **WSL** | 你电脑里的 Linux 系统（之前的模型转换、交叉编译都在这里做的） | 按 Win 键 → 输入 `Ubuntu` 或 `WSL` → 回车（出现黑窗口，光标前有 `$`） |
| **板端** | 开发板上的 Linux 系统 | 用 SSH 登录（见第 3 步）或串口登录 |
| **本聊天** | 把命令输出贴回来给我看 | — |

**从现在开始，所有操作都在 WSL 黑窗口里进行**（除非我特别说明）。

## 2. 第一步：在 WSL 里检查 SDK（约 1 分钟）

在 WSL 黑窗口里，**整行复制粘贴**下面这条命令，然后按回车：

    sh /mnt/e/getfun-f722-freertos/GETFUN_F722_FreeRTOS/Linux/RK3568/s7_4/precheck_wsl.sh

**预期看到**：屏幕刷出一串以 `== 1. SDK 根目录 ==`、`== 2. 内核 uapi DRM 头 ==` 等开头的内容，每行结尾是 `OK   ...` 或 `MISSING ...` 或 `absent ... (可选)`。

**可能遇到的问题**：
- 提示 `No such file or directory`（找不到文件）→ 说明你的仓库不在 E 盘这个位置。请告诉我你电脑上仓库的完整路径（例如 `D:\xxx\GETFUN_F722_FreeRTOS`），我帮你改命令。
- 提示 `wsl: command not found` → 说明没打开 WSL。请告诉我你之前做模型转换（S7.2/S7.3）用的什么软件，或者截图给我看。

**做完**：把窗口里**全部输出**复制，贴到本聊天里。

## 3. 第二步：登录开发板（两种方式任选其一）

### 方式 A：SSH 登录（推荐，网络已确认通）

在 WSL 黑窗口里输入：

    ssh root@169.254.151.86

- 第一次会问 `Are you sure you want to continue connecting (yes/no)?` → 输入 `yes` 回车
- 然后问 `root@169.254.151.86's password:`（密码，输入时屏幕不显示是正常的）：
  - 如果你知道密码，直接输入
  - 如果不知道，先试直接按回车；不行再试密码 `root`；还不行就跳到**方式 B**
- 登录成功后，命令行提示符会变成类似 `root@ATK-DLRK3568:~#` 或 `~ #`

### 方式 B：串口登录（SSH 不通时用）

1. 下载并打开 PuTTY（https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html ，选 Windows 安装版）
2. 左侧选 `Connection → Serial`，Serial line 填 `COM5`，Speed 先填 `115200`（如果屏幕无内容，关闭重开再试 `1500000`）
3. 左侧点回 `Session`，Connection type 选 `Serial`，点 `Open`
4. 回车几次，出现 `login:` 输入 `root` 回车，密码同样先试空或 `root`

## 4. 第三步：在板端跑屏显检查（约 1 分钟）

### 如果用的是方式 A（SSH）：

先按 `Ctrl+D` 退出板端登录回到 WSL，然后**逐条**复制执行（每行回车一次）：

    scp /mnt/e/getfun-f722-freertos/GETFUN_F722_FreeRTOS/Linux/RK3568/s7_4/precheck_board.sh root@169.254.151.86:/tmp/

（同样会要密码，和上面 ssh 一样。传完后再登录一次：）

    ssh root@169.254.151.86 "sh /tmp/precheck_board.sh"

### 如果用的是方式 B（串口）：

不用传文件，直接在串口窗口里**逐条**复制粘贴以下命令（每条回车）：

    ls -l /dev/dri/

    for d in /sys/class/drm/card*/; do echo "--- $d"; cat "$d/status"; cat "$d/modes"; done

    ls -l /dev/fb*

    cat /proc/cmdline

    dmesg | grep -iE "drm|dsi|panel|hdmi|vop" | tail -30

    ps aux | grep -iE "weston|Xorg|wayland|kms|qt" | grep -v grep

**做完**：把板端输出**全部复制**，贴回本聊天里。

## 5. 贴回输出后，我会做什么

1. 根据 `/dev/dri/card0` 是否存在、屏的分辨率、SDK 里的头文件是否齐全，判定"可以开始 / 还缺什么"
2. 通过后给你**构建命令**（还是在 WSL 里跑），生成两个程序
3. 再给你**屏显冒烟命令**（在板端跑），屏上会出现渐变色块，代表显示链路通了
4. 最后把实时显示合入推理程序，你就可以站在板子旁边直接看手部识别了

## 6. 常见问题速查

| 现象 | 处理 |
|---|---|
| `ssh: connect to host ... refused` | 板子没开机或网线没插好，检查第 0 步 |
| `Permission denied`（SSH 密码不对） | 换方式 B 串口 |
| 串口打开后一片空白 | PuTTY 里换 Speed：115200 ↔ 1500000 轮流试 |
| `sh: 0: Can't open ...` | 路径不对，把仓库完整路径告诉我 |
| precheck 输出全是 MISSING | 把输出贴回来，我会判断是路径问题还是环境问题 |
