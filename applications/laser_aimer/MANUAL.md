# Laser Aimer — 使用手册

## 概述

Laser Aimer 是基于 **Simple-Aimer-Runtime** 框架开发的激光瞄准应用，使用 MindVision SUA133GC 工业相机采集图像，通过图像处理检测激光目标，并通过完整控制器解算云台控制指令。

控制器设计借鉴了 [LaserTracking-2026](https://github.com/NCST-Horizon-RM/LaserTracking-2026) 的优化经验。

---

## 目录

1. [环境准备](#1-环境准备)
2. [编译](#2-编译)
3. [配置文件](#3-配置文件)
4. [运行](#4-运行)
5. [IMGUI 调试面板](#5-imgui-调试面板)
6. [控制器参数调优](#6-控制器参数调优)
7. [调试路径对照](#7-调试路径对照)
8. [常见问题](#8-常见问题)

---

## 1. 环境准备

### 硬件
- MindVision SUA133GC USB3 工业相机
- x86_64 Linux 主机

### 软件依赖
```bash
# 系统库
sudo apt install libopencv-dev libeigen3-dev libglfw3-dev

# MVS SDK (已预装于 include/mvsdk/)
# 如需重新安装:
# sudo bash include/mvsdk/install.sh
# 然后重新插拔相机
```

### USB 设备检查
```bash
lsusb | grep MindVision
# 应看到: Bus 004 Device 002: ID f622:d132 MindVision SUA133GC
```

---

## 2. 编译

```bash
cd Simple-Aimer-Runtime/build
cmake ..
make -j$(nproc)
```

生成的二进制: `build/applications/laser_aimer/laser_aimer`

---

## 3. 配置文件

### control.yaml (控制器配置)

文件位置: `applications/laser_aimer/control.yaml`

```yaml
# P 控制器
kp: 1.2                     # P 增益，越大响应越快，过大会抖
deadband_px: 1.0             # 死区像素，目标在此范围内不输出
max_angle_rate: 180.0        # 最大输出角速度 (deg/s)
lowpass_alpha: 0.45          # 输出平滑系数 (<1=更平滑)
yaw_sign: -1.0               # yaw 方向符号 (±1)
pitch_sign: -1.0             # pitch 方向符号 (±1)

# 速度前馈
use_velocity_ff: 1           # 根据目标移动速度附加前馈量
ff_alpha: 0.25               # 前馈低通平滑
ff_rate_max: 150.0           # 前馈最大速率 (deg/s)

# 阻尼
use_damping: 1               # 像素速度阻尼防止过冲
damping_kd: 0.02             # 阻尼系数 (秒)

# 搜索策略 (目标丢失后自动搜索)
scan_enable: 1               # 启用搜索
scan_pattern: "spiral"       # circle=画圆 | spiral=螺线
scan_radius_deg: 12.0        # 画圆/螺线初始半径 (度)
scan_r_max_deg: 15.0         # 螺线最大半径 (度)
scan_enter_delay_ms: 180     # 进入搜索前等待 (ms)

# 启动策略
startup_prep_ms: 1000        # 启动准备阶段 (ms)，0=跳过
```

### 关键参数说明

| 参数 | 作用 | 调优方向 |
|------|------|---------|
| `kp` | P 增益 | 跟踪慢则加大，抖动则减小 |
| `deadband_px` | 死区 | 目标稳但微抖则加大 |
| `lowpass_alpha` | 平滑度 | 太抖则减小，太慢则加大 |
| `damping_kd` | 阻尼 | 过冲则加大，响应慢则减小 |
| `scan_radius_deg` | 搜索半径 | 运动快则加大 |
| `yaw_sign / pitch_sign` | 方向符号 | 目标反方向运动时取反 |

---

## 4. 运行

```bash
cd Simple-Aimer-Runtime/build
./applications/laser_aimer/laser_aimer
```

### 启动流程

1. SDK 初始化 → 枚举相机 → 打开 SUA133GC
2. 设置曝光时间 4000μs，模拟增益 128
3. 进入 IMGUI 调试界面
4. 自动开始图像采集 + 检测 + 控制循环

### 预期的日志输出

```
Starting Laser Aimer Application...
[LaserAimerProvider] Found 1 camera(s)
[LaserAimerProvider] Exposure time set to 4000
[LaserAimerProvider] Analog gain set to 128
[LaserAimerProvider] Camera started (PLAY mode)
DBG startup_prep begin
DBG startup_prep done
```

---

## 5. IMGUI 调试面板

打开程序后出现 IMGUI 窗口，包含以下模块:

### 5.1 参数调节面板
- **Detection Result** — 实时检测结果（画出的检测框和十字线）
- **Binary Mask** — 二值化掩膜视图
- **参数滑块** — 实时调节 HSV 阈值、形态学参数等

### 5.2 曲线监控
- `err_x` / `err_y` — 像素误差 (相对 boresight)
- `cmd_yaw` / `cmd_pitch` — 输出的云台指令

### 5.3 调试输出
- 控制台输出 `DBG_*` 前缀消息:
  - `DBG startup_prep` — 启动准备进度
  - `DBG lost_target` — 目标丢失通知
  - `DBG scan_begin` — 进入搜索模式
  - `DBG reacquired` — 重新捕获目标

---

## 6. 控制器参数调优

### 6.1 调优流程

1. **先确定方向符号** (`yaw_sign` / `pitch_sign`)
   - 让目标在视野中移动，云台应跟随目标方向
   - 如果反向，取反对应的 sign

2. **调 P 增益** (`kp`)
   - 从 0.5 开始，逐步增加
   - 以跟踪不抖动为目标值上限

3. **设置死区** (`deadband_px`)
   - 以目标静止时云台不微抖为准

4. **调速度前馈** (`use_velocity_ff` / `ff_alpha`)
   - 先关闭，P 调好后开启
   - 观察前馈速率是否过大

5. **开启阻尼** (`use_damping` / `damping_kd`)
   - 0.02 秒是常用值
   - 目标快速运动时适当减小

### 6.2 模式场景推荐

| 场景 | kp | deadband | ff | damping |
|------|-----|---------|-----|---------|
| 定点瞄准 | 0.5-0.8 | 3-5px | OFF | ON(0.04) |
| 慢速移动 | 1.0-1.2 | 1-2px | ON | ON(0.02) |
| 高速追踪 | 1.5-2.0 | 0.5-1px | ON | ON(0.01) |

---

## 7. 调试路径对照

与 LaserTracking-2026 的调试路径完全对齐:

| 调试手段 | LaserTracking-2026 | Laser Aimer (本应用) |
|---------|-------------------|---------------------|
| **控制台 DBG 日志** | `DBG startup_prep/lost/scan/reacq` | ✅ 完全对齐 |
| **曲线监控** | control_panel 曲线 | ✅ IMGUI DebugContext 曲线 |
| **参数热调节** | YAML 配置文件 | ✅ IMGUI 滑块 + YAML |
| **图像可视化** | OpenCV imshow | ✅ IMGUI Binary Mask + Detection Result |
| **YAML 配置** | `control.yaml` | ✅ `control.yaml` |

---

## 8. 常见问题

### Q: 程序启动后有 U3V BulkIn 错误？
A: 这些是 MindVision SDK 内部尝试 U3V 控制协议的信息，不影响功能。只要看到 `Exposure time set to 4000` 和 `Camera started (PLAY mode)` 即表示相机已正常工作。

### Q: Debug 图像颜色不对？
A: 已通过 `cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR)` 修正 RGB→BGR 转换。

### Q: 如何修改曝光/增益？
A: 两种方式:
1. 在 IMGUI 面板直接拖动 `Exposure Time` / `Analog Gain` 滑块
2. 修改 `control.yaml` 中对应参数

### Q: 如何添加自定义检测算法？
A: 参考 `Runtime.hpp` 的模板架构，实现自定义 `DataProvider<T>` 和 `System<T, S>`。详见框架代码中的注释。

---

## 文件结构

```
applications/laser_aimer/
├── CMakeLists.txt              # 构建配置 (链接 MVSDK + OpenCV + Eigen)
├── control.yaml                # 控制器参数配置
├── main.cpp                    # 入口: 组装 Runtime
├── LaserAimerProvider.hpp/cpp  # DataProvider: MindVision 相机采集
├── LaserAimerSolver.hpp/cpp    # Solver: 完整控制器
│                                #   - P 控制 + 死区(带磁滞)
│                                #   - 速度前馈 + 阻尼
│                                #   - 丢失目标扫描 (画圆/螺线)
│                                #   - 启动准备阶段
│                                #   - 输出平滑 + 速率限制
```

---

## 架构对照

```
Simple-Aimer-Runtime 管道      ←→   LaserTracking-2026 架构
─────────────────────────────       ──────────────────────────
DataProvider (相机采集)         ←→   hik_camera
System (多帧跟踪)               ←→   detector (detect + toMeasurement)
Selector (最优目标选择)          ←→   toMeasurement 筛选
Solver (控制器)                 ←→   control::Controller.update()
  ControlConfig                 ←→   ControlConfig
  CameraModel                   ←→   CameraModel
  Boresight                     ←→   Boresight
  GimbalState                   ←→   GimbalState
  GimbalCommand                 ←→   GimbalCommand (via gimbal_serial)
IMGUI 调试面板                  ←→   control_panel + STAT_CTRL 日志
control.yaml                    ←→   control.yaml
```
