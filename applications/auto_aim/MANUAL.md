# AutoAim — 使用手册

> 基于 **Simple-Aimer-Runtime** 框架的自动瞄准应用，从 `spr_vision_try` 完整移植。

---

## 目录

1. [概述与 spr_vision_try 对应关系](#1-概述与-spr_vision_try-对应关系)
2. [架构设计](#2-架构设计)
3. [环境准备](#3-环境准备)
4. [编译](#4-编译)
5. [配置文件](#5-配置文件)
6. [运行](#6-运行)
7. [完整运行前置流程](#7-完整运行前置流程)
8. [后续开发空间](#8-后续开发空间)
9. [参数调优](#9-参数调优)

---

## 1. 概述与 spr_vision_try 对应关系

### 1.1 移植范围

| spr_vision_try 源文件 | → | auto_aim 目标文件 | 说明 |
|------------------------|---|-------------------|------|
| `tasks/auto_aim/armor.hpp/.cpp` | → | `types.hpp/.cpp` | 装甲板类型定义（原封不动保留） |
| `tools/extended_kalman_filter.hpp/.cpp` | → | `extended_kalman_filter.hpp/.cpp` | EKF 滤波器（原封不动保留） |
| `tasks/auto_aim/target.cpp` (EKF预测) | → | `AutoAimSystem.cpp::ekf_predict()` | EKF 状态传播（保留 F/Q 矩阵） |
| `tasks/auto_aim/target.cpp` (EKF更新) | → | `AutoAimSystem.cpp::ekf_update_ypda()` | EKF 观测更新（保留装甲板匹配） |
| `tasks/auto_aim/tracker.cpp` (状态机) | → | `AutoAimSystem.cpp::state_machine()` | 追踪器 FSM（保留全部迁移） |
| `tasks/auto_aim/tracker.cpp` (目标管理) | → | `AutoAimSystem.cpp::set_target()` / `update_target()` | 目标初始化与持续跟踪 |
| `tasks/auto_aim/aimer.cpp` (弹道) | → | `AutoAimSolver.cpp::aim_ballistic()` | 弹道解算 |
| `tasks/auto_aim/planner/planner.cpp` (MPC) | → | `AutoAimSolver.cpp::solve()` | MPC 预留（当前使用简化 P 控制） |
| `src/auto_aim_debug_mpc.cpp` (main) | → | `main.cpp` | 管线组装 |
| `io/gimbal/gimbal.hpp` (云台驱动) | → | `GimbalActuator.hpp/.cpp` | 包装为 Actuator 接口 |
| `tools/trajectory.hpp` (弹道计算) | → | `trajectory.hpp` | 最小依赖移植 |

### 1.2 管线映射

```
spr_vision_try:
  Camera → YOLO → Solver(PnP) → Tracker(FSM+EKF) → Aimer → Planner(MPC) → Gimbal

Simple-Aimer-Runtime:
  DataProvider ──→ System ──→ Selector ──→ Solver ──→ Actuator
      ↑               ↑           ↑           ↑           ↑
  相机+YOLO+PnP   EKF+状态机   选目标      弹道+MPC    云台驱动
```

### 1.3 与 laser_aimer 的区别

| 特性 | laser_aimer | auto_aim |
|------|-------------|----------|
| 检测方式 | HSV 阈值 + 形态学 | YOLO 神经网络 |
| 目标模型 | 2D 像素点 | 3D 世界坐标 (PnP) |
| 状态估计 | 无（直接透传） | 11/13 维 EKF |
| 追踪策略 | 无状态机 | 5 状态 FSM |
| 控制器 | P + 前馈 + 阻尼 | 弹道瞄准 + MPC(预留) + P(当前) |
| 云台协议 | 无（仅计算指令） | VisionToGimbal 串口帧 |

---

## 2. 架构设计

### 2.1 管道阶段

```
┌──────────────────────────────────────────────────────────────────┐
│                        Runtime (100Hz)                           │
│                                                                  │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
│  │ Provider │──▶│  System  │──▶│ Selector │──▶│  Solver  │──▶│ Actuator │
│  │(相机+YOLO│   │(EKF+FSM) │   │(提取目标)│   │(弹道+MPC)│   │(串口云台)│
│  │  +PnP)   │   │          │   │          │   │          │   │          │
│  └──────────┘   └────┬─────┘   └──────────┘   └──────────┘   └────┬─────┘
│                      │                                            │
│                      │        updateGimbalState(feedback)         │
│                      ◀────────────────────────────────────────────┘
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 类型系统

- **InputType**: `ArmorList` = `std::list<Armor>`（含 3D 世界坐标）
- **SystemStateType**: `AutoAimSystemState` — EKF 状态 + 追踪器元数据 + 云台反馈
- **TargetState**: 框架标准类型，从 SystemState 提取

### 2.3 EKF 状态向量

**11 维（普通机器人）**: `[x, vx, y, vy, z, vz, angle, ω, r, r_offset, z_offset]`

**13 维（前哨站）**: 11 维基础上 + `[z_top, z_bottom]`

- `(x, y, z)`: 整车旋转中心的世界坐标 (m)
- `(vx, vy, vz)`: 线速度 (m/s)
- `angle`: 当前朝向 (rad)
- `ω`: 角速度 (rad/s)
- `r`: 旋转半径 (m)
- `r_offset, z_offset`: 装甲板几何偏移

### 2.4 状态机

```
     ┌────── detect_count >= min ──────────────┐
     │                                          │
  LOST ──(found)──▶ DETECTING ─────────────▶ TRACKING
   ▲                  │ (not found)              │ (not found)
   │                  ▼                          ▼
   │               LOST                      TEMP_LOST
   │                  ▲                          │ (found)
   │                  │ temp_lost > max          ▼
   │                  │                       TRACKING
   │                  │
   └──────────────────┘ (timeout)
```

---

## 3. 环境准备

### 3.1 依赖

```bash
# 系统库
sudo apt install libopencv-dev libeigen3-dev libglfw3-dev

# 可选：yaml-cpp (用于读取 spr_vision_try 配置文件)
# 若未安装，自动回退使用 OpenCV FileStorage
sudo apt install libyaml-cpp-dev

# 可选：TinyMPC (用于 MPC 控制)
# 参见 spr_vision_try/tasks/auto_aim/planner/tinympc/
```

### 3.2 spr_vision_try 集成

若需要连接真实相机和云台（非测试模式），设置环境变量：

```bash
export SPR_ROOT=/path/to/spr_vision_try
```

编译时传入：

```bash
cmake .. -DSPR_ROOT=$SPR_ROOT
```

---

## 4. 编译

```bash
cd Simple-Aimer-Runtime/build

# 基础编译（测试模式，不含 spr_vision_try 依赖）
cmake ..
make auto_aim -j$(nproc)

# 完整编译（含 spr_vision_try 的相机/YOLO/Gimbal）
cmake .. -DSPR_ROOT=/path/to/spr_vision_try
make auto_aim -j$(nproc)
```

生成二进制: `build/applications/auto_aim/auto_aim`

---

## 5. 配置文件

配置文件格式与 `spr_vision_try/configs/` 兼容（YAML 1.0 / OpenCV FileStorage）。

### 5.1 必需字段

```yaml
# === 追踪器 ===
enemy_color: "blue"           # 敌方颜色: red / blue
min_detect_count: 5           # 检测确认帧数
max_temp_lost_count: 30       # 普通机器人短暂丢失容忍帧数
outpost_max_temp_lost_count: 60  # 前哨站短暂丢失容忍帧数

# === 偏移量 (deg → 内部转为 rad) ===
yaw_offset: 0.0
pitch_offset: 0.0

# === 弹道 ===
fire_thresh: 2.0              # 开火阈值
decision_speed: 3.0           # 高低速判定角速度 (rad/s)
high_speed_delay_time: 0.1    # 高速延时 (s)
low_speed_delay_time: 0.05    # 低速延时 (s)

# === 云台限幅 (rad/s) ===
max_yaw_rate: 10.0
max_pitch_rate: 5.0
```

### 5.2 完整配置示例

参见 `spr_vision_try/configs/standard4.yaml`（与原项目完全兼容）。

---

## 6. 运行

```bash
# 测试模式（无硬件）
./build/applications/auto_aim/auto_aim

# 指定配置文件
./build/applications/auto_aim/auto_aim configs/standard4.yaml
```

### 6.1 预期输出

```
[AutoAim] Loading config: configs/standard4.yaml
[AutoAim] Starting Runtime pipeline...
[AutoAim] Pipeline running. Press Ctrl+C to stop.
```

### 6.2 IMGUI 调试面板

启动后出现 IMGUI 窗口，包含：
- **Parameters** 面板：实时调节参数滑块
- **Curves** 面板：监控 err_x / err_y / cmd_yaw / cmd_pitch 曲线
- **2D Target View** 面板：目标点可视化
- **Images** 面板：Detection Result / Binary Mask（需 SPR_ROOT）

---

## 7. 完整运行前置流程

完整的瞄准系统部署需要以下步骤（已为后续开发预留空间）：

### 7.1 相机标定

```
目的：获取 camera_matrix 和 distort_coeffs
工具：spr_vision_try/calibration/calibrate_camera.cpp
输出：内参写入 config YAML 的 camera_matrix / distort_coeffs 字段
```

**Runtime 中的位置**：PnPSolverWrapper 通过 `Solver(config_path)` 加载内参。

### 7.2 手眼标定

```
目的：获取相机→云台的变换矩阵 R_camera2gimbal / t_camera2gimbal
工具：spr_vision_try/calibration/calibrate_handeye.cpp
输出：变换矩阵写入 config YAML 的 R_camera2gimbal / t_camera2gimbal 字段
```

**Runtime 中的位置**：PnPSolverWrapper 通过 `Solver(config_path)` 加载外参。

### 7.3 IMU 安装标定

```
目的：获取云台→IMU本体 的变换矩阵 R_gimbal2imubody
工具：spr_vision_try/calibration/calibrate_robotworld_handeye.cpp
输出：写入 config YAML 的 R_gimbal2imubody 字段
```

**Runtime 中的位置**：GimbalActuator 读取云台 IMU 四元数后，通过 `set_R_gimbal2world(q)` 更新 PnP 的世界变换。

### 7.4 YOLO 模型部署

```
目的：将训练好的 ONNX 模型部署到推理引擎
模型：spr_vision_try/assets/*.onnx 或自定义模型
配置：config YAML 中的 model_path / engine_path 字段
```

**Runtime 中的位置**：YoloWrapper 通过 `YOLO(config_path)` 加载模型。

### 7.5 串口通信配置

```
目的：配置与云台下位机的串口通信参数
配置：config YAML 中的 serial_port / baud_rate 字段
协议：GimbalToVision / VisionToGimbal 自定义帧格式
```

**Runtime 中的位置**：GimbalActuator 通过 `GimbalWrapper(config_path)` 初始化串口。

---

## 8. 后续开发空间

### 8.1 MPC 集成（高优先级）

当前 Solver 使用简化 P 控制器。集成 TinyMPC 的步骤：

1. 在 `CMakeLists.txt` 中链接 TinyMPC 库
2. 在 `AutoAimSolver` 构造函数中初始化 yaw/pitch 求解器
3. 在 `solve()` 中替换 P 控制器：
   - 设置 `tiny_set_x0(solver, x0)` 当前云台状态
   - 设置 `solver->work->Xref` 参考轨迹
   - 调用 `tiny_solve(solver)`
   - 读取 `solver->work->x` 和 `work->u` 填入 `GimbalCommand`

参考代码：`spr_vision_try/tasks/auto_aim/planner/planner.cpp` 的 `plan()` 方法。

### 8.2 丢失目标搜索（中优先级）

当前当目标丢失时，Solver 返回零指令。移植 `spr_vision_try` 的扫描搜索逻辑：

- 圆形扫描（`scan_pattern: "circle"`）
- 螺线扫描（`scan_pattern: "spiral"`）

参考代码：`spr_vision_try` 的 `LaserAimerSolver` 中的扫描逻辑（已在 laser_aimer 中实现）。

### 8.3 全向感知多相机融合（中优先级）

当前仅支持单相机。移植 `spr_vision_try` 的 `omniperception` 模块：

- 在 `System` 中增加 `SWITCHING` 状态
- 在 `update()` 中实现切换目标逻辑
- 在 `Selector` 中实现多相机输入的优先级判断

参考代码：`spr_vision_try/tasks/auto_aim/tracker.cpp` 的 `track(DetectionResult)` 重载。

### 8.4 离线数据回放（低优先级）

添加回放 DataProvider，从录制的数据包中读取 YOLO 检测结果，便于离线调参。

---

## 9. 参数调优

### 9.1 EKF 调优

| 参数 | 默认值 | 作用 | 调优方向 |
|------|--------|------|---------|
| `P0_dig` (初始化协方差) | 见 set_target() | 初始不确定性 | 不确定则增大，确定则减小 |
| `v1` (位置过程噪声) | 100 / 10 (outpost) | Q 矩阵缩放 | 运动快则增大 |
| `v2` (角度过程噪声) | 400 / 0.1 (outpost) | Q 矩阵缩放 | 旋转快则增大 |

### 9.2 追踪器调优

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `min_detect_count` | 5 | 确认新目标需要的连续检测帧数 |
| `max_temp_lost_count` | 30 | 短暂丢失的最大容忍帧数 |
| `outpost_max_temp_lost_count` | 60 | 前哨站短暂丢失容忍（因频繁遮挡） |

### 9.3 控制器调优

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `kp_yaw / kp_pitch` | 5.0 | P 控制器增益（MPC 集成后废弃） |
| `max_yaw_rate` | 10.0 rad/s | yaw 最大角速度 |
| `max_pitch_rate` | 5.0 rad/s | pitch 最大角速度 |
| `fire_thresh` | 2.0 | 开火误差阈值 |

---

## 附录

### A. 与原 spr_vision_try 的差异

| 方面 | spr_vision_try | auto_aim (新) |
|------|---------------|--------------|
| 管道组织 | 手写循环，顺序调用 | Runtime 框架，声明式组装 |
| 状态管理 | Target + Tracker 分散 | System 统一管理 SystemStateType |
| 云台反馈 | 通过 plan_thread 传递 | Actuator::feedback() → System::updateGimbalState() |
| 配置加载 | yaml-cpp | yaml-cpp / OpenCV FileStorage 双模式 |
| 调试界面 | WebDebugger + cv::imshow | ImGui 统一调试面板 |
| 测试模式 | 无 | 支持无硬件编译运行（空 Provider/Actuator） |

### B. 文件清单

```
applications/auto_aim/
├── CMakeLists.txt              # 构建配置
├── MANUAL.md                   # 本文档
├── main.cpp                    # 入口，组装 Runtime
├── types.hpp / types.cpp       # 装甲板类型定义
├── extended_kalman_filter.hpp  # EKF 滤波器（移植）
├── extended_kalman_filter.cpp
├── trajectory.hpp              # 弹道计算（移植）
├── AutoAimProvider.hpp/.cpp    # DataProvider（相机+YOLO+PnP）
├── AutoAimSystem.hpp/.cpp      # System（EKF+状态机）
├── AutoAimSelector.hpp         # Selector（目标提取）
├── AutoAimSolver.hpp/.cpp      # Solver（弹道+MPC预留）
└── GimbalActuator.hpp/.cpp     # Actuator（云台驱动）
```
