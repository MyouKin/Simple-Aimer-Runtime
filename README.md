# Simple-Aimer-Runtime

面向机器人瞄准任务的轻量级管线框架。两大核心能力：

- **Runtime** — 将 Detection → Tracking → Control 分解为可插拔的管线阶段，以固定频率驱动
- **Debug** — 独立 OS 窗口的日志 / 图像 / 曲线可视化，零侵入注入管线

> 已落地应用：`auto_aim`（YOLO + EKF + 弹道 MPC）、`laser_aimer`（HSV + P 控制）

---

## 目录

1. [核心能力一：Runtime](#核心能力一runtime)
2. [核心能力二：Debug](#核心能力二debug)
3. [管线组件速览](#管线组件速览)
4. [开发新应用](#开发新应用)
5. [已有应用](#已有应用)
6. [工具库](#工具库)
7. [项目结构](#项目结构)
8. [构建](#构建)

---

## 核心能力一：Runtime

### 是什么

`Runtime<InputType, SystemStateType>` 是一个模板化的**固定频率管线循环**，它不关心你的输入是 YOLO 检测框还是 HSV 像素点，不关心你的控制算法是 MPC 还是 PID——它只负责按固定节拍串联五个阶段。

```
┌────────────────────────────────  Runtime (默认 100Hz)  ────────────────────────────────┐
│                                                                                       │
│  ┌──────────────┐    ┌──────────┐    ┌────────────┐    ┌─────────┐    ┌──────────┐   │
│  │ DataProvider │───▶│  System  │───▶│  Selector  │───▶│  Solver │───▶│ Actuator │   │
│  │  采集数据     │    │ 维护状态  │    │  提取目标   │    │ 解算指令  │    │ 执行反馈  │   │
│  └──────────────┘    └──────────┘    └────────────┘    └─────────┘    └─────┬────┘   │
│                                                                             │        │
│                               SelfState feedback ◀──────────────────────────┘        │
└───────────────────────────────────────────────────────────────────────────────────────┘
```

### 循环逻辑（源码级）

```cpp
// Runtime 每帧执行（pipelineLoop 核心）：
//
// 0. 取 Actuator 最新反馈 → 非阻塞
//    if (actuator_) fb = actuator_->feedback();
//
// 1. 采集数据
//    provider->fetch(input);
//
// 2. 消化数据 + 回写自身状态
//    system->update(input);
//    system->updateSelfState(*fb);
//
// 3. 从全量状态中提取目标
//    target = selector->select(system->getState());
//
// 4. 解算控制指令
//    cmd = solver->solve(target, system->getState());
//
// 5. 发送到硬件
//    actuator->send(cmd);
```

### 关键设计决策

| 决策 | 说明 |
|------|------|
| **纯管线循环，零 Debug 耦合** | Runtime 不知道 ImGui、不知道日志窗口，只跑管线 |
| **调试窗口由应用层按需创建** | 在 `main()` 中 `AimerLogger::instance().showWindow()` |
| **Actuator 可选** | 纯算法验证时可传 `nullptr`，Solver 输出仅做日志 |
| **帧率可控** | 构造函数最后一个参数 `loop_rate_hz`，默认 100 Hz |
| **FrameTimer 平滑帧间隔** | `runtime.frameDt()` 返回指数平滑后的 dt，可用于外部监控 |

### 数据流中三个关键类型

```
     InputType                    SystemStateType                   Command
  (用户自定义)              (用户自定义，必须含 SelfState)         (框架固定)
 ┌──────────────┐     ┌──────────────────────────────┐     ┌──────────────────┐
 │ 相机的 Armor  │     │ SelfState self  ← 框架强制字段  │     │ yaw, pitch       │
 │ 激光的目标点  │     │ ???  ← 你的 EKF / 状态机 / …   │     │ yaw_vel, pitch_vel│
 │ ...          │     │                              │     │ yaw_acc, pitch_acc│
 └──────────────┘     └──────────────┬───────────────┘     │ is_fine_aiming   │
                                    │                     └──────────────────┘
                                    ▼
                           FinalTargetState
                          (框架固定，Selector → Solver 的中间格式)
                          ┌──────────────────┐
                          │ position, velocity│
                          │ acceleration      │
                          │ euler, image_point│
                          └──────────────────┘
```

- **InputType** 和 **SystemStateType** 由应用自由定义
- **FinalTargetState** 和 **Command** 由框架提供，是所有应用通用的"中间语言"
- **SelfState** 是 SystemStateType 的强制成员——Actuator 通过它把云台姿态回传给 Solver

---

## 核心能力二：Debug

调试系统由 **DebugContext**（数据桥）+ 三款**独立 OS 窗口**组成，各自运行在独立线程，从管线线程通过线程安全接口推送数据。

### 架构

```
Pipeline Thread                          Independent OS Windows
(100Hz)                                  (各自在独立线程)
─────────────                            ─────────────────────
                                         
  Solver ──pushCurveData()──▶ DebugContext ──▶ AimerCurve 窗口
                                    │           (多实例，多坐标轴)
  Provider ──setImage() ────────────┤
                                    │
  AimerLogger::info() ──────────────┼──▶ AimerLogger 窗口
                                    │    (spdlog 环形缓冲，单例)
  System ──setTarget2D() ──────────┘
```

### AimerLogger — 日志窗口

```cpp
#include "debug/AimerLogger.hpp"

// 在管线任意位置调用（线程安全）
aim::AimerLogger::instance().info("Target locked, distance={:.2f}m", dist);
aim::AimerLogger::instance().warn("EKF divergence detected");
aim::AimerLogger::instance().error("Serial timeout");

// 在 main() 中打开窗口
aim::AimerLogger::instance().showWindow();
```

- 基于 spdlog 环形缓冲区
- 支持文本复制、级别过滤
- 窗口大小/位置由 `imgui.ini` 自动记忆
- 也可获取底层 `spdlog::logger` 用 `SPDLOG_INFO` 等宏

### AimerImage — 图像窗口

```cpp
#include "debug/AimerImage.hpp"

aim::AimerImage cam("Front Camera");
cam.openWindow();       // 打开独立 OS 窗口

// 在 Provider 线程中推送（线程安全）
cv::Mat frame = ...;    // 画上检测框后的图像
cam.show(frame);
```

- 多实例——可以为多个相机各创建一个窗口
- 自动从 `cv::Mat` 创建 OpenGL 纹理
- 窗口可任意拖拽、缩放

### AimerCurve — 曲线窗口

```cpp
#include "debug/AimerCurve.hpp"

aim::AimerCurve curves("Control");
curves.openWindow();

// 从 Solver 线程推送（线程安全）
curves.push("yaw_error",  yaw_err);
curves.push("pitch_error", pitch_err);
curves.push("mpc_u",       mpc_output);
```

- 多实例、多坐标轴——用户在窗口中自由创建坐标轴并分配曲线
- 自动滑动窗口（保留最近 1000 个数据点）
- 实时缩放/平移

### DebugContext — 共享数据桥

三个窗口的数据源。全局单例，所有管线组件通过它交换调试数据：

| 接口 | 用途 |
|------|------|
| `pushCurveData(name, value)` | 推送时序数据到曲线 |
| `setImage(name, mat)` | 更新图像 |
| `setTarget2D(point, valid)` | 设定 2D 瞄准点 |

---

## 管线组件速览

全部在 `include/pipeline/` 下，都是纯虚基类，你需要继承并实现：

| 组件 | 头文件 | 你需要实现 |
|------|--------|-----------|
| `DataProvider<InputType>` | `DataProvider.hpp` | `fetch(InputType&) → bool` |
| `System<InputType, StateType>` | `System.hpp` | `update(InputType)`, `getState() → StateType&` |
| `Selector<StateType>` | `Selector.hpp` | `select(StateType) → FinalTargetState` |
| `Solver<StateType>` | `Solver.hpp` | `solve(FinalTargetState, StateType) → Command` |
| `Actuator` | `Actuator.hpp` | `send(Command)`, `feedback() → optional<SelfState>` |

---

## 开发新应用

### Step 1：定义你的类型

```cpp
// my_aimer/types.hpp
#include "pipeline/System.hpp"
#include <opencv2/opencv.hpp>

// 你的原始输入
struct MyInput {
    cv::Mat frame;
    std::vector<cv::Rect> detections;
};

// 你的系统状态（强制包含 SelfState self）
struct MyState {
    aim::SelfState self;          // ← 必须！框架通过它回写云台姿态
    cv::Point3d target_position;   // 你的建模：3D 目标位置
    cv::Point3d target_velocity;   // 你的建模：目标速度
    bool is_locked;
};
```

### Step 2：实现五个组件

```cpp
// MyProvider.hpp
class MyProvider : public aim::DataProvider<MyInput> {
public:
    bool fetch(MyInput& out) override {
        // 从相机读帧 + 运行检测模型
        out.frame = camera_.read();
        out.detections = detector_.detect(out.frame);
        return !out.detections.empty();
    }
private:
    Camera camera_;
    Detector detector_;
};

// MySystem.hpp
class MySystem : public aim::System<MyInput, MyState> {
public:
    void update(const MyInput& input) override {
        // 用 EKF / 状态机消化检测结果
        ekf_.predict(dt);
        if (!input.detections.empty()) {
            ekf_.update(measurement);
        }
        state_.target_position = ekf_.position();
        state_.target_velocity = ekf_.velocity();
        state_.is_locked = ekf_.isConverged();
    }
    void updateSelfState(const aim::SelfState& s) override {
        state_.self = s;  // 接收云台反馈
    }
    const MyState& getState() const override { return state_; }
private:
    MyState state_;
    aim::tools::ExtendedKalmanFilter ekf_;
};

// MySelector.hpp
class MySelector : public aim::Selector<MyState> {
public:
    aim::FinalTargetState select(const MyState& s) override {
        aim::FinalTargetState t;
        if (!s.is_locked) { t.valid = false; return t; }
        t.valid = true;
        t.has_position = true;
        t.position = aim::Vec3(s.target_position.x,
                               s.target_position.y,
                               s.target_position.z);
        t.has_velocity = true;
        t.velocity = aim::Vec3(s.target_velocity.x,
                              s.target_velocity.y,
                              s.target_velocity.z);
        return t;
    }
};

// MySolver.hpp
class MySolver : public aim::Solver<MyState> {
public:
    aim::Command solve(const aim::FinalTargetState& target,
                       const MyState& state) override {
        aim::Command cmd;
        if (!target.valid) return cmd;

        // 弹道解算 + 运动学 MPC / PID ...你的控制逻辑
        double fly_time = computeTrajectory(target.position, state.self.bullet_speed);
        cmd.yaw   = target_yaw;
        cmd.pitch = target_pitch + fly_time_offset;
        return cmd;
    }
};

// MyActuator.hpp
class MyActuator : public aim::Actuator {
public:
    void send(const aim::Command& cmd) override {
        serial_.write(packFrame(cmd));  // 按你的硬件协议封包
    }
    std::optional<aim::SelfState> feedback() override {
        auto raw = serial_.readFeedback();
        if (!raw) return std::nullopt;
        return parseSelfState(*raw);
    }
private:
    SerialPort serial_;
};
```

### Step 3：组装并运行

```cpp
// main.cpp
#include "runtime/Runtime.hpp"
#include "debug/AimerLogger.hpp"
#include "debug/AimerImage.hpp"
#include "debug/AimerCurve.hpp"

int main() {
    auto provider  = std::make_shared<MyProvider>();
    auto system    = std::make_shared<MySystem>();
    auto selector  = std::make_shared<MySelector>();
    auto solver    = std::make_shared<MySolver>();
    auto actuator  = std::make_shared<MyActuator>();

    // 100Hz 管线
    aim::Runtime<MyInput, MyState> runtime(
        provider, system, selector, solver, actuator, 100.0);
    runtime.start();

    // 打开调试窗口（按需，可选）
    aim::AimerLogger::instance().showWindow();
    // aim::AimerImage preview("Preview"); preview.openWindow();
    // aim::AimerCurve signals("Signals"); signals.openWindow();

    aim::waitForShutdown();  // 阻塞直到 Ctrl+C
    runtime.stop();
    return 0;
}
```

### 无硬件时的纯算法调试

Actuator 是可选的——传 `nullptr` 即可纯跑算法：

```cpp
aim::Runtime<MyInput, MyState> runtime(
    provider, system, selector, solver,
    nullptr,   // ← 无硬件
    100.0);
```

Solver 的 `Command` 输出仍然可以推到 `AimerCurve` 窗口中观察。

---

## 已有应用

### auto_aim — YOLO + EKF + 弹道 MPC

```
InputType = std::list<Armor>
StateType = AutoAimSystemState { SelfState self, EKF 状态, 5-状态 FSM }
```

| 阶段 | 实现 | 说明 |
|------|------|------|
| Provider | `AutoAimProvider` | 相机 → YOLO → PnP → 装甲板列表 |
| System | `AutoAimSystem` | 11/13 维 EKF（predict + YPDA update）+ 追踪/丢失/等待 5 状态 FSM |
| Selector | `AutoAimSelector` | EKF 状态 → `FinalTargetState`（含位姿、速度） |
| Solver | `AutoAimSolver` | 抛物线弹道 + TinyMPC 双积分器 + 精瞄判定 |
| Actuator | `AutoAimGimbal` | `GimbalActuator` 封装串口 VisionToGimbal 帧 + IMU 四元数插值 |

详见 [`applications/auto_aim/MANUAL.md`](applications/auto_aim/MANUAL.md)

### laser_aimer — HSV + P 控制

```
InputType = std::optional<FinalTargetState>
StateType = std::optional<FinalTargetState>
```

| 阶段 | 实现 | 说明 |
|------|------|------|
| Provider | `LaserAimerProvider` | 相机 → HSV 阈值 + 形态学 → 目标点 |
| System | 透传（内联定义） | 直接持有 `optional<FinalTargetState>` |
| Selector | 恒等映射（内联定义） | `has_value() → 直接返回` |
| Solver | `LaserAimerSolver` | P 控制 + 螺旋扫描 + 光轴偏移视差 |
| Actuator | 未实现 | — |

---

## 工具库

`include/tools/` 下的共享算法，不依赖任何管线类型，所有应用可直接复用：

| 工具 | 头文件 | 用途 |
|------|--------|------|
| **ExtendedKalmanFilter** | `tools/extended_kalman_filter.hpp` | 通用 EKF（predict / update / 卡方检验 / NIS 统计） |
| **Trajectory** | `tools/trajectory.hpp` | 理想抛物线弹道（v₀, d, h → fly_time, pitch） |
| **TinyMPCSolver** | `tools/tinympc_solver.hpp` | 运动学 MPC（双积分器模型），可选 TinyMPC 后端 |

---

## 项目结构

```
Simple-Aimer-Runtime/
├── include/
│   ├── pipeline/          # 管线虚基类 + 标准类型
│   │   ├── DataProvider.hpp
│   │   ├── System.hpp          # System 基类 + SelfState 定义
│   │   ├── Selector.hpp
│   │   ├── Solver.hpp
│   │   ├── Actuator.hpp
│   │   ├── Command.hpp         # 通用云台控制指令
│   │   └── FinalTargetState.hpp # Selector → Solver 中间格式
│   ├── core/              # 辅助组件
│   │   ├── FrameTimer.hpp      # 平滑帧间隔计时器
│   │   ├── Registry.hpp        # 参数注册表（ImGui 可调参）
│   │   ├── DebugContext.hpp    # 调试数据桥（单例）
│   │   └── types.hpp           # 向后兼容聚合 include
│   ├── runtime/
│   │   └── Runtime.hpp         # ★ 固定频率管线循环
│   ├── debug/             # ★ 独立 OS 调试窗口
│   │   ├── AimerLogger.hpp     # 日志窗口（spdlog）
│   │   ├── AimerImage.hpp      # 图像窗口（OpenCV → GL 纹理）
│   │   ├── AimerCurve.hpp      # 曲线窗口（多实例多坐标轴）
│   │   └── ImGuiDebugger.hpp   # 集成调试面板（已弃用，被上述独立窗口取代）
│   └── tools/             # 共享算法
│       ├── extended_kalman_filter.hpp
│       ├── trajectory.hpp
│       └── tinympc_solver.hpp
├── applications/
│   ├── auto_aim/          # YOLO + EKF + 弹道 MPC
│   └── laser_aimer/       # HSV + P 控制
├── src/                   # 实现文件
└── CMakeLists.txt
```

---

## 构建

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

可选：启用 TinyMPC 支持
```bash
cmake .. -DTINYMPC_INCLUDE_DIR=/path/to/TinyMPC/include
```

生成的应用二进制在 `build/applications/auto_aim/` 和 `build/applications/laser_aimer/` 下。