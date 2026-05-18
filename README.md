# Simple-Aimer-Runtime
用于快速实现机器人开发中常见的各类根据检测目标建模预测而后控制云台瞄准的任务，具有专为此类任务设计的 runtime，以及可视化调试等实用功能。

## 框架分层

```
                  ┌───────────────────────────────────────────────────────────┐
                  │                     pipeline/                              │
                  │   管线接口（DataProvider / System / Selector / Solver / Actuator）  │
                  │   数据流：InputType → SystemState → FinalTargetState → Command     │
                  └───────────────┬───────────────────────────────────────────┘
                                  │ 依赖
                  ┌──────────────▼───────────────────────────────────────────┐
                  │                     core/                                 │
                  │   核心数据类型：Command / FinalTargetState / SelfState / System 基类  │
                  │   Command 与 System 同级——用户通过 System 建模，通过 Command 输出    │
                  └───────────────┬───────────────────────────────────────────┘
                                  │ 依赖
                  ┌──────────────▼───────────────────────────────────────────┐
                  │                     tools/                                │
                  │   共享算法：EKF / 弹道模型 / TinyMPC 解算器                    │
                  └───────────────────────────────────────────────────────────┘
```

## 数据流

```
                         SystemStateType (用户自定义，必须含 SelfState)
                    ┌──────────────────────────────────────────┐
                    │  SelfState self        (自身运动)         │  ← 强制字段
                    │  ???                   (目标跟踪建模)     │  ← EKF / 状态机 / ...
                    └──────────────────┬──────────────────────┘
                                       │
  ┌──────────────┐    ┌──────────┐    │   ┌────────────┐    ┌─────────┐    ┌──────────┐
  │ DataProvider │───▶│  System  │────┼──▶│  Selector  │───▶│  Solver │───▶│ Actuator │──▶ 硬件
  │ (输入数据)   │    │(维护状态)│    │   │ (提取目标) │    │(解算指令)│    │(执行指令)│
  └──────────────┘    └──────────┘    │   └────────────┘    └─────────┘    └─────┬────┘
                      ┌──────────┐    │                                          │
                      │ Command  │◀───┘           FinalTargetState ──────────────┘
                      │ (同级)   │                (Selector 输出)           ↑ feedback
                      └──────────┘                                          SelfState
```

---

## 核心设计

### 数据类型的对等地位

**SystemStateType** 和 **Command** 处于同一层级，都定义在 `core/`：

```
               core/
               ├── System.hpp    系统基类 + SelfState 定义
               ├── Command.hpp   指令类型
               ├── FinalTargetState.hpp  Selector 输出的标准中间格式
               └── types.hpp     聚合 include

用户通过 SystemStateType 建模（状态是什么），
    通过 Command           输出（告诉硬件做什么）。
```

### 管线组件

| 组件 | 输入 | 职责 | 输出 | 基类路径 |
|------|------|------|------|---------|
| **DataProvider** | (外部传感器) | 获取原始数据 | `InputType` | `pipeline/DataProvider.hpp` |
| **System** | `InputType` | 消化数据，维护全量状态 | `const SystemStateType&` | `core/System.hpp` |
| **Selector** | `SystemStateType` | 提取标准目标 | `FinalTargetState` | `pipeline/Selector.hpp` |
| **Solver** | `FinalTargetState` + `SystemStateType` | 解算控制指令 | `Command` | `pipeline/Solver.hpp` |
| **Actuator** | `Command` | 发送指令 + 反馈 `SelfState` | (硬件) | `pipeline/Actuator.hpp` |

---

## 运行时 (Runtime)

```cpp
template <typename InputType, typename SystemStateType>
class Runtime;

// 每周期循环 (可配置 100Hz):
//   1. provider.fetch(input)
//   2. system.update(input)
//   3. target = selector.select(system.getState())
//   4. cmd    = solver.solve(target, system.getState())
//   5. actuator.send(cmd)
//   6. if feedback = actuator.feedback():
//        system.updateSelfState(feedback)
```

---

## 各应用管线

### auto_aim
```
InputType = std::list<Armor>
StateType = AutoAimSystemState { SelfState self, EKF 状态, 状态机 }

Provider(相机→YOLO→PnP)
  → System(EKF predict/update + 5状态FSM)
    → Selector(EKF → FinalTargetState)
      → Solver(弹道 + TinyMPC + is_fine_aiming)
        → Actuator(串口 VisionToGimbal, is_fine_aiming→开火)
```

### laser_aimer
```
InputType = std::optional<FinalTargetState>
StateType = std::optional<FinalTargetState>  (无 SelfState，标记 valid=false)

Provider(相机→HSV检测)
  → System(透传)
    → Selector(恒等变换)
      → Solver(P控制 + 螺旋扫描)
        → Actuator(未实现)
```

---

## 快速开始

```cpp
// 1. 定义系统状态
struct MyState {
  aim::SelfState self;           // 强制字段
  // 你的跟踪状态
};

// 2. 实现各阶段
class MyProvider  : public aim::DataProvider<MyInput> { … };
class MySystem    : public aim::System<MyInput, MyState> { … };
class MySelector  : public aim::Selector<MyState> { … };
class MySolver    : public aim::Solver<MyState> { … };
class MyActuator  : public aim::Actuator { … };

// 3. 组装运行
aim::Runtime<MyInput, MyState> runtime(
  provider, system, selector, solver, actuator, 100.0);
runtime.start();
runtime.runUI();   // ImGui 调试面板
runtime.stop();
```

---

## 工具库 (`tools/`)

共享算法组件，不依赖 pipeline 管线类型。

| 工具 | 头文件 | 用途 |
|------|--------|------|
| **ExtendedKalmanFilter** | `tools/extended_kalman_filter.hpp` | 通用 EKF（predict / update / 卡方检验） |
| **Trajectory** | `tools/trajectory.hpp` | 理想抛物线弹道解算 |
| **TinyMPCSolver** | `tools/tinympc_solver.hpp` | 运动学 MPC 封装（双积分器模型） |