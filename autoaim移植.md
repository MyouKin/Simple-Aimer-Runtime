先系统读取完整技术链路的所有关键文件。

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/src/auto_aim_debug_mpc.cpp#1-1), lines 1 to 300

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/io/camera.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/io/gimbal/gimbal.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/yolo.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/classifier.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/detector.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/yolo.cpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/solver.cpp#1-1), lines 1 to 200

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/aimer.cpp#1-1), lines 1 to 200

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/aimer.cpp#200-200), lines 200 to 211

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/solver.cpp#200-200), lines 200 to 293

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/io/mindvision/mindvision.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tools/math_tools.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/io/camera.cpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/target.cpp#277-277), lines 277 to 320

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/configs/standard4.yaml#1-1), lines 1 to 100

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/configs/standard3.yaml)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tools/trajectory.cpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/yolos)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/io/command.hpp)

Read [](file:///Users/myoukin/Codes/Simple-Aimer-Runtime/spr_vision_try/tasks/auto_aim/yolos/yolov5.hpp)

我已经读完了全部关键文件。下面是完整的技术流程总结。

---

## spr_vision_try 完整技术流程

### 一张图总览

```
┌──────────────────────────────────────────────────────────────────────────┐
│                      auto_aim_debug_mpc.cpp (main)                       │
│                                                                          │
│  ┌─────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌────────┐ │
│  │ Camera  │──▶│   YOLO   │──▶│  Solver  │──▶│ Tracker  │──▶│Planner │ │
│  │(SDK采集)│   │(检测装甲板)│   │(PnP+姿态)│   │(EKF+FSM) │   │(MPC+弹道)│ │
│  └─────────┘   └──────────┘   └──────────┘   └──────────┘   └───┬────┘ │
│       │                                                          │      │
│       │              ┌─────────────────────┐                     │      │
│       └──────────────│     Gimbal (串口)     │◀────────────────────┘      │
│                      │  收发 + 姿态回读      │                           │
│                      └─────────────────────┘                           │
└──────────────────────────────────────────────────────────────────────────┘
```

实际上是 **两个并行线程**：
- **main 线程**: Camera → YOLO → PnP → Tracker → 入队 target_queue
- **plan 线程**: 从 target_queue 取 → Planner(MPC) → Gimbal.send()

---

### 阶段 0: 相机采集 (`io/camera.hpp/.cpp` + `io/mindvision/`)

```
配置文件:
  camera_name: "mindvision" | "hikrobot"
  exposure_ms: 2.0
  gamma: 0.5         (mindvision专用)
  gain: 16.0         (hikrobot专用)
  vid_pid: "f622:d132"
```

**MindVision SDK 采集流程**（以 `io/mindvision/mindvision.hpp` 为例）：

1. **查找相机**: `CameraSdkInit(1)` → `CameraEnumerateDevice()` 获取设备列表
2. **初始化**: `CameraInit(&dev, -1, -1, &handle)` → `CameraGetCapability(handle, &capability)` 获取分辨率/bayer格式等
3. **设置参数**: `CameraSetAeState(handle, false)` 关闭自动曝光 → `CameraSetExposureTime(handle, exposure_ms)` → `CameraSetAnalogGain(handle, gamma*尺度)`
4. **启动采集**: `CameraPlay(handle)` → `CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_RGB8)`
5. **循环读取**: 后台线程 `CameraGetImageBuffer()` → `CameraImageProcess()` Bayer→RGB → 入队 `ThreadSafeQueue<CameraData>`
6. **对外接口**: `read(cv::Mat &img, timestamp)` 从队列取帧，可选 `cv::flip()`

**HikRobot SDK**: 类似流程，使用海康SDK的 `MV_CC_*` 系列API。

相机驱动采用的是 **CameraBase 虚基类 + Camera 工厂模式**：
```cpp
class CameraBase { virtual void read(cv::Mat&, timestamp) = 0; };
class Camera {
  unique_ptr<CameraBase> camera_;
  // 根据 config["camera_name"] 选择 MindVision 或 HikRobot
};
```

---

### 阶段 1: YOLO 目标检测 (`tasks/auto_aim/yolo.hpp/.cpp`)

```
配置文件:
  yolo_name: "yolov5" | "yolov8" | "yolo11"
  device: "GPU"
  min_confidence: 0.8
  classify_model: "assets/tiny_resnet.onnx"
```

**推理引擎**: OpenVINO (`ov::Core` / `ov::CompiledModel`)，支持三种 YOLO 变体：

| 模型 | 文件 | 输入尺寸 | 输出格式 |
|------|------|---------|---------|
| YOLOV5 | `yolos/yolov5.hpp` | 640x640 | 13类(颜色+编号组合) + keypoints |
| YOLOV8 | `yolos/yolov8.hpp` | 640x640 | 同上 |
| YOLO11 | `yolos/yolo11.hpp` | 640x640 | 同上 |

**YOLOV5 检测流程**（以 `yolov5.cpp` 为例）：

1. **预处理**: `letterbox(img, 640x640)` → `cv::dnn::blobFromImage` → BGR→RGB + /255 归一化
2. **推理**: `compiled_model_.create_infer_request()` → `infer()` → 输出 (1, 255, 80, 80) + (1, 255, 40, 40) + (1, 255, 20, 20) 三个特征层
   - 255 = 3 anchors × (4 bbox + 1 conf + 13 class + 2×4 keypoints)
3. **后处理**: `parse()`: sigmoid 解码 → NMS (IoU 0.3) → 过滤 score < score_threshold_
4. **构造 Armor**: 每 4 个 keypoints → `Armor(class_id, confidence, bbox, keypoints)`
5. **分类器二次确认**: `Classifier(assets/tiny_resnet.onnx)` 对每个候选区域的 pattern 做 23 类分类（颜色×兵种），覆盖 class_id
6. **传统方法补充**: `Detector` 用 HSV 阈值+灯条几何特征做备选检测（`use_traditional: true` 时启用）

**输出**: `std::list<Armor>`，每个 Armor 包含：
- `points[4]`: 装甲板 4 个角点（像素坐标）
- `class_id / color / name / type / confidence`

---

### 阶段 2: PnP 3D 姿态解算 (`tasks/auto_aim/solver.hpp/.cpp`)

```
配置文件:
  camera_matrix:       [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  distort_coeffs:      [k1, k2, p1, p2, k3]
  R_camera2gimbal:     [9个值] (3×3行优先)
  t_camera2gimbal:     [3个值] (m)
  R_gimbal2imubody:    [9个值] (3×3行优先)
```

**坐标变换链**: `Camera → Gimbal → World`

```
     solvePnP                       手眼标定                    IMU变换
  2D像素 ──────→ camera坐标系 ───→ gimbal坐标系 ───→ world坐标系
  (keypoints)    (rvec, tvec)    (R_camera2gimbal)  (R_gimbal2world)
```

**PnP 求解** (solver.cpp):

1. **构建 3D 模型点**: 
   - 大装甲板 (BIG): 宽 230mm, 高 56mm → 4 个角点世界坐标
   - 小装甲板 (SMALL): 宽 135mm, 高 56mm

2. **solvePnP**: `cv::solvePnP(BIG/SMALL_ARMOR_POINTS, armor.points, camera_matrix, distort_coeffs, rvec, tvec, false, SOLVEPNP_IPPE)`
   - 使用 IPPE 算法（对平面物体更稳定）

3. **坐标变换**:
   ```
   xyz_in_camera = tvec
   xyz_in_gimbal = R_camera2gimbal * xyz_in_camera + t_camera2gimbal
   xyz_in_world  = R_gimbal2world * xyz_in_gimbal
   
   R_armor2camera = Rodrigues(rvec)
   ypr_in_gimbal  = eulers(R_camera2gimbal * R_armor2camera)  // ZYX
   ypr_in_world   = eulers(R_gimbal2world * R_armor2gimbal)
   ypd_in_world   = xyz2ypd(xyz_in_world)  // 笛卡尔 → 球坐标
   ```

4. **Yaw 角度优化** (`optimize_yaw`): 在 ±70° 范围内搜索最优 yaw，通过重投影误差最小化：遍历 yaw0…yaw0+140°，按 1° 步长计算 `armor_reprojection_error`，选择误差最小的 yaw，覆盖 `armor.ypr_in_world[0]`。

5. **重投影验证**: `reproject_armor()`: 给定世界坐标的 xyz+yaw → 反算到像素坐标 → `cv::projectPoints()`，用于可视化调试

**动态 IMU 更新**: `set_R_gimbal2world(Eigen::Quaterniond q)`:
```
R_imubody2imuabs = q.toRotationMatrix()
R_gimbal2world   = R_gimbal2imubody^T * R_imubody2imuabs * R_gimbal2imubody
```

---

### 阶段 3: Tracker 追踪器 (`tasks/auto_aim/tracker.hpp/.cpp` + `target.hpp/.cpp`)

这是最核心的状态估计模块。

**状态机** (Tracker) → 5 个状态：

```
         found & detect>=5
  LOST ──→ DETECTING ──→ TRACKING ──(not found)──→ TEMP_LOST
   ▲          │               ▲                        │
   │  not found               │    temp_lost > max     │(found)
   └──────────────────────────┴────────────────────────┘
              timeout / diverged / nis_fail > 40%
```

**Target 目标类** (`target.hpp/.cpp`) — 核心 EKF 模型：

**11 维状态向量**（普通机器人）:
```
[x, vx, y, vy, z, vz, angle, ω, r, r_offset, z_offset]
 位置    速度   旋转中心  角速度  旋转半径  几何偏移
```

**13 维状态向量**（前哨站，3 块板，每块 Z 不同）:
```
[x, vx, y, vy, z, vz, angle, ω, r, 0, 0, z_top, z_bottom]
```
前哨站三块装甲板垂直排列，Z 向间距 100mm。

**EKF 预测** (`Target::predict(dt)`):

状态转移矩阵 F（线性部分）:
```
F = I + dt*偏导   // 对角 2×2 块: [1 dt; 0 1] for 位置×速度
```

过程噪声 Q（分段常数加速度模型）:
```
Q(i,i) = a*v  (位置→位置),  c*v  (速度→速度)  其中 a=dt⁴/4, c=dt²
v = 100 (普通) / 10 (outpost)  for 平移分量
v = 400 (普通) / 0.1 (outpost) for 旋转分量
```

预测后做角度归一化: `x[6] = limit_rad(x[6])`

**前哨站角速度钳制**: 收敛后若 |ω| > 2 rad/s，钳制到 ±2.51 rad/s（对应物理角速度上限 0.4 转/秒）。

**EKF 观测更新** (`Target::update → update_ypda`):

1. **装甲板匹配**: 找与观测量 yaw 角度最近的那块板
2. **观测函数 h(x)**: 从状态 x 计算期望观测 [yaw, pitch, distance, 装甲板yaw]
   ```
   h(x) = {
     yaw   = atan2(armor_y, armor_x)
     pitch = atan2(armor_z, sqrt(armor_x²+armor_y²))
     dist  = sqrt(armor_x²+armor_y²+armor_z²)
     angle = limit_rad(x[6] + id * 2π/armor_num)
   }
   ```
3. **观测噪声 R**: 动态缩放
   ```
   R_diag = [4e-3*r_scale, 4e-3*r_scale, log(|Δangle|+1)+1, log(d+1)/200+9e-2]
   ```
   r_scale 针对前哨站做了放大（yaw 残差越大，噪声越大）
4. **雅可比**: `h_jacobian()` 数值微分 (ε=1e-6)
5. **残差**: 角度分量 wrap 到 [-π, π]

**发散/收敛检测**:
- 发散: `r < 0 || r > 0.7` 或 `r + r_offset < 0.07 || r + r_offset > 0.9`
- 收敛: update_count > 3 (普通) 或 > 50 (outpost) 且未发散
- 收敛质量: `recent_nis_failures / window_size >= 40%` → 丢弃目标

**目标初始化** (`Tracker::set_target`):

根据兵种选择不同的初始协方差:
| 兵种 | r | armor_num | P0_diag |
|------|---|-----------|---------|
| 平衡步兵(big 3/4/5) | 0.2 | 2 | [1,64,1,64,1,64,0.4,100,1,1,1] |
| 前哨站 | 0.2765 | 3 | [1,64,1,64,1,81,0.4,100,1e-4,0,0,100,100] |
| 基地 | 0.3205 | 3 | [1,64,1,64,1,64,0.4,100,1e-4,0,0] |
| 普通 | 0.2 | 4 | [1,64,1,64,1,64,0.4,100,1,1,1] |

初始状态从第一帧观测的 xyz/ypr 推算旋转中心:
```
center_x = xyz[0] + r * cos(ypr[0])
center_y = xyz[1] + r * sin(ypr[0])
center_z = xyz[2]
x0 = [center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0]
```

---

### 阶段 4: Planner 规划器 — MPC + 弹道 (`tasks/auto_aim/planner/`)

这是最终的控制输出阶段。分为两个子任务：

#### 4a. 弹道解算 (aimer.cpp / trajectory.cpp)

```
配置文件:
  yaw_offset: 2.0      # 度 → /57.3 转为弧度
  pitch_offset: 6.5
  decision_speed: 7    # rad/s, 高低速判定
  high_speed_delay_time: 0.1  # s
  low_speed_delay_time: 0.05
  fire_thresh: 0.0035  # 开火角度阈值
  comming_angle: 55    # 度
  leaving_angle: 25
```

**弹道模型** (`tools/trajectory.cpp`):

忽略空气阻力，抛物线解析解：
```
a = g*d²/(2v₀²),  b = -d,  c = a + h
Δ = b² - 4ac

tanθ = (-b ± √Δ) / (2a)
选择飞行时间较短的那个解
pitch = arctan(tanθ)
fly_time = d / (v₀*cos(pitch))
```

**瞄准点选择** (`Aimer::choose_aim_point` / `Planner::choose_aim_xyza`):

1. 从 EKF 状态提取所有装甲板的世界坐标 `armor_xyza_list[id] = [x, y, z, yaw]`
2. **静态模式** (|ω| ≤ 2 且非前哨站):
   - 选 `|δ_angle| < 60°` 范围内的装甲板
   - 如果有两块，进入锁定模式（防止在两块板之间跳变）
3. **旋转模式** (|ω| > 2 或前哨站):
   - 选正在迎来的那块板：`coming_angle < |δ|` 且 `leaving_angle > |δ|`

**弹道迭代** (`Aimer::aim`):

最多 10 次迭代求解飞行时间：
```
for iter in 0..9:
  1. 预测目标在 (future + prev_fly_time) 时刻的位置
  2. 选瞄准点
  3. 弹道解算 → new_fly_time
  4. if |new_fly_time - prev_fly_time| < 0.001 → 收敛退出
```

#### 4b. MPC 控制 (planner.cpp)

```
配置文件:
  max_yaw_acc: 50       # rad/s²
  Q_yaw: [9e6, 0]       # 状态权重 [角度, 角速度]
  R_yaw: [1]             # 控制权重 [角加速度]
  max_pitch_acc: 100
  Q_pitch: [9e6, 0]
  R_pitch: [1]
```

**模型** (planner.hpp):
```
DT = 0.01s
HALF_HORIZON = 50, HORIZON = 100  # 覆盖 -0.5s ~ +0.5s

双积分器: A=[1 DT; 0 1], B=[0; DT]
状态: [角度, 角速度], 控制: 角加速度
约束: |u| ≤ max_acc
```

**MPC 流程** (`Planner::plan`):

1. **弹道解算**: `Trajectory(bullet_speed, d, z)` → fly_time
2. **生成参考轨迹** (`get_trajectory`):
   ```
   for i in 0..HORIZON:
     predict_forward(DT)     # EKF 前向传播
     [yaw, pitch] = aim_ballistic()  # 弹道解算
     yaw_vel = Δyaw/(2*DT), pitch_vel = Δpitch/(2*DT)
     traj.col(i) = [yaw-yaw0, yaw_vel, pitch, pitch_vel]
   ```
   这是一个 4×100 的矩阵：yaw轨迹、yaw_vel轨迹、pitch轨迹、pitch_vel轨迹。

3. **TinyMPC 求解**: 分别对 yaw 和 pitch 轴
   ```
   tiny_set_x0(solver, [traj(0,0), traj(1,0)])
   solver->Xref = traj.block(0,0,2,HORIZON)
   tiny_solve(solver)
   → solver->x(0:1, HALF_HORIZON)  = 优化后状态
   → solver->u(0, HALF_HORIZON)    = 优化后控制
   ```

4. **开火判断**:
   ```
   fire = hypot(traj_ref(H+HALF+2) - mpc_traj(H+HALF+2)) < fire_thresh
   ```
   在 `HALF_HORIZON + 2` 步（约 0.52 秒后），比较参考轨迹与 MPC 轨迹之间的 2D 误差。误差 < 阈值 → 可以开火。

5. **输出** (`Plan` 结构体):
   ```cpp
   { control: true, fire: bool,
     yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc,
     target_yaw, target_pitch }
   ```

---

### 阶段 5: Gimbal 云台通信 (`io/gimbal/gimbal.hpp`)

**串口协议** — 自定义帧格式 (最大 64 字节):

**下行帧 (Vision→Gimbal)**: 视觉发送给云台的控制指令
```
[0] 'S'   [1] 'P'
[2] mode    0=不控制 1=仅控制 2=控制且开火
[3-6]   yaw       (float)
[7-10]  yaw_vel   (float)
[11-14] yaw_acc   (float)
[15-18] pitch     (float)
[19-22] pitch_vel (float)
[23-26] pitch_acc (float)
[27]    tail=0xef
```

**上行帧 (Gimbal→Vision)**: 云台回传给视觉的状态
```
[0] 'S'   [1] 'P'
[2] mode    0=空闲 1=自瞄 2=小符 3=大符
[3-6]   q[0] (w)  (float)
[7-10]  q[1] (x)  (float)
[11-14] q[2] (y)  (float)
[15-18] q[3] (z)  (float)  ← IMU 四元数 wxyz
[19-22] yaw        (float)
[23-26] yaw_vel    (float)
[27-30] pitch      (float)
[31-34] pitch_vel  (float)
[35-38] bullet_speed (float)
[39-40] bullet_count (uint16)
[41]    tail=0xef
```

**Gimbal 类**:
- 后台线程持续 `read(frame)` → 解析 GimbalToVision → `state_` 更新
- `send(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)` → 打包 VisionToGimbal 帧 → 串口发送
- `q(timestamp)` → 从 IMU 四元数队列中取对应时间的姿态，线性外插补

---

### 完整的帧周期时序

```
一帧的完整流程 (main 线程):
┌──────────────────────────────────────────────────────┐
│ 1. camera.read(img, t)           ← 取一帧图像+时间戳      │
│ 2. q = gimbal.q(t)               ← 取IMU姿态(对应帧时刻)   │
│ 3. solver.set_R_gimbal2world(q)   ← 更新世界变换矩阵       │
│ 4. armors = yolo.detect(img)      ← YOLO推理 → Armor列表  │
│ 5. targets = tracker.track(armors,t) ← EKF+状态机 → 目标  │
│ 6. target_queue.push(target)      ← 入队给 plan 线程       │
│ 7. reprojection 可视化            ← 画到 img 上            │
│ 8. web_debugger 推送              ← 给浏览器调试界面        │
│ 9. 帧率控制 (无固定频率)          ← 摄像头帧率驱动          │
└──────────────────────────────────────────────────────┘

plan 线程 (独立运行, 100Hz):
┌──────────────────────────────────────────────────────┐
│ 1. target = target_queue.front()  ← 取最新目标          │
│ 2. gs = gimbal.state()            ← 取云台反馈          │
│ 3. plan = planner.plan(target, gs.bullet_speed)        │
│    ├── 弹道解算 + 飞行时间迭代                          │
│    ├── 参考轨迹生成 (100步)                             │
│    ├── yaw MPC 求解                                    │
│    ├── pitch MPC 求解                                  │
│    └── 开火判定                                        │
│ 4. gimbal.send(plan)              ← 发送控制指令        │
│ 5. plotter.plot(data)             ← 发送遥测数据        │
│ 6. sleep(10ms)                    ← 保持 100Hz          │
└──────────────────────────────────────────────────────┘
```

### 标定依赖

在系统工作前，必须完成的标定工作（配置文件中的静态参数）：

| 标定项 | 工具 | 输出字段 | 用途 |
|--------|------|---------|------|
| **相机内参** | `calibration/calibrate_camera.cpp` | `camera_matrix`, `distort_coeffs` | PnP 解算 |
| **手眼标定** | `calibration/calibrate_handeye.cpp` | `R_camera2gimbal`, `t_camera2gimbal` | 相机→云台变换 |
| **IMU外参** | `calibration/calibrate_robotworld_handeye.cpp` | `R_gimbal2imubody` | 云台→IMU本体变换 |

---

### 与 Runtime 框架的映射

| spr 组件 | → | Runtime 组件 | 移植状态 |
|----------|---|-------------|---------|
| `io::Camera` + `io::MindVision` | → | `AutoAimProvider` (内部) | 待实现 |
| `auto_aim::YOLO` | → | `AutoAimProvider` (内部) | 待实现 |
| `auto_aim::Solver` (PnP) | → | `AutoAimProvider` (内部) | 待实现 |
| `auto_aim::Tracker` + `Target` | → | `AutoAimSystem` | ✅ 已移植 |
| 装甲板排序+过滤 | → | `AutoAimSelector` | ✅ 已移植 |
| `Planner` (MPC+弹道) + `Aimer` | → | `AutoAimSolver` | ✅ 已移植 |
| `io::Gimbal` | → | `GimbalActuator` | 存根，待实现 |

---

这就是完整的从相机打开到云台控制的全部技术细节。