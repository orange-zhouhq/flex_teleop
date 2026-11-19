# flex_teleop

多轴机械臂 VR 遥操的参考架构。核心设计是**以 `pose_id` 贯穿全链的帧对齐机制**——在 30Hz 稀疏目标、200Hz 关节流、1kHz 伺服轨迹三级时基之间保持无歧义的因果溯源，并让同一套 id 直接服务于多传感器数据对齐与 VLA 数据采集。

> 当前阶段：**核心 + QP IK + ROS 2 适配层可运行**。插值、双入口仲裁、200Hz 编排、可插拔 IK、S-R-S 速度级 QP 与端到端 ROS 2 节点均已实现并验证。

---

## 三张图

| 视图 | 文件 | 回答什么问题 |
|---|---|---|
| **数据流架构** | [page1](assets/page1_30Hz-200Hz-teleop-chain.png) | 每个环节传什么 TARGET、各自维护什么 STATE |
| **帧对齐结构** | [page2](assets/page2_frame-alignment-3layer.png) | pose_id / tick_seq / stamp 如何分工，1:N 组关系如何成立 |
| **控制系统视角** | [page3](assets/page3_control-system-view.png) | 延迟从哪里来、为什么不会失稳、什么可调 |

矢量版在 `assets/*.svg`，可编辑源文件 `assets/flex_teleop_architecture.drawio`（三页）。

---

## 核心设计

### 1. pose_id：帧身份的唯一依据

- 在**源头分配一次**（VR 采集侧），单调递增 `uint64`
- 全链**只读透传**，任何环节不得重生
- **不承担时间语义**：跨机时钟不同源，`stamp` 只做延迟诊断

### 2. 三级时基，各管一段

| 机制 | 作用域 | 能做什么 | 不能做什么 |
|---|---|---|---|
| `pose_id` | 跨机器 · 30Hz | 帧身份、乱序丢弃、丢帧判定、因果分组 | 测延迟 |
| `tick_seq` | 单机 · 200Hz | 指令↔反馈同拍配对、tick 级丢包 | 跨机对齐 |
| `stamp` | 各端本地 | 端到端延迟、录制回放、抖动统计 | 帧身份 |

1kHz 层**不带任何 frame id**——它是本地闭环的实现细节。

### 3. 1:N 组关系，不是合并 ID

30Hz : 200Hz = **3 : 20**（非整数比）。一个 pose 帧对应 6~7 个 tick，逐帧在 6/7 之间循环。

因此每个 200Hz tick 登记它的上游来源：

```
src = (pose_id_a, pose_id_b, alpha)
```

查询 `WHERE id_a <= P < id_b` 即可取出该 pose 帧影响过的全部关节指令。**这是外键，不是合并后的新 ID。**

### 4. 两个入口汇入同一条 200Hz 关节流

- **pose 模式**：VR 位姿 → 插值 → QP IK → 关节
- **joint 模式**：上层直发 30Hz 关节目标（旁路 IK）

两条路径汇聚到同一个流式目标层，按最近一帧仲裁，模式切换需保证 `pose_id` 不断档。

### 5. 视觉 / 传感器的帧对齐

图像本身**不需要 id**。图像到达时刻查询插值器状态，得到 `(id_a, id_b, alpha)` 并附着到样本上：

```
{ image_k, (id_a, id_b, alpha), q_fb, T_actual }
```

无需跨设备时钟同步，帧率不同也能落到同一套 pose 区间。这就是模仿学习数据集"观测-动作对齐"的通用解。

---

## 快速开始

```bash
# 核心（零第三方依赖）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/flex_teleop_minimal

# 可选：启用参考 QP IK 后端（需要 Eigen3 + proxsuite）
cmake -S . -B build-qp -DCMAKE_BUILD_TYPE=Release -DFLEX_TELEOP_WITH_QP_IK=ON
cmake --build build-qp -j
./build-qp/test_qp_backend

# ROS 2 适配层（需要 ROS 2）
source /opt/ros/humble/setup.bash
colcon build --base-paths adapters/ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run flex_teleop_ros teleop_node --ros-args -p ik_type:=none
```

核心只要求 C++17 与 CMake ≥ 3.16；QP 后端额外需要 Eigen3 与 proxsuite（纯头文件）。

最小用法：

```cpp
#include <flex_teleop/teleop_core.hpp>

TeleopCore::Config cfg;
cfg.fixed_delay_s = 0.035;          // 1.05 / 30Hz
TeleopCore core(cfg);
core.setIK(&my_ik);                 // 实现 IKBackend

// 接收线程
core.submitPose(T, pose_id, t_arrival_ns);

// 200Hz 定时器
const JointStream s = core.tick(t_now_ns);
// s.q_cmd / s.src.{id_a,id_b,alpha} / s.mode
```

示例输出（30Hz 正弦 → 200Hz 关节流）：

```
 t(ms)       mode         q0     id_a     id_b alpha
    50       POSE     0.0234        1        2 0.450
   100       POSE     0.1017        3        4 0.000
   150       POSE     0.1644        4        5 0.450
```

---

## 接口契约

**[docs/04-interface.md](docs/04-interface.md)** 定义了框架的全部输入输出：

- 5 个核心数据结构（`PoseFrame` / `JointTarget` / `JointStream` / `JointFeedback` / `TeleopStatus`）
- 时序契约（频率、单调性、超时判据）
- 误差语义（`e_model` / `e_real` / `Δq` 三者的分工）
- 扩展点（IK 后端、传感器打标）

框架核心**不依赖任何特定中间件**，ROS 2 / ZMQ / 共享内存都只作为适配器。

---

## 文档

- [01 数据流架构](docs/01-architecture.md)
- [02 帧对齐](docs/02-frame-alignment.md)
- [03 控制系统视角](docs/03-control-view.md)
- [04 接口契约](docs/04-interface.md)

---

## 仓库结构

```
flex_teleop/
├── README.md
├── CMakeLists.txt
├── core/
│   ├── include/flex_teleop/
│   │   ├── types.hpp              # 6 个数据结构
│   │   ├── pose_math.hpp          # SE(3) 基础运算（零依赖）
│   │   ├── interpolator.hpp       # SE(3) 双缓存插值器
│   │   ├── joint_interpolator.hpp # 关节空间同构实现
│   │   ├── source_mux.hpp         # 双入口仲裁
│   │   ├── ik_backend.hpp         # IK 可插拔抽象
│   │   └── teleop_core.hpp        # 200Hz 编排
│   └── src/
├── plugins/qp_ik/         # 参考 IK 后端：FlexQpIk -> IKBackend 适配
├── adapters/ros2/         # ROS 2 适配层（独立包，不引用任何外部消息定义）
├── third_party/
│   ├── srs_dh_model/      # S-R-S DH 几何与 FK/Jacobian（BSD-3）
│   └── flex_qp_ik/        # 速度级 QP 逆运动学（BSD-3，proxsuite）
├── tests/                 # 82 项单元检查（含 QP 后端冒烟）
├── examples/              # 最小可运行示例
├── docs/                  # 架构与契约文档
└── assets/                # 三视图（PNG / SVG / drawio 源）
```

后续将新增：ZMQ / 共享内存等其它传输适配层。

---

## ROS 2 适配层

`adapters/ros2/` 下两个包，消息定义**完全按 [04-interface.md](docs/04-interface.md) 独立重建**，
不引用任何外部 `robot_interface` 之类的包：

| 包 | 内容 |
|---|---|
| `flex_teleop_msgs` | `PoseFrame` / `JointTarget` / `JointFeedback` / `JointStream` / `TeleopStatus` |
| `flex_teleop_ros` | `teleop_node`：订阅 → 核心 → 发布，不含任何算法逻辑 |

话题（相对节点命名空间）：`pose_target`、`joint_target`、`joint_feedback` 输入；
`joint_stream`（200Hz，reliable）、`teleop_status`（20Hz，best-effort）输出。

⚠️ 这两个包会被 `colcon` 递归扫描到。如果你不希望它们进入既有工作空间的整体构建，
在 `flex_teleop/adapters/` 下放一个空的 `COLCON_IGNORE` 文件即可。

---

## License

- `third_party/` 下的 `srs_dh_model` 与 `flex_qp_ik` 沿用 **BSD-3-Clause**，版权头原样保留
- 框架自身许可证随首个稳定版本确定

`third_party/` 中的几何参数仅为通用 S-R-S 构型的测试值，**不含任何具体机器人型号的产品参数**；实际部署时由使用方通过 `QpIkBackend::Config` 注入。
