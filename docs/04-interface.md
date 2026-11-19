# 04 · 接口契约

本文件定义 `flex_teleop` 框架的**全部外部边界**。框架核心不依赖任何中间件、不假设部署形态；ROS 2、ZMQ、共享内存、进程内直调都只是这套契约的传输载体。

---

## 设计原则

1. **契约先于实现**。数据结构、时序、误差语义先固定，实现可替换。
2. **框架不认识"你的机器人"**。只认下面 6 个结构；URDF、DH 参数、话题名全部由适配层注入。
3. **id 是身份，时间戳是诊断**。二者不可互相替代。
4. **单向数据流 + 单向反馈**。外环不闭合，稳定性由伺服独立保证。

---

## 分层

```
┌──────────────────────────────────────────────────────┐
│  adapters/       ROS 2 · ZMQ · 进程内                  │  ← 只做序列化与转发
├──────────────────────────────────────────────────────┤
│  core/           坐标变换 · 插值 · 仲裁 · 状态机         │  ← 零中间件依赖
├──────────────────────────────────────────────────────┤
│  plugins/        IK 后端 · 传感器打标                    │  ← 可插拔
└──────────────────────────────────────────────────────┘
```

依赖方向**只能向下**。`core` 不得 `#include` 任何适配层头文件。

---

## 数据结构

以下为语言无关定义。`float64` 为 IEEE-754 双精度；矩阵为**行主序 4×4 齐次变换**。

### 1. `VrPose` — 框架输入 A（原始位姿）

| 字段 | 类型 | 语义 |
|---|---|---|
| `pose_id` | `uint64` | **源头分配**，单调递增，全链只读 |
| `stamp_ns` | `int64` | 采集侧时钟（仅诊断，不参与对齐） |
| `tracking_mode` | `uint8` | 0 = controller，1 = hand |
| `left_pose` / `right_pose` / `head_pose` | `float64[7]` | `[qx,qy,qz,qw, x,y,z]`，OpenXR 原始世界系 |
| `valid_L` / `valid_R` / `head_valid` | `bool` | 跟踪有效性；false 时下游必须冻结而非跳变 |
| `pinch_L` / `pinch_R` | `float64` | 夹爪映射（透传） |

**坐标系**：OpenXR 世界系。基座变换、锚点、重定位全部在 `core` 的 `FrameMapper` 内完成，**不进协议**。

**保活要求**：即使位姿不变，也必须按标称频率出帧并递增 `pose_id`。否则下游无法区分"静止"与"断链"。

### 2. `PoseFrame` — 框架内部（转换后目标）

| 字段 | 类型 | 语义 |
|---|---|---|
| `pose_id` | `uint64` | 原样透传 |
| `side` | `uint8` | 0 = left，1 = right |
| `T_base_tcp` | `float64[16]` | 基座系下末端目标位姿 |
| `stamp_ns` | `int64` | 诊断用 |
| `pinch` | `float64` | 透传 |

### 3. `JointTarget` — 框架输入 B（关节旁路）

| 字段 | 类型 | 语义 |
|---|---|---|
| `pose_id` | `uint64` | 与 pose 通路共用同一序列空间，保证模式切换不断档 |
| `side` | `uint8` | 0 / 1 |
| `q` | `float64[7]` | 关节目标（rad） |

### 4. `JointStream` — 框架输出 A（流式目标）

| 字段 | 类型 | 语义 |
|---|---|---|
| `tick_seq` | `uint64` | 200Hz 本地单调计数，从 0 开始 |
| `side` | `uint8` | 0 / 1 |
| `q_cmd` | `float64[7]` | 关节指令（rad） |
| `src.id_a` | `uint64` | 上游区间左端 |
| `src.id_b` | `uint64` | 上游区间右端 |
| `src.alpha` | `float64` | `[0, 1]`，在本区间内的位置 |
| `mode` | `uint8` | 0 = POSE（经 IK），1 = JOINT（旁路），2 = HOLD |
| `solver` | `uint8` | 0 = QP，1 = analytic，2 = numerical，3 = hold |

**语义**：`q_cmd` 是"这一拍要追到的关节位置"，由 `src` 三元组唯一确定其上游来源。`mode == HOLD` 时 `src` **冻结**在最后一次成功的那一帧。

**输出要求**：连续流，每拍必发，不"变化才发"。

### 5. `JointFeedback` — 框架输入 C

| 字段 | 类型 | 语义 |
|---|---|---|
| `tick_seq` | `uint64` | 与 `JointStream` 同拍配对 |
| `side` | `uint8` | 0 / 1 |
| `q` | `float64[7]` | 实际关节角（rad） |
| `qd` | `float64[7]` | 关节速度（rad/s） |
| `tau` | `float64[7]` | 关节力矩（N·m，可选，全 0 表示不支持） |

反馈频率 **≥ 200Hz**。若驱动器只提供更低频率，由适配层做零阶保持，但 `tick_seq` 必须标出它实际对应的拍号。

### 6. `TeleopStatus` — 框架输出 B（状态上报）

| 字段 | 类型 | 语义 |
|---|---|---|
| `stamp_ns` | `int64` | 本端时钟 |
| `side` | `uint8` | 0 / 1 |
| `state` | `uint8` | 0 TRACKING / 1 HOLD / 2 DEGRADED / 3 FAULT |
| `mode` | `uint8` | 同 `JointStream.mode` |
| `pose_id_ack` | `uint64` | 已完成下发的最大 `pose_id` |
| `src.id_a` / `src.id_b` / `src.alpha` | | 当前流的溯源三元组 |
| `e_model.pos` / `e_model.rot` | `float64` | `r − FK(q_prev)`，米 / 弧度 |
| `e_real.pos` / `e_real.rot` | `float64` | `r − FK(q_fb)`，米 / 弧度 |
| `lead_q` | `float64[7]` | `q_cmd − q_fb`（rad） |
| `solve_us` | `float64` | 本拍求解耗时 |

上报频率 10–50Hz。

---

## 时序契约

| 项 | 约定 |
|---|---|
| 输入频率 `f_in` | 默认 30Hz，可配置（15 / 30 / 60） |
| 输出频率 `f_out` | 默认 200Hz，必须整除下游 OTG 频率（推荐 1kHz） |
| `f_out / f_in` | **允许非整数**（30:200 = 3:20）；实现必须处理 6/7 交替 |
| `pose_id` 单调性 | 严格递增；接收侧丢弃 `id <= max_seen` |
| 乱序 | 丢弃，不重排 |
| 跳号 | 视为丢帧，插值器直接跨区间，不报错 |
| 超时 | `pose_id` 停滞超过 `N × T_in`（默认 N=3）→ 各层 HOLD |
| 帧身份 | **只用 `pose_id`**；`stamp_ns` 不参与任何判定 |

### 固定延迟

插值器使用固定延迟 `Δt`：

```
alpha = (t_exec - Δt - t_a) / (t_b - t_a)
```

`Δt` 必须 ≥ 一个输入周期 + 网络抖动峰峰值。默认 `1.05 × T_in`。

- 不钳 `alpha` 到 1（那会退化成零阶保持）
- 只有一帧可用时 HOLD 到第二帧到达

---

## 误差语义

三个量必须分开上报，混用会导致误判：

| 量 | 定义 | 回答 |
|---|---|---|
| `e_model` | `r − FK(q_prev)` | 算法对不对 |
| `e_real` | `r − FK(q_fb)` | 系统到没到 |
| `lead_q` | `q_cmd − q_fb` | 伺服跟不跟 |

关系：`e_real ≈ e_model + J · lead_q`

> **只看 `e_model` 会把"伺服没跟上"误判为"跟踪良好"。** `e_real` 是遥操精度的唯一验收口径。

---

## 状态机

```
        ┌──────────┐  pose_id 超时   ┌────────┐
        │ TRACKING │ ─────────────▶ │  HOLD  │
        └──────────┘                └────────┘
             ▲                          │
             │      新帧到达             │ 连续 HOLD > M 拍
             └──────────────────────────┤
                                        ▼
                                   ┌──────────┐  持续失败   ┌───────┐
                                   │ DEGRADED │ ─────────▶ │ FAULT │
                                   └──────────┘            └───────┘
```

- `HOLD`：输出上一拍 `q_cmd`，`src` 冻结，`solver = hold`
- `DEGRADED`：连续 HOLD 超过阈值（默认 20 拍），仍需继续输出
- `FAULT`：停止输出，由安全层接管

---

## 扩展点

### IK 后端

```cpp
class IKBackend {
public:
  virtual ~IKBackend() = default;

  virtual bool solve(const Pose& T_target,
                     const double q_hint[7],
                     const double q_lower[7],
                     const double q_upper[7],
                     double dt,
                     double q_out[7]) = 0;

  /// 正运动学。e_model / e_real 的必需输入。
  virtual void fk(const double q[7], Pose& T_out) const = 0;

  /// 后端类别，写入 JointStream.solver。
  virtual Solver kind() const { return Solver::kQp; }

  virtual const char* name() const = 0;
};
```

要求：
- 单次调用无堆分配（200Hz 实时路径）
- 失败时返回 false，由 `core` 决定 HOLD
- **不得读取全局状态**，所有前值经参数传入

参考实现可接入 `srs_qp_ik`（proxsuite 速度级 QP，限位硬约束）。

### 传感器打标

```cpp
class SensorTagger {
public:
  virtual void onFrame(const char* sensor_id,
                       int64_t stamp_ns,
                       uint64_t id_a, uint64_t id_b,
                       double alpha) = 0;
};
```

每帧图像/点云到达时由适配层调用，`core` 负责提供当前插值区间。打标结果由使用方自行落盘。

> 若传感器链路存在固定延迟（USB + 编码等），应在 `stamp_ns` 中扣除标定值后再取区间。

---

## 兼容性

- 结构体只允许**尾部新增**字段；删除或改语义必须升主版本
- 所有多字节字段假定小端序
- 序列化格式由适配层决定（ROS msg / protobuf / 裸 struct 均可），但字段集合与语义必须与本文件一致

---

## 与实现无关

本契约**不包含**：

- 任何具体机器人型号、URDF、DH 参数
- 任何中间件、话题名、QoS
- 任何标定流程细节
- 任何具体 IK 算法

这些都在适配层与插件中提供。框架核心只认本文件。
