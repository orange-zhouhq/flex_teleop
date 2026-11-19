// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：flex_qp_ik.hpp
// 包角色：L0 层速度级 QP 逆运动学（仅 Eigen + QpSolver + srs_dh_model，无 ROS）
//
// 与 srs_analytic_ik 的关系：**互补，不是替代**。
//   - 解析解（SRS 闭式）能枚举全部 8 个 GC branch × 臂角，供采样规划器构建图。
//     QP 是局部优化，给不出全局分支枚举。
//   - QP 能表达约束（关节限位硬约束、零空间次任务、未来的相对位姿约束与力任务），
//     解析解表达不了，只能靠解后拒绝 + 爬山补救。
//   ⇒ 规划路径用解析解，伺服 / 约束 / 力控路径用 QP。
//
// 每帧求解的 QP（决策变量只有 q̇，n = 7）：
//
//   min  ½‖J q̇ − ẋ_des‖²_W  +  ½λ‖q̇‖²  +  ½ w_post ‖q̇ − q̇_post‖²
//   s.t. l_box ≤ q̇ ≤ u_box
//
// 展开成标准形式：H = JᵀWJ + λI + w_post·I，g = −JᵀW ẋ_des − w_post·q̇_post。
//
// box 约束是**速度限幅与位置限位屏障的合流**（全部是简单边界，故走 QP 的 box 快路径）：
//
//   l_box_i = max( q̇_min_i ,  −k_lim·(q_i − q_min_i − ε) )
//   u_box_i = min( q̇_max_i ,  +k_lim·(q_max_i − ε − q_i) )
//
// 这一条同时取代了旧解析路径上的两坨启发式：
//   - 关节限位：从"解出来再检查 → 拒绝 → 沿 phi 爬山补救"变成**硬约束**；
//   - 连续性/翻支：输出是 q̇ 且被 box 限幅，**跳变在构造上不可能发生**，
//     不再需要 branch/phi hint 与连续性闸门（branch 是解析参数化的产物，
//     关节空间里不存在这个不连续）。
// =============================================================================

#ifndef FLEX_QP_IK__FLEX_QP_IK_HPP_
#define FLEX_QP_IK__FLEX_QP_IK_HPP_

#include <array>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "srs_dh_model/srs_dh_model.hpp"
#include "flex_qp_ik/arm_angle.hpp"
#include "flex_qp_ik/qp_solver.hpp"

namespace flex_qp_ik
{

using JointArray = std::array<double, srs_dh_model::kArmDoF>;

/// "自动"哨兵：用于 q_pref 等可留空字段（与 srs_analytic_ik 的 phi=NaN 约定一致）。
inline constexpr double kAuto = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kInf = std::numeric_limits<double>::infinity();

/// 按**任务速度**缩放的收敛/故障判据。
///
///   tol_eff = clamp(tol_min + k_v · v_task, tol_min, tol_max)
///
/// ★ 为什么需要：跟随误差本身与速度成正比（`arm.cpp:1927-1933` 记的 e ≈ v/Kp、
///   Kp ≈ 10，0.3 m/s 下正常滞后就有 30 mm）。拿固定阈值判"跟踪失效"在高速下
///   必然误报、静止时又会漏报。
///
/// ★ 缩放变量由**调用方提供**，求解器不自己推断。两个理由：
///   ① `FlexQpIk::solve()` 收到的是静态目标，**根本不知道目标在动多快**；
///   ② 若改用求解器自己的输出 twist 来缩放，会形成反馈陷阱——收敛 ⇒ 输出 twist
///      趋零 ⇒ 容差收紧到下限 ⇒ 判定"不收敛"。搜索器自己的收敛性不能当速度代理。
///   伺服层知道目标轨迹的运动速率，由它调用 effectivePosTolerance() 算好再传入。
///
/// ★ 它只服务**监督语义**（这次算不算成功 / 算不算跟踪失效），不参与控制。
///   不要把它接到"提前退出迭代"上——那会让高速时提前停止解算、跟踪变差。
struct SpeedScaledTolerance
{
  /// 关闭时直接用 min 值，等价于固定判据（默认，保持行为可预期）。
  bool enabled{false};
  double pos_tol_min_m{2e-3};
  double pos_tol_max_m{2e-2};
  /// 每 1 m/s 任务线速度放宽多少米。
  double pos_k_v_s{3e-2};
  double rot_tol_min_rad{2e-3};
  double rot_tol_max_rad{1e-1};
  /// 每 1 rad/s 任务角速度放宽多少弧度。
  double rot_k_v_s{3e-2};
};

/// 由任务线速度算有效位置容差：clamp(min + k_v·v, min, max)。
double effectivePosTolerance(const SpeedScaledTolerance & st, double task_lin_speed);
/// 由任务角速度算有效姿态容差：clamp(min + k_v·ω, min, max)。
double effectiveRotTolerance(const SpeedScaledTolerance & st, double task_ang_speed);

/// 求解模式。**必须显式选择**——两种模式的调参差异很大，选错会静默劣化：
///
/// | | 伺服 kServo | 一次性 kOneShot |
/// |---|---|---|
/// | 调用形态 | 每帧一次，跨帧累加收敛 | 一次调用解到位 |
/// | `max_iterations` | 1 | 200 |
/// | `solve_step_s` | 控制周期（5e-3） | 3e-2 |
/// | 自适应阻尼 | 开（奇异处靠它平滑退化） | 关（纯拖慢，实测近奇异处关掉反而更好） |
/// | 零空间姿态次任务 | 开（主动远离限位） | 关（纯拖慢） |
/// | 每步增益 | 0.5（防过冲） | 0.95（近死拍） |
///
/// 实测（600 个可达位姿、0.1 mm 容差）：伺服默认值拿去跑一次性求解只有
/// **63.5% 成功率、775 μs**；换 kOneShot 是 **100%、35 μs**。
/// 反之把 kOneShot 拿去跑 200 Hz 回路，最坏单帧 **4.6 ms**，逼近 5 ms 的 tick 预算。
enum class FlexQpIkMode
{
  /// 伺服跟踪：每帧一次 QP + 一步积分。**200 Hz 回路 / 遥操作 / 力控用这个。**
  kServo,
  /// 一次性求解到位：给定位姿求关节角。**规划点、离线计算用这个。**
  kOneShot,
};

/// 模式的可读名（日志/诊断）。
const char * toString(FlexQpIkMode mode);

/// 求解选项。默认值面向 200 Hz 伺服回路（单帧小步、连续、贴限位即刹车）。
struct FlexQpIkOptions
{
  // ── 位姿任务 ──────────────────────────────────────────────────────────
  /// **每步消除误差的比例**。内部增益 = `*_gain_per_step / solve_step_s`。
  ///
  /// ★ 为什么用"每步比例"而不是绝对增益 [1/s]：
  ///   每步的关节位移是 `Δq ≈ k·J⁺·e·step`，所以真正决定跟踪行为的是乘积
  ///   `k·step`——它就是每步消除的误差比例。若把 k 写成绝对值，同一个值在
  ///   "伺服 step=5e-3" 与 "一次性 step=3e-2" 下相差 6 倍：**必有一种是错的**。
  ///   （这是实测踩到的：伺服下 k=20、step=5e-3 ⇒ k·step=0.1，比死拍小 10 倍，
  ///    滞后被放大 10 倍才等到速度限幅撑满，表现为"末端明显跟不上目标"。）
  ///
  /// 取 0.5：每步消掉一半误差，1~2 步收敛且不过冲。
  ///   - 取 1.0 = 死拍（一步到位，但对非线性误差会过冲）；
  ///   - 取 > 1.0 会振荡；
  ///   - 远小于 0.5 则伺服下会产生明显稳态滞后。
  double pos_gain_per_step{0.5};
  double rot_gain_per_step{0.5};
  /// 期望 twist 的模长限幅。0 = 不限幅。
  ///
  /// ★ **这两个是"线性化护栏"，不是速度限制**，默认取到远超臂能力。
  ///   真正的速度限制是 `qd_max`（关节空间，可与 TeleopSafety 对齐），关节空间
  ///   的约束天然随构型自适应，而这组任务空间限幅不会。
  ///   实测踩到的坑：早期取 0.30/0.60 时，它比关节预算紧 3 倍，于是**取代 qd_max
  ///   成了事实上的限速**，表现为"末端明显跟不上目标"（A/B 里末端轨迹只有解析
  ///   链的 0.53~0.71 倍）；放宽到护栏量级后恢复到 0.93~1.23 倍。
  ///
  ///   保留它们是为了挡住病态指令（奇异位形附近巨大误差给出的荒谬 twist），
  ///   正常跟踪时不该生效。
  double max_linear_vel{3.0};
  double max_angular_vel{8.0};

  /// 任务空间权重（选择矩阵 S），按 6 个自由度逐一可调，行序 [vx vy vz | wx wy wz]。
  ///
  /// 置 0 = 该自由度不做位置控制（留给力控/自由），这是**混合力位控制**的基础：
  /// 某方向接入力控时把对应权重关掉，位置任务就不会与之争抢。
  /// 一般保持标量统一值；要降权某个自由度（如绕工具轴的旋转）单独调低即可。
  std::array<double, 6> w_task{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // ── 正则化 / 自适应 DLS ───────────────────────────────────────────────
  /// 是否启用**自适应阻尼**（自适应 DLS）。
  ///
  /// 关闭时 λ 恒为 damping_floor（固定 DLS）；开启时按最小奇异值放大：
  ///   σ < σ_th 时  λ = λ_floor + λ_span·(1 − σ_min/σ_th)²，否则 λ = λ_floor
  ///
  /// 实测依据：本臂在肘接近伸直时 σ_min(J) 会掉到 5e-4，而固定 λ=1e-4 在那里
  /// 压不住——q̇ 直接打满 qd_max，表现为关节增速、跟踪变差。
  bool enable_adaptive_damping{true};
  /// 阻尼下限 λ_floor。保证 H 始终正定（数值必需），并给出奇异处之外的基线正则。
  double damping_floor{1e-4};
  /// σ < σ_th 时额外增加的阻尼幅值。
  double damping_span{1e-2};
  /// 触发自适应阻尼的奇异值阈值 σ_th（无量纲，J 的量纲混合，按臂的量级试定）。
  double damping_sigma_threshold{0.05};

  // ── 零空间姿态次任务 ──────────────────────────────────────────────────
  /// 是否启用。它是旧 phi 爬山逻辑的**正确替代**：在零空间里把构型拉离限位。
  ///
  /// 实现上会先把次任务投影到 J 的零空间再纳入代价，故**不会**偏置主任务的
  /// 稳态精度——这一点是能否收敛到容差的前提，不是可选的优化。
  bool enable_posture{true};
  /// 逐关节权重：min ‖w_i·(q̇_i − q̇_post,i)‖²。
  ///
  /// 给易漂进限位的关节单独加大权重（实测该臂是 j6，index 5），零空间就会优先
  /// 把它拉回中间。均匀权重时零空间只按各关节的到位速度自然分配，管不住特定关节。
  JointArray w_posture{0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
  /// 朝 q_pref 的**姿态速度**增益 [1/s]：`q̇_post = k_posture·(q_pref − q)`。
  ///
  /// ★ 与主任务的 pos_gain_per_step **刻意用不同的归一化**，这不是疏漏：
  ///   - 主任务是位置误差反馈，"每步消除比例"才是决定收敛行为的量，故按步长归一；
  ///   - 姿态任务是**速度级**目标，`rad/s 就是 rad/s`——同一个 0.1 rad/s 在
  ///     200 Hz 与 50 Hz 下都是 0.1 rad/s，这才符合物理。若也按步长归一，小步长下
  ///     会算出大 10 倍的速度（实测 37 rad/s），既不物理，又会放大零空间投影的
  ///     泄漏（泄漏量 ∝ 姿态速度）。
  ///
  /// ★ `q̇_post` 会被**钳到 qd_min/qd_max**（见 buildPostureVelocity）。原因同上：
  ///   投影的泄漏发生在 box 约束之前，不先钳住就等于让一个 37 rad/s 的指令去污染
  ///   主任务，而 box 根本来不及管。
  double k_posture{2.0};
  /// 零空间投影 (JJᵀ + λ_ns·I)⁻¹ 的阻尼。取小值保证奇异位形下投影不发散。
  double posture_nullspace_damping{1e-6};
  /// 姿态偏好构型。kAuto = 取各关节限位中心。
  JointArray q_pref{
    kAuto, kAuto, kAuto, kAuto, kAuto, kAuto, kAuto};

  // ── 零空间臂角（psi）次任务 ────────────────────────────────────────────
  /// 臂角软跟随权重。`phi_des` 为 kAuto/NaN 时本项完全不装配（行为与旧版一致）。
  ///
  /// 实现：½ w_phi ‖J_φ N q̇ − φ̇_des‖²，N 为主任务雅可比零空间投影，
  /// **一阶不偏置末端跟踪**（与 posture 同纪律）。
  double w_phi{0.05};
  /// 臂角误差 → 期望臂角速度 [1/s]：φ̇_des = k_phi · wrap(phi_des − φ)。
  /// 与 k_posture 同为速度级增益，不按 solve_step_s 归一。
  double k_phi{2.0};

  // ── 臂角次任务的纪律（★ 以下四道闸门都有实测依据，不要删） ─────────────
  //
  // 背景（实测，38146 个可达构型）：|ψ − ψ_ref| 中位数 1.20 rad、最大 3.14 rad——
  // 把"回到某个固定臂角"当常态目标时，这个次任务是**持续的巨量自运动需求**。
  // 而 φ̇_des = k_phi·wrap(psi_des−φ) 原本**没有任何钳位**（对照 posture 是钳过的），
  // k_phi=2.0 下 Δψ=π 直接给出 6.28 rad/s。实测后果：ψ 目标不可达时末端残差
  // 0.2 mm → **5.2 mm**、归一化限位余量 0.288 → **0.008**（顶住软限位），
  // 而 QP 仍报 solved。四道闸门把语义钉回"防漂移"。

  /// ① φ̇_des 钳位 [rad/s]。与 posture 的钳位纪律对齐。
  double phi_qd_max_rad_s{0.5};
  /// ② 死区 [rad]：|wrap(phi_des − φ)| 超过它则**整个不装配** ψ 次任务。
  ///    语义 = "防漂移"，不是"拉回远处臂角"（要拉回用 posture/q_pref）。
  ///    副作用是好的：目标恒在当前 ψ 附近 ⇒ 位姿早退仍能触发。
  double phi_deadband_rad{0.4};
  /// ③ 主任务门控 [m]：位姿残差大于它时不装配 ψ 次任务，
  ///    防"末端还没到位、次任务先抢 box"（实测中把主任务拖坏的那条路径）。
  double phi_gate_pos_err_m{0.005};
  /// ④ ψ 可观测性门限（见 ArmAngleInfo::conditioning，0 = 奇异）。
  ///    默认 sin(5°)≈0.087：肘离完全伸直 5° 以内就不再信任 ψ。
  double phi_cond_min{0.087};
  /// ψ 的"到位"判据 [rad]，供 solve() 的位姿早退使用（原先硬编码 1e-3）。
  double phi_exit_tol_rad{1e-3};

  // ── ψ 势函数（可选，替代"线性跟踪 + 死区"，2026-10-23 实验）──────────
  /// 臂角目标模式。
  /// - `kPotential`（**默认，2026-10-23 起**）：φ̇_des = clamp(−∂U/∂ψ)，软参考 +
  ///   限位势；**不用死区闸门**（退化/病态闸门仍保留——J_φ 不可信时必须拦）。
  ///   U(ψ)=½w_ref·wrap(ψ−ψ_des)² + ½w_lim·max(0, ψ_min+m−ψ)²
  ///        + ½w_lim·max(0, ψ−ψ_max+m)²
  ///   ★ 动机：kLinear 的死区是**永久失效**——|Δψ|>phi_deadband_rad 时整帧不装配，
  ///   而 ψ 不会自己靠近 ⇒ 目标一旦超死区就永远追不上（实测 500 帧仍差 1.0 rad）。
  /// - `kLinear`（旧行为）：φ̇_des = clamp(k_phi·Δψ)，|Δψ|>phi_deadband_rad 不装配。
  enum class PhiPotentialMode { kLinear = 0, kPotential = 1 };
  PhiPotentialMode phi_potential_mode{PhiPotentialMode::kPotential};
  double phi_potential_w_ref{2.0};      ///< 软参考权重 [1/s]（≈ k_phi）
  double phi_potential_w_limit{20.0};   ///< 限位势权重 [1/s]
  double phi_potential_psi_min{-3.14159265358979323846};
  double phi_potential_psi_max{3.14159265358979323846};
  double phi_potential_margin{0.15};    ///< 限位势作用裕度 [rad]

  // ── 关节限位屏障（速度级 CBF） ────────────────────────────────────────
  /// k_lim [1/s]：越大刹车越早越硬。取 2 表示"进入 ε 带后最多用 2·(距离) rad/s 靠近"。
  double limit_barrier_gain{2.0};
  /// ε [rad]：距限位多远开始刹车。
  double limit_margin_rad{0.02};

  // ── 关节速度限幅 ──────────────────────────────────────────────────────
  /// ★ 应直接取上层真正能下发的速度（如 TeleopSafety::dq_max_per_tick / dt），
  ///   让 IK 与下发能力**同速**——否则 IK 会跑在下发前面，积累领先量后被钳制。
  JointArray qd_max{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  JointArray qd_min{-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0};

  // ── 迭代 ──────────────────────────────────────────────────────────────
  /// 单次 solve() 内的最大 QP 迭代次数。
  ///
  /// 取 100 是为"一次性求解到位"服务：0.1 rad 量级的初始扰动下，前若干步受速度
  /// 限幅支配（线性推进），之后按 (1 − K·step) 几何收敛，实测需 20~80 步。
  /// **伺服请改成 1**（见 solve_step_s 的说明）。
  int max_iterations{100};
  /// ★ `solve()` 内部迭代的**积分步长** [s]，**不是控制周期**。
  ///
  /// 它同时决定两件事：
  ///   - 收敛速度：误差沿任务方向的每步衰减因子约 (1 − K·step)。取 3e-2 时
  ///     K·step = 0.6 ⇒ 每步衰减到 0.4，收敛快且不过冲；
  ///     接近 1 会死拍、超过 1 会振荡，故不宜继续加大；
  ///   - 每步位移上限：被 max_*_vel 与 qd_* 限幅后，每步最多走
  ///     `max_linear_vel · step` 或 `qd_max · step`。
  ///
  /// ★★ **200 Hz 伺服请显式改成本包的非默认配置**：
  ///       max_iterations = 1 ,  solve_step_s = 控制周期(5e-3)
  ///     语义是"每帧一次 QP + 一步积分"，跨帧累加收敛——这才是伺服回路的正确
  ///     形态，也是与解析 IK 做 A/B 对拍时的对标设置。默认值面向"一次性求解
  ///     到位"（规划点、单元测试），一次调用就把误差压进容差。
  double solve_step_s{3e-2};

  // ── 收敛判据（★ 两个角色必须拆开，混用会有害） ─────────────────────────
  /// ① **迭代提前退出**判据。固定，不随速度变化。
  ///
  /// 作用只有一个：已经到位了就别再白解一次 QP。所以它必须取**紧值**。
  /// ★ 不要让它随速度放松：伺服下这会变成"高速时提前停止解算、跟踪变差"，
  ///   与"高速更该跟紧"的直觉正好相反。
  double early_exit_pos_tol_m{2e-3};
  double early_exit_rot_tol_rad{2e-3};

  /// ② `converged` 的汇报判据。**固定值**，由调用方决定。
  ///
  /// 需要按任务速度缩放时，调用方用 effectivePosTolerance() 算好再写进来
  /// （求解器不知道目标在动多快，见 SpeedScaledTolerance 的说明）。
  double converged_pos_tol_m{2e-3};
  double converged_rot_tol_rad{2e-3};

  // ── 求解器 ────────────────────────────────────────────────────────────
  /// QP 后端名。见 availableQpBackends()。
  std::string qp_backend{"proxqp"};

  /// QP **内层**迭代上限。**这是实时界的唯一硬来源。**
  ///
  /// ★ 为什么它重要：外层 `max_iterations` 决定"这一步走多远"，而**这一步要花
  ///   多少计算量**由内层迭代数决定。每次 QP 迭代约 2.1 万条指令（实测），
  ///   所以总指令数 ≈ 73k + 内层迭代数 × 21.5k。
  ///
  ///   取 100 时，理论最坏是 2.22M 条指令——在 800 MHz（本机 scaling_min_freq）
  ///   配 IPC 1.0 的悲观假设下是 **2.78 ms，超过 2 ms 的截止期**。
  ///   取 30 时最坏 718k 条指令 → 同样假设下 **0.90 ms**，留 2.2 倍余量。
  ///
  ///   实测（6 万个不同构型）内层迭代：常态平均 1.7 / 最大 7；困难输入平均 7.1 /
  ///   最大 14。所以 30 对实测最坏仍有 2 倍余量。
  ///
  /// ★ 这是**有意为之的算力换确定性**：截断掉的迭代会让那一帧的 QP 返回
  ///   `kMaxIterations`，`solveVelocity` 据此返回 false，外层走"保持"。
  ///   对伺服而言这是安全的行为（宁可停一帧，也不能超时）。
  int qp_max_iterations{30};

  // ── 预设 ──────────────────────────────────────────────────────────────
  //
  // ★ 上面那一堆默认值**全是伺服口径**（自适应阻尼、零空间姿态次任务、
  //   每步 0.5 增益、与下发同速的 qd_max）。直接拿去做**一次性求解**会严重
  //   拖后腿——实测 600 个可达位姿、0.1 mm 容差下：
  //
  //     forServo()    ：381/600 (63.5%)   775 μs
  //     forOneShot()  ：600/600 (100%)     35 μs      ← 只改了四个开关
  //
  //   差的不是算法，是配置。这两个工厂存在的意义就是不让调用方去猜该动哪几个。

  /// 伺服形态：每帧一次 QP + 一步积分，跨帧累加收敛。**200 Hz 回路用这个。**
  static FlexQpIkOptions forServo(double tick_s)
  {
    FlexQpIkOptions o;
    o.max_iterations = 1;
    o.solve_step_s = tick_s;
    return o;
  }

  /// 一次性求解到位：关掉伺服专属机制、提高增益。
  ///
  /// 关掉的两样都是"用速度换稳定"的选择，对一次性求解是纯负担：
  ///   - `enable_adaptive_damping`：阻尼压 σ_min 处的不稳定，但也拖慢收敛。
  ///     关掉后近奇异处反而更好（实测 σ_min<0.01：98% vs 45%）——因为一次性
  ///     求解的步长本就受限，不需要靠阻尼额外限幅；
  ///   - `enable_posture`：姿态次任务实测拖慢 2 倍以上（每步推 0.06 rad 零空间运动）；
  ///   - 增益 0.5 → 0.95：0.5 是为了伺服防过冲，一次性求解可以近死拍。
  ///
  /// ★ `qd_max` **刻意保持伺服值**（不放宽）。实测把它放开到 ±20 并无收益
  ///   （成功率 99.8% vs 保持 ±2.0 的 100%），却会让这个预设带上"命令速度
  ///   远超硬件"的风险。`qd_max` 是唯一阻止 QP 输出超过关节实际能力的东西，
  ///   不该为了调参方便而放宽。
  static FlexQpIkOptions forOneShot()
  {
    FlexQpIkOptions o;
    o.max_iterations = 200;
    o.solve_step_s = 3e-2;
    o.enable_adaptive_damping = false;
    o.enable_posture = false;
    o.pos_gain_per_step = 0.95;
    o.rot_gain_per_step = 0.95;
    // ★ 收紧容差。结构体默认的 2 mm 是**伺服口径**（伺服单帧到不了那么准，
    //   阈值取紧只会白烧迭代），一次性求解没有这个借口——它有 200 次迭代的预算，
    //   实测能做到 0.03 mm。沿用 2 mm 会让"解到位"在 2 mm 就停下，与预设名不符。
    o.early_exit_pos_tol_m = 1e-4;      // 0.1 mm
    o.early_exit_rot_tol_rad = 5e-4;    // 0.5 mrad
    o.converged_pos_tol_m = 1e-4;
    o.converged_rot_tol_rad = 5e-4;
    return o;   // qd_max 保持默认（伺服值），见上
  }
};

/// 诊断输出（成功失败均填写）。
struct FlexQpIkDiagnostics
{
  QpStatus qp_status{QpStatus::kUnknown};
  int qp_iterations{0};
  double qp_time_us{0.0};
  /// 最终 FK 复核的位姿残差（不是 QP 内部量，而是"解到底把末端送到了哪里"）。
  double pos_err_m{kInf};
  double rot_err_rad{kInf};
  /// 归一化限位余量：0 = 贴限位，0.5 = 量程正中，**<0 = 越限**。
  double worst_limit_margin{std::numeric_limits<double>::quiet_NaN()};
  int iterations_used{0};

  // ── 自适应阻尼的诊断（用于确认它在奇异位形真的介入过） ────────────────
  /// 本次 solveVelocity 实际使用的阻尼 λ。
  double damping_used{0.0};
  /// 本次 solveVelocity 用到的 J 最小奇异值。接近 0 说明处于奇异附近。
  double sigma_min{0.0};

  /// **整次**位姿级 `solve()` 墙钟耗时 [μs]（含 FK/装配/全部 QP 迭代）。
  /// 速度级 `solveVelocity` 单次调用时等于当次 `qp_time_us`。
  double solve_time_us{0.0};

  // ── 实时性诊断 ────────────────────────────────────────────────────────
  /// **本次调用的 QP 是否撞上内层迭代上限**（= 算力不够没算完，不是无解）。
  ///
  /// 撞上限会让该帧走"保持"。伺服下这是**安全**的（宁可停一帧也不超时），
  /// 但它是"控制频率被打断"的直接信号，所以必须能被统计。
  /// 上实机验证 2 ms 实时界时，看这个计数比看耗时更能说明问题。
  bool qp_hit_iteration_cap{false};
  /// 自构造以来累计撞上限的次数与总调用次数。**用比值判断实时性裕量**：
  ///   - 比值为 0 → 迭代上限（默认 30）对当前工况绰绰有余
  ///   - 比值升高 → 工况变难或上限偏紧，需调 `qp_max_iterations` 或降低目标速度
  long qp_hit_iteration_cap_total{0};
  long calls_total{0};

  // ── 臂角次任务的诊断（ABI ≥ 6）─────────────────────────────────────────
  /// 本次求解位形上 ψ 是否处于**几何退化回退**（1 = 返回的是 wrap(θ3)，不是臂角）。
  /// 该回退是**有限值**，只判 isfinite 一定漏。
  int phi_fallback{0};
  /// 本次求解位形上 ψ 的可观测性 ∈[0,1]（0 = 奇异）。见 ArmAngleInfo::conditioning。
  double phi_conditioning{0.0};
  /// `phi_des` 有限但 ψ 次任务**本次未装配**（死区 / 可观测性不足 / 主任务门控）
  /// ——这是"次任务为什么没干活"的唯一信号，判"臂角跟随失效"要看它。
  int phi_gated{0};
};

/// 位姿级求解结果。
///
/// ★ `ok` 与 `converged` 是**两件事**，伺服场景下必须分清：
///   - `ok`        = 本次产出了一个**可用的关节角**。伺服模式下这就是"本帧可以走
///                   这一步"，**不代表已经到达目标**。
///   - `converged` = 已经落到 position/rotation_tolerance 之内（真的到位了）。
///
/// 为什么必须分开：伺服是 `max_iterations = 1` 的形态，单帧位移被 qd_max 限幅，
/// 通常只有零点几毫米，**永远到不了 2 mm 容差**。若把"到位"当成可用性判据，
/// 就会每帧判定失败 → 每帧回退到 q_hint → 机械臂完全冻结。
struct FlexQpIkSolution
{
  /// 是否产出了可用的 q（QP 至少成功走了一步，或已收敛）。
  bool ok{false};
  /// 是否已收敛到容差内。
  bool converged{false};
  /// `ok = false` 时等于输入的 q_hint，即**保持**语义（宁可停住也不跳变）。
  /// `ok = true` 但 `converged = false` 时是本帧走到的中间位（已受限位与步长约束）。
  JointArray q{};
  FlexQpIkDiagnostics diag{};
};

/// 速度级 QP 逆运动学求解器。纯 C++，无 ROS 依赖，可在 200 Hz 回路内同步调用。
///
/// 使用约定：
///   - 实例**串行复用**（内部 QP 热启动），不要跨线程共享；
///   - 关节限位从 geometry.lower/upper 读，构造时必须已填好（否则抛异常）。
class FlexQpIk
{
public:
  /// **按模式构造（推荐）**。两种模式的调参差异很大，故要求调用方显式选择，
  /// 而不是依赖 `FlexQpIkOptions{}` 的默认值——实测显示靠默认值猜模式会静默劣化。
  ///
  /// @param geometry   S-R-S DH 几何，**必须已填 lower/upper**（见 srs_kinematics_config）
  /// @param mode       求解模式；见 FlexQpIkMode 的对比表
  /// @param tick_s     仅 kServo 用：控制周期（200 Hz → 5e-3）
  /// @throws std::invalid_argument 限位未填或 lower >= upper
  FlexQpIk(srs_dh_model::SrsGeometry geometry, FlexQpIkMode mode, double tick_s = 5e-3);

  /// 按完整选项构造。需要细调参数时才用；否则请用上面的模式构造函数。
  /// @throws std::invalid_argument 限位未填或 lower >= upper
  explicit FlexQpIk(
    srs_dh_model::SrsGeometry geometry, FlexQpIkOptions options = FlexQpIkOptions{});

  const srs_dh_model::SrsDhModel & model() const { return model_; }
  const FlexQpIkOptions & options() const { return options_; }
  const JointArray & lower() const { return model_.geometry().lower; }
  const JointArray & upper() const { return model_.geometry().upper; }

  /// 速度级核心：给定当前 q 与**模型系**期望 twist，解 QP 得 q̇。
  ///
  /// 这是双臂协调层与力控要用的那一层——相对位姿约束、笛卡尔阻抗任务都可以
  /// 直接写进同一个 QP，而不必像现在的解析路径那样先扰动目标位姿再求解。
  ///
  /// @param q            当前关节角（MoveIt 约定）
  /// @param twist_des    期望末端 twist，[v(0:3); ω(3:6)]，模型系，ω 为空间角速度
  /// @param qd_out       解出的关节速度 [rad/s]
  /// @param diag         可选诊断输出
  /// @return false = QP 未解出可用解（不可行 / 迭代耗尽 / 数值失败），
  ///         此时 qd_out 未被写入，调用方应"保持"。
  bool solveVelocity(
    const JointArray & q,
    const Eigen::Matrix<double, 6, 1> & twist_des,
    JointArray & qd_out,
    FlexQpIkDiagnostics * diag = nullptr,
    double phi_des = kAuto);

  /// 位姿级便捷入口：内部迭代 max_iterations 次，每次 = 位姿误差 → twist → QP → 积分。
  ///
  /// 与 IkSolver::solve 同形（目标位姿在模型系，通常为 tc_link 下的 link7），
  /// 便于直接插入现有调用点做 A/B 对比；两者结果类型不同（本包是 L0，不依赖
  /// real_time_planner 的 types.hpp），由上层适配。
  ///
  /// @param phi_des 臂角目标（rad）。**kAuto/NaN = 不管**（默认）；传入有限值则
  ///                零空间软跟随（见 options.w_phi / k_phi）。非硬约束。
  FlexQpIkSolution solve(
    const Eigen::Isometry3d & T_target_model,
    const JointArray & q_hint,
    double phi_des = kAuto);

  /// 直观入口：位置 + RPY（ZYX，与 ROS `setRPY` 一致）→ 关节角。
  /// 内部转到 Isometry 再走 `solve()`；结果里的 `diag.solve_time_us` 含转换开销。
  FlexQpIkSolution solveXyzRpy(
    const Eigen::Vector3d & xyz,
    const Eigen::Vector3d & rpy,
    const JointArray & q_hint,
    double phi_des = kAuto);

  /// 直观入口：位置 + 四元数 xyzw（ROS 约定）→ 关节角。
  FlexQpIkSolution solveXyzQuat(
    const Eigen::Vector3d & xyz,
    const Eigen::Quaterniond & quat_xyzw,
    const JointArray & q_hint,
    double phi_des = kAuto);

  /// 当前构型的臂角 ψ（rad），与解析 IK 的 psi 同定义。
  ///
  /// ★ 退化时返回的是**回退量 wrap(θ3)**（有限值，不是 NaN）。新代码请用
  ///   armAngleInfo()，否则无法区分"真臂角"与"回退量"。
  double armAngle(const JointArray & q) const;

  /// 当前构型的臂角 ψ + 状态 + 可观测性。
  ArmAngleInfo armAngleInfo(const JointArray & q) const;

  /// **运行期**更新四个容差，**不重建 QP、不丢热启动**。
  ///
  /// ★ 与"重建选项"的区别：重建（C ABI 的 set_options）会丢弃 QP 热启动状态，
  ///   库注释明确禁止每帧调；本方法只改 4 个标量——已核实它们只在 `solve()` 的
  ///   早退/汇报两处被读取，不参与 QP 装配、不参与预处理器（proxsuite 的
  ///   warmed_up 路径与之无关）。这正是把 effectivePosTolerance() 接进伺服
  ///   回路的缺失环节：没有它，按速度缩放容差只能靠重建。
  ///
  /// ★ 不变量 `converged_* >= early_exit_*`：违反会让 solve() 出现**永久保持**——
  ///   早退命中时 stepped=false，若此时又判 converged=false，则 ok=false
  ///   ⇒ 调用方丢帧 ⇒ q 不变 ⇒ 下一帧同样 ⇒ 永久停在容差带里。
  ///
  /// @return false = 违反该不变量（**未做任何修改**，调用方应修正后重试）。
  bool setTolerances(double early_pos, double early_rot, double conv_pos, double conv_rot);

  /// **运行期**更新姿态偏好构型与权重（同样不重建 QP、不丢热启动）。
  ///
  /// ★ 只写 `options_.q_pref` 是**静默 no-op**：posture 次任务用的是构造期解析好的
  ///   `q_pref_resolved_`（kAuto→限位中心，见 resolvePostureTarget）。本方法按
  ///   正确顺序刷新两者，避免"设了没生效、而且看不出来"。
  void setPosturePref(
    const JointArray & q_pref, double k_posture, const JointArray & w_posture);

  /// 归一化限位余量：min_i( min(q_i−lo_i, up_i−q_i) / (up_i−lo_i) )。
  /// 定义与 IkSolver::limitMargin 一致，便于 A/B 直接比较。
  double limitMargin(const JointArray & q) const;

private:
  /// 组装 box 约束：速度限幅 ∩ 位置限位屏障。两侧挤压时取中点以保证可行性。
  void buildLimitBox(
    const JointArray & q, Eigen::VectorXd & l_box, Eigen::VectorXd & u_box) const;

  /// 由当前位姿与目标位姿算期望 twist（模型系，含增益与限幅）。
  Eigen::Matrix<double, 6, 1> computeTwist(
    const Eigen::Isometry3d & T_target, const Eigen::Isometry3d & T_cur) const;

  /// 姿态偏好构型：q_pref 中 kAuto 的项填成限位中心。
  void resolvePostureTarget();

  srs_dh_model::SrsDhModel model_;
  FlexQpIkOptions options_;
  JointArray q_pref_resolved_{};
  /// 求解器临时量：复用缓冲，避免 200 Hz 下每帧堆分配。
  Eigen::MatrixXd H_{};
  Eigen::VectorXd g_{};
  Eigen::VectorXd l_box_{};
  Eigen::VectorXd u_box_{};
  std::unique_ptr<QpSolver> qp_;

  /// 实时性累计计数（供 TotalGuard 递增）。
  long calls_total_{0};
  long hit_cap_total_{0};

  friend struct TotalGuard;
};

}  // namespace flex_qp_ik

#endif  // FLEX_QP_IK__FLEX_QP_IK_HPP_
