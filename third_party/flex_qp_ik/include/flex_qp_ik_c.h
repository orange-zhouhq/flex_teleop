/* SPDX-License-Identifier: BSD-3-Clause
 * ============================================================================
 * flex_qp_ik_c.h —— S-R-S 7 轴机械臂速度级 QP 逆运动学，**稳定的 C 接口**
 *
 * 为什么要有这一层（而不是直接交付 C++ API 的 .so）：
 *
 *   1) Eigen 对齐 ABI 陷阱。C++ API 的 `FlexQpIk(SrsGeometry)` 按值传递结构体，
 *      而 SrsGeometry 含 Eigen::Isometry3d。Eigen 对定长类型的静态对齐取决于
 *      编译期宏 EIGEN_MAX_STATIC_ALIGN_BYTES（SSE 下 16、AVX 下 32），
 *      于是**同一个结构体在不同编译选项下大小不同**（实测 656 vs 672 字节）。
 *      调用方按自己的布局压栈、库按自己的布局读字段 ⇒ 静默读到错误几何参数，
 *      解出错误关节角，全程不报错。
 *   2) C++ ABI 耦合：C++ API 里有 std::string / std::array / std::unique_ptr，
 *      要求调用方匹配 libstdc++ 版本与 _GLIBCXX_USE_CXX11_ABI。
 *
 * 本接口只用 `double` / `int` / 不透明指针：
 *   - 裸 double 数组与 4x4 行主序矩阵，**不存在对齐契约**，与编译选项无关；
 *   - 无任何 C++ 标准库类型跨边界；
 *   - 因此可被任意编译器、任意语言（C / C++ / Python ctypes / Rust …）调用。
 *
 * 用法：
 *     flex_qp_ik_geometry_t g = ...;            // 填 DH 参数与关节限位
 *     flex_qp_ik_options_t  o = flex_qp_ik_default_options();
 *     o.solve_step_s = 0.005;  o.max_iterations = 1;      // 200 Hz 伺服形态
 *     o.qd_max[0..6] = 2.0;    o.qd_min[0..6] = -2.0;
 *
 *     flex_qp_ik_status st;
 *     flex_qp_ik_t* ik = flex_qp_ik_create(&g, &o, sizeof(o), &st);
 *     if (!ik) { fprintf(stderr, "%s\n", flex_qp_ik_last_error()); return 1; }
 *
 *     double T_target[16];                     // 4x4 行主序，末端目标位姿
 *     flex_qp_ik_solution_t sol;
 *     flex_qp_ik_solve_pose(ik, T_target, q_hint, &sol);
 *     // sol.ok / sol.converged / sol.q[7]
 *
 *     flex_qp_ik_destroy(ik);
 *
 * 线程安全：句柄**不可跨线程共享**（内部持有 QP 热启动状态）。不同句柄可并行。
 * 版本兼容：本文件与 .so 必须**成对**交付。用 flex_qp_ik_abi_version() 在运行时
 *          校验；create() 也会校验调用方传的 options_size。
 * ============================================================================
 */

#ifndef FLEX_QP_IK_C_H_
#define FLEX_QP_IK_C_H_

#include <stddef.h>

/*
 * 符号导出宏。
 *
 * 库默认以 `-fvisibility=hidden` 构建（只暴露 C ABI，防止外部误链 C++ 内部符号）。
 * 因此每个对外函数都必须显式标注本宏，否则会被隐藏、调用方链接时报未定义符号。
 */
#if defined(_WIN32) || defined(__CYGWIN__)
#  ifdef FLEX_QP_IK_C_EXPORTS
#    define FLEX_QP_IK_API __declspec(dllexport)
#  else
#    define FLEX_QP_IK_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define FLEX_QP_IK_API __attribute__((visibility("default")))
#else
#  define FLEX_QP_IK_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ── 版本 ────────────────────────────────────────────────────────────────── */

/**
 * ABI 版本。
 *
 * ★ **新增符号也会递增**，不只是破坏性改动。理由：本库的头文件与 .so 必须
 *   成对交付，而"新头文件 + 旧 .so"的失败形式是**符号未定义**——链接期或
 *   首次调用时才报，且信息是 `undefined symbol: flex_qp_ik_xxx`，很难定位到
 *   版本不匹配。递增版本号让这种错配可以在 create() 时立刻被发现。
 *
 * 版本历史：
 *   1 —— 初版：IK（速度级/位姿级）、FK、限位余量、模式与预设
 *   2 —— 新增雅可比、运动学专用句柄、几何/位姿转换 helper
 *   3 —— （中间修订）
 *   4 —— 臂角零空间次任务：options 末尾追加 w_phi/k_phi；
 *        新增 arm_angle / solve_*_phi*
 *   5 —— 诊断追加 solve_time_us；solution 追加耗时/残差；
 *        新增 xyzrpy / xyzquat 直观求解入口
 *   6 —— options 末尾追加 q_pref[7] 与臂角次任务纪律四参数（phi_qd_max_rad_s /
 *        phi_deadband_rad / phi_gate_pos_err_m / phi_cond_min）与 phi_exit_tol_rad；
 *        诊断追加 phi_fallback / phi_conditioning / phi_gated；
 *        新增 set_tolerances（不重建、不丢热启动）、set_posture_pref、arm_angle_info、
 *        get_options（现场核对部署配置）
 *   7 —— （2026-10-23）options 末尾追加 phi_potential_mode 与势函数参数
 *        （w_ref / w_limit / psi_min / psi_max / margin）；**默认模式切到势函数**，
 *        phi_potential_mode=0 可回退线性+死区。
 *        动机：死区是"永久失效"——|Δψ| 超死区即整帧不装配，目标一旦超死区
 *        就永远追不上（见 phi_potential_mode 注释）。
 */
#define FLEX_QP_IK_ABI_VERSION 7

/** 关节自由度，固定 7（S-R-S 冗余臂）。 */
#define FLEX_QP_IK_DOF 7

/** 库版本字符串，形如 "0.1.0"。 */
FLEX_QP_IK_API const char * flex_qp_ik_version(void);

/** ABI 版本号，与头文件里的 FLEX_QP_IK_ABI_VERSION 比对。 */
FLEX_QP_IK_API int flex_qp_ik_abi_version(void);

/* ── 状态码 ──────────────────────────────────────────────────────────────── */

typedef enum
{
  FLEX_QP_IK_OK = 0,
  /** 入参非法（空指针、非有限值、关节数不符、options_size 不匹配等）。 */
  FLEX_QP_IK_ERR_INVALID_ARGUMENT = 1,
  /** 本次求解未产出可用解。**不是错误**，调用方应"保持上一帧"，不要停机。 */
  FLEX_QP_IK_ERR_NOT_SOLVED = 2,
  FLEX_QP_IK_ERR_INTERNAL = 3,
} flex_qp_ik_status;

/**
 * 最近一次错误的可读说明（**线程局部**，无需释放，下次出错时被覆盖）。
 *
 * 主要用于 create() 失败——那时还没有句柄可查，只能靠这个。
 */
FLEX_QP_IK_API const char * flex_qp_ik_last_error(void);

/* ── 不透明句柄 ──────────────────────────────────────────────────────────── */

/**
 * 求解器句柄。定义对调用方不可见（不完整类型），只能通过指针使用。
 *
 * ★ 一个句柄**不可跨线程共享**：内部持有 QP 热启动状态。不同句柄可并行。
 */
typedef struct flex_qp_ik_s flex_qp_ik_t;

/* ── 几何 ────────────────────────────────────────────────────────────────── */

/**
 * 机械臂几何：标准 DH 参数 + 关节限位 + q↔theta 映射 + 首尾固定变换。
 *
 * 与 C++ 侧 `srs_dh_model::SrsGeometry` 逐字段对应，但**全部是裸 double**。
 * 矩阵一律 **4x4 行主序**（16 个 double，行优先展开）。
 */
typedef struct
{
  /** 连杆长度 a_i（m）。 */
  double a[FLEX_QP_IK_DOF];
  /** 连杆扭转 alpha_i（rad）。 */
  double alpha[FLEX_QP_IK_DOF];
  /** 连杆偏距 d_i（m）。 */
  double d[FLEX_QP_IK_DOF];
  /** 关节下限（rad，MoveIt/URDF 关节角约定）。**必须填**，lower < upper。 */
  double lower[FLEX_QP_IK_DOF];
  /** 关节上限（rad）。 */
  double upper[FLEX_QP_IK_DOF];
  /** q↔theta 符号：theta_i = q_sign_i · q_i + q_offset_i，通常 ±1。 */
  double q_sign[FLEX_QP_IK_DOF];
  /** q↔theta 偏置（rad）。 */
  double q_offset[FLEX_QP_IK_DOF];
  /** tc_link → DH 基座的固定刚体变换，4x4 行主序。 */
  double T_tc_link_dh_base[16];
  /** DH 法兰 → link7 的固定刚体变换，4x4 行主序。 */
  double T_dh_flange_link7[16];
} flex_qp_ik_geometry_t;

/** 把几何置为"单位阵 + 单位映射 + 零 DH"的初值，便于逐字段填写。 */
FLEX_QP_IK_API void flex_qp_ik_geometry_identity(flex_qp_ik_geometry_t * geometry);

/** 几何里可设置的两个固定变换。 */
typedef enum
{
  /** tc_link → DH 基座 的变换。 */
  FLEX_QP_IK_XFORM_TC_LINK_TO_DH_BASE = 0,
  /** DH 法兰 → link7 的变换。 */
  FLEX_QP_IK_XFORM_DH_FLANGE_TO_LINK7 = 1,
} flex_qp_ik_xform;

/**
 * 用**位置 + RPY** 设置几何里的固定变换，省去调用方自己搓旋转矩阵。
 *
 * RPY 约定：**ZYX 内旋**，即 `R = Rz(yaw)·Ry(pitch)·Rx(roll)`（与 ROS 的
 * `tf2::Quaternion::setRPY` 一致）。
 *
 * 之前调用方得自己把 rpy 展开成 4x4 再逐元素填 16 个 double——这是几何配置里
 * 最容易写错的一步（顺序、内旋外旋、行主序），所以做成库函数。
 *
 * @param geometry 目标几何
 * @param which    要设置哪一个变换
 * @param xyz      平移 [3]（m）
 * @param rpy      姿态 [3]（rad，ZYX）
 */
FLEX_QP_IK_API void flex_qp_ik_geometry_set_transform(
  flex_qp_ik_geometry_t * geometry, flex_qp_ik_xform which,
  const double xyz[3], const double rpy[3]);

/**
 * 由**位置 + RPY** 构造 4x4 位姿（行主序 [16]），供 IK 的目标位姿用。
 * RPY 同样是 ZYX：`R = Rz(yaw)·Ry(pitch)·Rx(roll)`。
 */
FLEX_QP_IK_API void flex_qp_ik_pose_from_xyzrpy(
  const double xyz[3], const double rpy[3], double T_out[16]);

/**
 * 由**位置 + 四元数** 构造 4x4 位姿（行主序 [16]）。
 *
 * 四元数顺序为 **xyzw**（ROS `geometry_msgs/Quaternion` 的约定）。
 * 内部会归一化；模长接近 0 时退化为单位旋转（不产生 NaN）。
 */
FLEX_QP_IK_API void flex_qp_ik_pose_from_xyzquat(
  const double xyz[3], const double quat_xyzw[4], double T_out[16]);

/* ── 求解选项 ────────────────────────────────────────────────────────────── */

/**
 * 求解选项。字段与 C++ 侧 `flex_qp_ik::FlexQpIkOptions` 一一对应。
 *
 * **务必先用 flex_qp_ik_default_options() 取一份默认值再改**，不要手工清零：
 * 默认值经过实测调参（见各字段注释里的实测依据）。
 *
 * 版本演进约定：字段只允许**追加在末尾**，且必须同步递增 ABI 版本；
 * create() 用 options_size 检测不匹配，宁可拒绝也不静默误读。
 */
typedef struct
{
  /* ── 位姿任务 ── */
  /** 每步消除误差的比例。内部增益 = 该值 / solve_step_s。取 0.5：1~2 步收敛不过冲。 */
  double pos_gain_per_step;
  double rot_gain_per_step;
  /** 期望 twist 的模长限幅。**是线性化护栏，不是速度限制**，默认远超臂能力。 */
  double max_linear_vel;
  double max_angular_vel;
  /** 任务空间选择矩阵（6 维权，顺序 vx vy vz wx wy wz）。置 0 = 该自由度不做位置控制。 */
  double w_task[6];

  /* ── 正则化 / 自适应 DLS ── */
  /** 是否启用自适应阻尼（奇异位形下 λ 随 σ_min 放大）。 */
  int enable_adaptive_damping;
  double damping_floor;
  double damping_span;
  double damping_sigma_threshold;

  /* ── 零空间姿态次任务 ── */
  /** 是否启用。替代解析路径的 phi 余量爬山。 */
  int enable_posture;
  /** 逐关节权重：给易漂进限位的关节单独加大。 */
  double w_posture[FLEX_QP_IK_DOF];
  /** 朝 q_pref 的姿态速度增益 [1/s]。 */
  double k_posture;
  double posture_nullspace_damping;

  /* ── 关节限位屏障 ── */
  double limit_barrier_gain;
  double limit_margin_rad;

  /* ── 关节速度限幅 ── */
  /** 应取上层真正能下发的速度（如 TeleopSafety 每帧增量 / 控制周期）。 */
  double qd_max[FLEX_QP_IK_DOF];
  double qd_min[FLEX_QP_IK_DOF];

  /* ── 迭代 ── */
  /** 单次求解的最大 QP 迭代数。**伺服请设 1**。 */
  int max_iterations;
  /** 迭代步长 [s]，**不是控制周期**。伺服请设控制周期（200 Hz → 5e-3）。 */
  double solve_step_s;
  /** 迭代提前退出判据（固定紧值，不随速度变化）。 */
  double early_exit_pos_tol_m;
  double early_exit_rot_tol_rad;
  /** "已收敛"的汇报判据（固定值；需要按任务速度缩放时用下面的 helper 算好再填）。 */
  double converged_pos_tol_m;
  double converged_rot_tol_rad;

  /* ── 零空间臂角（psi）次任务（ABI ≥ 4）── */
  /**
   * 臂角软跟随权重。求解时传入有限 phi_des 才生效；NaN/不传则完全不管。
   * 实现为零空间次任务，一阶不偏置末端跟踪。
   */
  double w_phi;
  /** 臂角误差 → 期望臂角速度增益 [1/s]。 */
  double k_phi;

  /* ── 臂角次任务的纪律（ABI ≥ 6；★ 四道闸门都有实测依据，见 .so 源码注释）── */

  /** ① φ̇_des 钳位 [rad/s]。原先完全没钳（对照 posture 是钳过的），
   *  k_phi=2.0 下 Δψ=π 会给出 6.28 rad/s；不钳会吃光 box 预算并污染主任务
   *  （实测：末端残差 0.2 mm → 5.2 mm、限位余量 0.288 → 0.008，QP 仍报 solved）。 */
  double phi_qd_max_rad_s;
  /** ② 死区 [rad]：|wrap(phi_des−φ)| 超过它则整个不装配 ψ 次任务。
   *  语义是"防漂移"而非"拉回远处臂角"（要拉回请用 posture/q_pref）。 */
  double phi_deadband_rad;
  /** ③ 主任务门控 [m]：位姿残差大于它时不装配 ψ 次任务。 */
  double phi_gate_pos_err_m;
  /** ④ ψ 可观测性门限（0 = 奇异）。默认 sin(5°)；详见 arm_angle_info。 */
  double phi_cond_min;
  /** ψ 的"到位"判据 [rad]，供位姿级 solve() 的提前退出使用（原先硬编码 1e-3）。 */
  double phi_exit_tol_rad;

  /* ── 姿态偏好构型（ABI ≥ 6）── */
  /**
   * 零空间姿态次任务的目标关节构型 `q_pref[i]`。
   * **NaN = 该项取对应关节的限位中心**（与 srs_analytic_ik 的 kAuto 约定一致）。
   *
   * ★ 这一项此前只在 C++ 接口里存在，C ABI 下恒为限位中心（改不了）——
   *   于是"把构型拉回出发位形"这类需求在 C 侧做不到。典型用法：把种子
   *   （首帧关节反馈）写进来，零空间就有了**有记忆的**恢复力，
   *   而不是只能靠"以当前 ψ 为目标"那种无记忆的防漂移任务。
   */
  double q_pref[FLEX_QP_IK_DOF];

  /* ── ψ 势函数模式（ABI ≥ 7，2026-10-23）──
   *
   * 动机：`phi_deadband_rad` 的语义是"防漂移"，但实际效果是**永久失效**——
   * |Δψ| 超过死区时整帧不装配 ψ 次任务，而 ψ 不会自己靠近，于是目标一旦超死区
   * 就永远追不上（实测 500 帧仍差 1.0 rad）。势函数用连续软参考 + 限位势替代
   * 死区闸门，退化/病态闸门仍保留。
   *
   * `phi_potential_mode`：1 = 势函数（默认，与 C++ 侧一致）；0 = 线性 + 死区。
   */
  int phi_potential_mode;
  /** 软参考权重 [1/s]（≈ k_phi 口径）。 */
  double phi_potential_w_ref;
  /** 限位势权重 [1/s]。 */
  double phi_potential_w_limit;
  /** ψ 可行区间下/上界 [rad]（默认 ±π 即不启用限位势，现场按机构收紧）。 */
  double phi_potential_psi_min;
  double phi_potential_psi_max;
  /** 限位势作用裕度 [rad]。 */
  double phi_potential_margin;
} flex_qp_ik_options_t;

/** 取一份实测调过参的默认选项。**等同于 flex_qp_ik_options_for_servo(5e-3)**。 */
FLEX_QP_IK_API flex_qp_ik_options_t flex_qp_ik_default_options(void);

/**
 * 伺服形态预设：每帧一次 QP + 一步积分，跨帧累加收敛。**200 Hz 回路用这个。**
 * @param tick_s 控制周期（200 Hz → 5e-3）
 */
FLEX_QP_IK_API flex_qp_ik_options_t flex_qp_ik_options_for_servo(double tick_s);

/**
 * 一次性求解到位预设：关掉伺服专属机制并提高增益。
 *
 * 实测（600 个可达位姿、0.1 mm 容差）：
 *   伺服默认值 → 成功率 63.5%、单次 775 μs
 *   本预设     → 成功率 100%、 单次 35 μs
 *
 * ★ 两个预设**不可混用**：把本预设拿去跑 200 Hz 回路会让单帧耗时远超 tick 预算
 *   （200 次迭代 × 35 μs ≈ 7 ms > 5 ms）。
 */
FLEX_QP_IK_API flex_qp_ik_options_t flex_qp_ik_options_for_oneshot(void);

/**
 * 按任务速度算有效位置容差：clamp(min + k_v·|v|, min, max)。
 *
 * 用于"跟踪是否失效"的判据——跟随误差与速度成正比，固定阈值在高速下必然误报。
 * ★ 缩放变量由**调用方**提供：求解器收到的是静态目标，不知道目标在动多快。
 */
FLEX_QP_IK_API double flex_qp_ik_effective_pos_tolerance(
  int enabled, double tol_min_m, double tol_max_m, double k_v_s, double task_lin_speed);
FLEX_QP_IK_API double flex_qp_ik_effective_rot_tolerance(
  int enabled, double tol_min_rad, double tol_max_rad, double k_v_s, double task_ang_speed);

/* ── 诊断与结果 ──────────────────────────────────────────────────────────── */

/** 一次求解的诊断（成功失败均填写）。 */
typedef struct
{
  /** QP 求解器状态，见 flex_qp_ik_qp_status()。 */
  int qp_status;
  int qp_iterations;
  double qp_time_us;
  /** 最终返回的 q 经 FK 复核的位姿残差。 */
  double pos_err_m;
  double rot_err_rad;
  /** 归一化限位余量：0 = 贴限位，0.5 = 量程正中，**<0 = 越限**。 */
  double worst_limit_margin;
  int iterations_used;
  /** 本次实际使用的阻尼 λ 与 J 的最小奇异值（确认自适应阻尼是否介入过）。 */
  double damping_used;
  double sigma_min;

  /* ── 实时性诊断（PoseTrackingStatus 上报这两个是最有价值的） ── */
  /**
   * **本次调用的 QP 是否撞上内层迭代上限**（= 算力不够没算完，不是无解）。
   *
   * 撞上限会让该帧走"保持"。伺服下这是**安全**的（宁可停一帧也不超时），
   * 但它是"控制频率被打断"的直接信号——上实机验证 2 ms 实时界时，
   * **看这个计数比看耗时更能说明问题**（耗时会被调度干扰污染，见实时界报告）。
   */
  int qp_hit_iteration_cap;
  /** 自构造以来累计撞上限次数 / 总调用次数。用比值判断实时性裕量。 */
  long qp_hit_iteration_cap_total;
  long calls_total;

  /* ── 臂角次任务的诊断（ABI ≥ 6）── */
  /**
   * 本次求解位形上 ψ 是否处于**几何退化回退**（1 = 返回的是 wrap(θ3)，不是臂角）。
   * ★ 该回退是**有限值**，只判 isfinite 一定漏。
   */
  int phi_fallback;
  /** 本次求解位形上 ψ 的可观测性 ∈[0,1]（0 = 奇异）。见 flex_qp_ik_arm_angle_info。 */
  double phi_conditioning;
  /**
   * `phi_des` 有限但 ψ 次任务**本次未装配**（死区 / 可观测性不足 / 主任务门控）。
   * ★ 这是"次任务为什么没干活"的唯一信号——见它才能区分"跟踪正常"与"任务被闸门关了"。
   */
  int phi_gated;

  /**
   * **整次**位姿级求解墙钟耗时 [μs]（含 FK / 装配 / 全部 QP 迭代）。
   * 速度级单次调用时约等于 qp_time_us。ABI ≥ 5。
   */
  double solve_time_us;
} flex_qp_ik_diagnostics_t;

/** 本 .so 里 flex_qp_ik_diagnostics_t 的字节数——供调用方断言版本匹配。 */
FLEX_QP_IK_API size_t flex_qp_ik_sizeof_diagnostics(void);

/** 位姿级求解结果。 */
typedef struct
{
  /** 是否产出可用解。伺服下单帧通常为 1（走了一步），但不代表已到位。 */
  int ok;
  /** 是否已收敛到容差内。与 ok 是**两件事**，伺服下极少为 1。 */
  int converged;
  /** ok=0 时等于输入的 q_hint（保持语义）。 */
  double q[FLEX_QP_IK_DOF];
  /* ── ABI ≥ 5：简单路径也能读到的关键反馈（不必调 *_ex）── */
  /** 整次求解墙钟耗时 [μs]。 */
  double solve_time_us;
  /** 最终返回 q 的位姿残差（与 diagnostics 同值）。 */
  double pos_err_m;
  double rot_err_rad;
} flex_qp_ik_solution_t;

/** QP 状态的整数编码 → 可读字符串（仅诊断用，永不返回 NULL）。 */
FLEX_QP_IK_API const char * flex_qp_ik_qp_status_str(int qp_status);

/* ── 求解模式 ────────────────────────────────────────────────────────────── */

/**
 * 求解模式。**必须显式选择**——两种模式的调参差异很大，靠默认值猜会静默劣化。
 *
 * | | SERVO | ONESHOT |
 * |---|---|---|
 * | 调用形态 | 每帧一次，跨帧累加收敛 | 一次调用解到位 |
 * | 自适应阻尼 / 零空间姿态次任务 | 开 | 关 |
 * | 每步增益 | 0.5（防过冲） | 0.95（近死拍） |
 *
 * 实测（600 个可达位姿、0.1 mm 容差）：伺服默认值拿去跑一次性求解只有
 * 63.5% 成功率、775 μs；换 ONESHOT 是 100%、35 μs。反之把 ONESHOT 拿去跑
 * 200 Hz 回路，最坏单帧 4.6 ms，逼近 5 ms 的 tick 预算。
 */
typedef enum
{
  /** 伺服跟踪：200 Hz 回路 / 遥操作 / 力控用这个。 */
  FLEX_QP_IK_MODE_SERVO = 0,
  /** 一次性求解到位：规划点、离线计算用这个。 */
  FLEX_QP_IK_MODE_ONESHOT = 1,
} flex_qp_ik_mode;

/** 模式的可读名（日志/诊断用；永不返回 NULL）。 */
FLEX_QP_IK_API const char * flex_qp_ik_mode_str(flex_qp_ik_mode mode);

/* ── 生命周期 ────────────────────────────────────────────────────────────── */

/**
 * 构造求解器。
 *
 * @param geometry      几何与关节限位。**lower/upper 必须已填**且 lower < upper。
 * @param options       选项；可为 NULL 表示用默认值。
 * @param options_size  传 sizeof(flex_qp_ik_options_t)。与库内定义不符时拒绝构造，
 *                      避免头文件/.so 版本错配时静默误读结构体。
 * @param status        可选输出状态；失败时填错误码，成功时填 FLEX_QP_IK_OK。
 * @return 句柄；失败返回 NULL，用 flex_qp_ik_last_error() 取原因。
 */
FLEX_QP_IK_API flex_qp_ik_t * flex_qp_ik_create(
  const flex_qp_ik_geometry_t * geometry,
  const flex_qp_ik_options_t * options,
  size_t options_size,
  flex_qp_ik_status * status);

/**
 * 只构造**运动学**（FK + 雅可比），**不分配 QP 求解器**。
 *
 * 用途：只需要 FK / 雅可比的调用方——力控算 `JᵀF`、标定校验、奇异分析、
 * 可视化——不必提供关节限位，也不付 QP 工作区的构造开销。
 *
 * ★ 与 flex_qp_ik_create* 的区别（不是性能，是契约）：
 *   - `geometry.lower/upper` **可以不填**（运动学用不到限位，create 会校验它）；
 *   - 用本句柄调 `flex_qp_ik_solve_*` / `flex_qp_ik_limit_margin` 会返回
 *     `FLEX_QP_IK_ERR_INVALID_ARGUMENT`——需要 IK 请用 create_with_mode。
 *
 * 性能说明：**FK / 雅可比的耗时与句柄类型无关**。实测（Release）走 IK 句柄的
 * FK 是 0.157 μs、雅可比 0.252 μs，与直接调 C++ 的 0.154 / 0.205 μs 同量级
 * （差异只是函数调用边界与一次 6×7 展开拷贝）。它们不触碰任何 QP 状态，
 * 对 5 ms 的 200 Hz tick 而言可忽略。
 *
 * @param geometry 几何。lower/upper 可以留空。
 * @return 句柄；失败返回 NULL，用 flex_qp_ik_last_error() 取原因
 */
FLEX_QP_IK_API flex_qp_ik_t * flex_qp_ik_create_kinematics_only(
  const flex_qp_ik_geometry_t * geometry, flex_qp_ik_status * status);

/**
 * **按模式构造（推荐）**。等价于用 flex_qp_ik_options_for_servo/for_oneshot 取
 * 预设再调 flex_qp_ik_create，但不可能选错或忘记选。
 *
 * @param mode   求解模式；见 flex_qp_ik_mode 的对比表
 * @param tick_s 仅 FLEX_QP_IK_MODE_SERVO 用：控制周期（200 Hz → 5e-3）
 * @return 句柄；失败返回 NULL，用 flex_qp_ik_last_error() 取原因
 */
FLEX_QP_IK_API flex_qp_ik_t * flex_qp_ik_create_with_mode(
  const flex_qp_ik_geometry_t * geometry,
  flex_qp_ik_mode mode,
  double tick_s,
  flex_qp_ik_status * status);

/** 销毁句柄。传 NULL 安全。 */
FLEX_QP_IK_API void flex_qp_ik_destroy(flex_qp_ik_t * handle);

/**
 * 更新选项（例如运行中切换伺服/一次性求解形态）。
 *
 * 会重建内部 QP 求解器（丢弃热启动状态），故不要每帧调用。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_set_options(
  flex_qp_ik_t * handle, const flex_qp_ik_options_t * options, size_t options_size);

/* ── 求解 ────────────────────────────────────────────────────────────────── */

/**
 * 速度级求解：给定当前关节角与期望末端 twist，解出关节速度。
 *
 * 这是双臂协调与力控要用的那一层——相对位姿约束、笛卡尔阻抗任务都可直接写进
 * 同一个 QP，而不必先扰动目标位姿再求解。
 *
 * @param q          当前关节角（rad，[7]）
 * @param twist_des  期望 twist [6]：前 3 线速度（m/s）、后 3 角速度（rad/s）；
 *                   **模型系（tc_link）下的空间角速度**，即与
 *                   `log(R_tgt·R_curᵀ)` 同系。
 * @param qd_out     输出关节速度（rad/s，[7]）。返回非 OK 时**不写**。
 * @param diagnostics 可选诊断输出，可为 NULL。
 * @return FLEX_QP_IK_OK 表示 qd_out 有效；其余状态调用方应"保持"。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_velocity(
  flex_qp_ik_t * handle,
  const double q[FLEX_QP_IK_DOF],
  const double twist_des[6],
  double qd_out[FLEX_QP_IK_DOF],
  flex_qp_ik_diagnostics_t * diagnostics);

/**
 * 位姿级求解：目标位姿 → 关节角。
 *
 * 内部迭代 options.max_iterations 次，每次 = 位姿误差 → twist → QP → 积分。
 * - **伺服**：max_iterations=1、solve_step_s=控制周期，跨帧累加收敛；
 * - **一次性到位**：用默认值（100 次 × 3e-2）。
 *
 * @param T_target  目标位姿（模型系/tc_link 下 link7），**4x4 行主序 [16]**。
 * @param q_hint    关节初值 [7]（通常为上一帧解或当前反馈）。
 * @param solution  输出。
 * @return OK 表示 solution 已填；NOT_SOLVED 时 solution->q 等于 q_hint（保持）。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_pose(
  flex_qp_ik_t * handle,
  const double T_target[16],
  const double q_hint[FLEX_QP_IK_DOF],
  flex_qp_ik_solution_t * solution);

/** 位姿级求解 + 诊断（solve_pose 的完整版）。 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_pose_ex(
  flex_qp_ik_t * handle,
  const double T_target[16],
  const double q_hint[FLEX_QP_IK_DOF],
  flex_qp_ik_solution_t * solution,
  flex_qp_ik_diagnostics_t * diagnostics);

/**
 * 带臂角目标的位姿级求解。
 *
 * @param phi_des 臂角目标（rad）。**NaN = 不管**（与 solve_pose 等价）；
 *                有限值 → 零空间软跟随（见 options.w_phi / k_phi）。非硬约束。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_pose_phi(
  flex_qp_ik_t * handle,
  const double T_target[16],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution);

/** 带臂角目标的位姿级求解 + 诊断。 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_pose_phi_ex(
  flex_qp_ik_t * handle,
  const double T_target[16],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution,
  flex_qp_ik_diagnostics_t * diagnostics);

/**
 * **直观入口**：位置 + RPY（ZYX，与 ROS 一致）→ 关节角。
 * 不必自己搓 4x4。phi_des 传 NaN 表示不管臂角。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_xyzrpy(
  flex_qp_ik_t * handle,
  const double xyz[3],
  const double rpy[3],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution);

/** 同上 + 完整诊断。 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_xyzrpy_ex(
  flex_qp_ik_t * handle,
  const double xyz[3],
  const double rpy[3],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution,
  flex_qp_ik_diagnostics_t * diagnostics);

/**
 * **直观入口**：位置 + 四元数 xyzw（ROS 约定）→ 关节角。
 * phi_des 传 NaN 表示不管臂角。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_xyzquat(
  flex_qp_ik_t * handle,
  const double xyz[3],
  const double quat_xyzw[4],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution);

FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_xyzquat_ex(
  flex_qp_ik_t * handle,
  const double xyz[3],
  const double quat_xyzw[4],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution,
  flex_qp_ik_diagnostics_t * diagnostics);

/**
 * 带臂角目标的速度级求解。phi_des=NaN 时与 flex_qp_ik_solve_velocity 等价。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_solve_velocity_phi(
  flex_qp_ik_t * handle,
  const double q[FLEX_QP_IK_DOF],
  const double twist_des[6],
  double phi_des,
  double qd_out[FLEX_QP_IK_DOF],
  flex_qp_ik_diagnostics_t * diagnostics);

/* ── 工具 ────────────────────────────────────────────────────────────────── */

/**
 * 当前构型的臂角 ψ（rad）。与解析 IK（srs_analytic_ik）的 psi 同定义。
 * 运动学专用句柄也可用。失败返回 NaN。
 *
 * ★ 几何退化（肩-肘-腕共线）时**不返回 NaN**，而是回退成 `wrap(θ3)`——
 *   一个与 ψ 完全不同的有限值。需要区分"真臂角"与"回退量"请用下面的 info 版。
 */
FLEX_QP_IK_API double flex_qp_ik_arm_angle(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF]);

/**
 * 臂角 + 状态 + 可观测性（推荐入口）。
 *
 * @param psi_out           臂角 [rad]（status=1 时是回退量 wrap(θ3)）。可为 NULL。
 * @param status_out        0 = 正常；1 = 几何退化回退。可为 NULL。
 * @param conditioning_out  ψ 可观测性 ∈[0,1]（两平面法向夹角 sin），0 = 奇异。可为 NULL。
 * @return OK；句柄非法或 q 非有限时返回错误且不写输出。
 *
 * ★ 用法（伺服回路）：把本函数当"ψ 次任务的可用性门控"——
 *   status!=0 或 conditioning 低于阈值时，不要把 psi 目标传给求解器。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_arm_angle_info(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF],
  double * psi_out, int * status_out, double * conditioning_out);

/**
 * **运行期**更新四个容差，**不重建 QP、不丢热启动**（ABI ≥ 6）。
 *
 * ★ 与 set_options 的区别：set_options 会重建内部 QP 求解器并**丢弃热启动状态**
 *   （其注释明确禁止每帧调用）。本函数只改 4 个标量——它们只在位姿级 solve()
 *   的早退/汇报两处被读取，不参与 QP 装配、不参与预处理器。因此可以每帧调用：
 *   这正是把 flex_qp_ik_effective_pos_tolerance() 接进 200 Hz 伺服回路的通道。
 *
 * ★ 不变量 `converged_* >= early_exit_*`（**必须遵守**）：违反会让位姿级 solve()
 *   出现**永久保持**——早退命中时 stepped=false，若此时又判 converged=false，
 *   则 ok=false ⇒ 调用方丢帧 ⇒ q 不变 ⇒ 下一帧同样 ⇒ 永久停在容差带里。
 *
 * @return OK；参数非正/非有限或违反不变量时返回 FLEX_QP_IK_ERR_INVALID_ARGUMENT
 *         且**不做任何修改**（调用方修正后重试）。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_set_tolerances(
  flex_qp_ik_t * handle,
  double early_exit_pos_tol_m, double early_exit_rot_tol_rad,
  double converged_pos_tol_m, double converged_rot_tol_rad);

/**
 * 读回句柄当前生效的选项（ABI ≥ 6）。
 *
 * 用途：现场核对"部署的配置到底是什么"——尤其是经 set_tolerances /
 * set_posture_pref 运行期改过的值。返回的是副本，改它不影响句柄。
 *
 * @param options      输出缓冲，必须非空。
 * @param options_size 必须 == sizeof(flex_qp_ik_options_t)。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_get_options(
  flex_qp_ik_t * handle, flex_qp_ik_options_t * options, size_t options_size);

/**
 * **运行期**更新姿态偏好构型与权重（ABI ≥ 6；同样不重建 QP、不丢热启动）。
 *
 * @param q_pref     目标构型 [7]；某分量传 NaN 表示该关节取限位中心。必须非 NULL。
 * @param k_posture  朝 q_pref 的速度增益 [1/s]；<= 0 表示不改这一项。
 * @param w_posture  逐关节权重 [7]；可传 NULL 表示不改。
 *
 * ★ 为什么需要专门的入口：姿态次任务用的是**构造期解析好的**限位中心/构型，
 *   只写 options.q_pref 再 set_options 既贵（丢热启动）又容易忘。
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_set_posture_pref(
  flex_qp_ik_t * handle, const double q_pref[FLEX_QP_IK_DOF],
  double k_posture, const double w_posture[FLEX_QP_IK_DOF]);

/** 正运动学：关节角 → 模型系末端位姿（4x4 行主序 [16]）。 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_forward_kinematics(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF], double T_out[16]);

/**
 * 几何雅可比：6×7，**模型系**（tc_link），与 flex_qp_ik_forward_kinematics 同参考系。
 *
 * 行序 [v(0:3); ω(3:6)]，列 i 对应 ∂/∂q_i，满足 `[v; ω] = J · q̇`：
 *   - v：**link7 原点**的线速度（m/s，模型系）；
 *   - ω：link7 的**空间角速度**（rad/s，模型系）。
 *
 * ★ 角速度约定（对接时最易踩的坑）：ω 是空间（fixed-frame）角速度，与
 *   `log(R_tgt · R_curᵀ)` 同系。若姿态误差改用体坐标系形式
 *   `log(R_curᵀ · R_tgt)`，需先左乘 `R_curᵀ` 才能与这里的 ω 相加。
 *
 * 参考点已取在 link7 原点，故 T_dh_flange_link7 的平移偏置已正确计入。
 *
 * @param q   关节角（rad，[7]）
 * @param J   输出，**行主序 6×7 = 42 个 double**
 * @return FLEX_QP_IK_OK 表示 J 已填
 */
FLEX_QP_IK_API flex_qp_ik_status flex_qp_ik_jacobian(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF], double J[42]);

/**
 * 归一化限位余量：min_i( min(q_i−lo_i, up_i−q_i) / (up_i−lo_i) )。
 * 0 = 贴限位，0.5 = 量程正中，<0 = 越限。
 */
FLEX_QP_IK_API double flex_qp_ik_limit_margin(flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF]);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* FLEX_QP_IK_C_H_ */
