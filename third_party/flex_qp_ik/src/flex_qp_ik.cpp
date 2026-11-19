// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：flex_qp_ik.cpp
// 包角色：速度级 QP 逆运动学的装配与积分逻辑
//
// 每帧的装配只有三条：
//   ① H = JᵀWJ + (λ + w_post)·I ，g = −JᵀW·ẋ_des − w_post·q̇_post
//   ② box = 速度限幅 ∩ 位置限位屏障（两侧挤压时取中点）
//   ③ 解出 q̇ 后按 q ← clamp(q + q̇·dt, q_min, q_max) 积分
// =============================================================================

#include "flex_qp_ik/flex_qp_ik.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "flex_qp_ik/arm_angle.hpp"
#include "flex_qp_ik/proxqp_solver.hpp"
#include "flex_qp_ik/rotation_utils.hpp"

#include <chrono>

namespace flex_qp_ik
{
namespace
{

using Jacobian = Eigen::Matrix<double, 6, srs_dh_model::kArmDoF>;

/// 关节限位是否可用。SrsGeometry 允许 lower = upper = 0 表示"尚未从 URDF 写入"，
/// 这种状态下 QP 无法建 box、也无法做积分钳制，必须显式拒绝而不是静默出错。
bool limitsUsable(const srs_dh_model::SrsGeometry & g)
{
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    if (!std::isfinite(g.lower[i]) || !std::isfinite(g.upper[i])) {
      return false;
    }
    if (g.lower[i] >= g.upper[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

const char * toString(FlexQpIkMode mode)
{
  switch (mode) {
    case FlexQpIkMode::kServo:
      return "servo";
    case FlexQpIkMode::kOneShot:
      return "one_shot";
    default:
      return "unknown";
  }
}

FlexQpIk::FlexQpIk(srs_dh_model::SrsGeometry geometry, FlexQpIkMode mode, double tick_s)
: FlexQpIk(
    std::move(geometry),
    mode == FlexQpIkMode::kServo ? FlexQpIkOptions::forServo(tick_s)
                                : FlexQpIkOptions::forOneShot())
{
}

FlexQpIk::FlexQpIk(srs_dh_model::SrsGeometry geometry, FlexQpIkOptions options)
: model_(std::move(geometry)), options_(std::move(options))
{
  if (!limitsUsable(model_.geometry())) {
    throw std::invalid_argument(
            "FlexQpIk: geometry.lower/upper 未填或存在 lower >= upper；"
            "QP IK 需要真实关节限位才能建立硬约束。");
  }

  resolvePostureTarget();

  const int n = static_cast<int>(srs_dh_model::kArmDoF);
  H_.resize(n, n);
  g_.resize(n);
  l_box_.resize(n);
  u_box_.resize(n);

  qp_ = makeQpSolver(options_.qp_backend);
  // ★ 把内层迭代上限传下去——它是实时界的唯一硬来源（见选项注释）。
  if (auto * prox = dynamic_cast<ProxQpSolver *>(qp_.get())) {
    ProxQpSolver::Settings st = prox->settings();
    st.max_iterations = std::max(1, options_.qp_max_iterations);
    prox->setSettings(st);
  }
  // 本包全部约束都是简单边界（速度限幅 + 限位屏障），无等式、无通用不等式，
  // 正好走 ProxQP 的 box 快路径。未来双臂闭链约束会在这里加 m_eq。
  qp_->init(n, /*m_eq=*/0, /*m_in=*/0);
}

void FlexQpIk::resolvePostureTarget()
{
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    const double pref = options_.q_pref[i];
    q_pref_resolved_[i] = std::isnan(pref) ?
      0.5 * (model_.geometry().lower[i] + model_.geometry().upper[i]) :
      pref;
  }
}

double FlexQpIk::limitMargin(const JointArray & q) const
{
  const auto & lo = model_.geometry().lower;
  const auto & up = model_.geometry().upper;
  double worst = kInf;
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    const double span = up[i] - lo[i];
    const double margin = std::min(q[i] - lo[i], up[i] - q[i]) / span;
    worst = std::min(worst, margin);
  }
  return worst;
}

double FlexQpIk::armAngle(const JointArray & q) const
{
  return computeArmAngleMoveItQ(model_, q);
}

ArmAngleInfo FlexQpIk::armAngleInfo(const JointArray & q) const
{
  return computeArmAngleMoveItQInfo(model_, q);
}

bool FlexQpIk::setTolerances(
  double early_pos, double early_rot, double conv_pos, double conv_rot)
{
  // 不变量 `converged >= early_exit`：违反会让 solve() 永久保持——
  // 早退命中时 stepped=false，若同时判 converged=false 则 ok=false
  // ⇒ 调用方丢帧 ⇒ q 不变 ⇒ 下一帧同样 ⇒ 永久停在容差带里。
  // 用"取反"写法把 NaN 一并挡掉（NaN 的比较恒假）。
  if (!(early_pos > 0.0) || !(early_rot > 0.0) ||
    !(conv_pos > 0.0) || !(conv_rot > 0.0) ||
    !(conv_pos >= early_pos) || !(conv_rot >= early_rot))
  {
    return false;
  }
  options_.early_exit_pos_tol_m = early_pos;
  options_.early_exit_rot_tol_rad = early_rot;
  options_.converged_pos_tol_m = conv_pos;
  options_.converged_rot_tol_rad = conv_rot;
  return true;
}

void FlexQpIk::setPosturePref(
  const JointArray & q_pref, double k_posture, const JointArray & w_posture)
{
  options_.q_pref = q_pref;
  options_.k_posture = k_posture;
  options_.w_posture = w_posture;
  // ★ 必须刷新：posture 次任务读的是 q_pref_resolved_（构造期解析），
  //   只写 options_.q_pref 是静默 no-op。
  resolvePostureTarget();
}

void FlexQpIk::buildLimitBox(
  const JointArray & q, Eigen::VectorXd & l_box, Eigen::VectorXd & u_box) const
{
  const double k = options_.limit_barrier_gain;
  const double eps = options_.limit_margin_rad;
  const auto & lo = model_.geometry().lower;
  const auto & up = model_.geometry().upper;

  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    // 速度限幅是基础上下界。
    double lower = options_.qd_min[i];
    double upper = options_.qd_max[i];

    // 位置限位屏障：把"离限位还有多远"折算成"本帧允许朝该方向走多快"。
    // 关节已越过 (q_min + ε) 时，下界变成正值 → 强制朝外走（主动退让）。
    lower = std::max(lower, -k * (q[i] - lo[i] - eps));
    upper = std::min(upper, k * (up[i] - eps - q[i]));

    // 两侧屏障可能互相矛盾（关节被挤在宽度 < 2ε 的区间内，或配置本身
    // qd_min > qd_max）。此时取中点：QP 需要可行域，这是"最不坏"的折中。
    if (lower > upper) {
      const double mid = 0.5 * (lower + upper);
      lower = mid;
      upper = mid;
    }
    l_box[i] = lower;
    u_box[i] = upper;
  }
}

Eigen::Matrix<double, 6, 1> FlexQpIk::computeTwist(
  const Eigen::Isometry3d & T_target, const Eigen::Isometry3d & T_cur) const
{
  Eigen::Matrix<double, 6, 1> twist;

  // 增益由"每步消除比例 / 步长"导出，保证 `k·step = *_gain_per_step` 恒成立——
  // 改 solve_step_s 不会静默改变跟踪行为（见选项注释里实测踩到的那个坑）。
  const double step = std::max(options_.solve_step_s, 1e-9);
  const double k_p = options_.pos_gain_per_step / step;
  const double k_r = options_.rot_gain_per_step / step;

  const Eigen::Vector3d e_p = T_target.translation() - T_cur.translation();
  twist.head<3>() = clampVectorNorm(k_p * e_p, options_.max_linear_vel);

  // ★ 空间角速度约定：log(R_tgt · R_curᵀ)，与 jacobianMoveItQ 的 ω 同系。
  //   换成体坐标系形式 log(R_curᵀ · R_tgt) 不会报错，只会让收敛行为变怪。
  const Eigen::Matrix3d R_err = T_target.linear() * T_cur.linear().transpose();
  twist.tail<3>() = clampVectorNorm(
    k_r * rotationVector(R_err), options_.max_angular_vel);

  return twist;
}

bool FlexQpIk::solveVelocity(
  const JointArray & q, const Eigen::Matrix<double, 6, 1> & twist_des,
  JointArray & qd_out, FlexQpIkDiagnostics * diag, double phi_des)
{
  // 累计计数在返回前统一填，保证从任何失败路径出去都算过。
  struct TotalGuard
  {
    FlexQpIk * self;
    FlexQpIkDiagnostics * diag;
    bool hit{false};
    ~TotalGuard()
    {
      ++self->calls_total_;
      if (hit) {
        ++self->hit_cap_total_;
      }
      if (diag != nullptr) {
        diag->qp_hit_iteration_cap_total = self->hit_cap_total_;
        diag->calls_total = self->calls_total_;
      }
    }
  } guard{this, diag, false};

  const Jacobian J = model_.jacobianMoveItQ(q);

  // ── ⓪ 一次算好 JJᵀ：自适应阻尼与零空间投影都要用 ───────────────────────
  const Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();

  // ── ① 自适应阻尼（自适应 DLS） ─────────────────────────────────────────
  // σ_min 取 JJᵀ 最小特征值的平方根（J 是 6×7，JJᵀ 6×6，比直接对 J 做 SVD 便宜）。
  double lambda = options_.damping_floor;
  if (options_.enable_adaptive_damping) {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(JJt);
    const double sigma_min = std::sqrt(std::max(0.0, es.eigenvalues().minCoeff()));
    const double th = options_.damping_sigma_threshold;
    if (th > 0.0 && sigma_min < th) {
      const double deficit = 1.0 - sigma_min / th;   // ∈ (0, 1]
      lambda += options_.damping_span * deficit * deficit;
    }
    if (diag != nullptr) {
      diag->sigma_min = sigma_min;
    }
  }
  if (diag != nullptr) {
    diag->damping_used = lambda;
  }

  // ── ② 装配 H 与 g ──────────────────────────────────────────────────────
  // H = JᵀWJ + λI + diag(w_post)，g = −JᵀW·ẋ_des − diag(w_post)·q̇_post
  //
  // W 是对角阵（选择矩阵 S），故 (JᵀWJ)_{ij} = Σ_k J_{ki}·w_k·J_{kj} —— **行和列
  // 都要按同一权缩放**。做法是先对 J 按行加权得到 WJ，再算 Jᵀ(WJ)；只缩一边会漏权。
  Jacobian WJ = J;
  for (int r = 0; r < 6; ++r) {
    WJ.row(r) *= options_.w_task[static_cast<std::size_t>(r)];
  }

  H_.noalias() = J.transpose() * WJ;
  H_.diagonal().array() += lambda;

  Eigen::Matrix<double, 6, 1> wtwist;
  for (int r = 0; r < 6; ++r) {
    wtwist[r] = options_.w_task[static_cast<std::size_t>(r)] * twist_des[r];
  }
  g_.noalias() = -(J.transpose() * wtwist);

  // ── ③ 零空间姿态次任务 ─────────────────────────────────────────────────
  // ★ 必须先投影到 J 的零空间再进 g，否则它会与主任务在**同一个加权和**里对抗：
  //   误差趋近 0 时主任务项 JᵀW·K·e → 0，而姿态项不随之消失，两者平衡处留下
  //   一个偏置的稳态误差（实测约 6 mm，直接把"收敛到 2 mm 容差"变成不可能）。
  //   投影后 J·(N·q̇_post) = 0，次任务对主任务一阶无干扰。
  //
  //   实现上用 N·v = v − Jᵀ(JJᵀ + λ_ns·I)⁻¹(J·v) 的形式，避免真的构造 7×7 的 N。
  if (options_.enable_posture) {
    // ★ 姿态速度是**速度级**目标，不按步长归一化（见 k_posture 的说明），
    //   但必须先钳到关节速度限幅——零空间投影的泄漏发生在 box 之前。
    Eigen::VectorXd qd_post(srs_dh_model::kArmDoF);
    bool any_posture = false;
    for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
      const double raw = options_.k_posture * (q_pref_resolved_[i] - q[i]);
      qd_post[static_cast<Eigen::Index>(i)] =
        std::clamp(raw, options_.qd_min[i], options_.qd_max[i]);
      any_posture = any_posture || options_.w_posture[i] > 0.0;
    }

    if (any_posture) {
      Eigen::Matrix<double, 6, 6> JJt_ns = JJt;
      JJt_ns.diagonal().array() += options_.posture_nullspace_damping;
      const Eigen::VectorXd qd_post_proj =
        qd_post - J.transpose() * JJt_ns.ldlt().solve(J * qd_post);

      for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
        const Eigen::Index idx = static_cast<Eigen::Index>(i);
        H_(idx, idx) += options_.w_posture[i];
        g_[idx] -= options_.w_posture[i] * qd_post_proj[idx];
      }
    }
  }

  // ── ③b 零空间臂角次任务（可选）────────────────────────────────────────
  // phi_des 为 NaN/非有限 → 完全跳过，装配与旧版逐字节等价路径一致。
  // 有限值 → ½ w_phi ‖J_φ N q̇ − φ̇_des‖²，N 用与 posture 相同的 (JJᵀ+λ)⁻¹ 投影。
  //
  // ★ 四道闸门（实测依据见 options 里 phi_qd_max_rad_s 等字段的注释）：
  //   退化（status）/ 病态（conditioning）/ 超死区 → 不装配；φ̇_des → 钳位。
  //   主任务门控由 solve() 传 kAuto 实现（速度级调用方自理）。
  if (std::isfinite(phi_des) && options_.w_phi > 0.0) {
    const ArmAngleInfo phi_info = computeArmAngleMoveItQInfo(model_, q);
    if (diag != nullptr) {
      diag->phi_fallback =
        (phi_info.status == ArmAngleStatus::kFallbackDegenerate) ? 1 : 0;
      diag->phi_conditioning = phi_info.conditioning;
    }
    const double dphi = wrapAngleRad(phi_des - phi_info.psi);
    const bool potential_mode =
      options_.phi_potential_mode == FlexQpIkOptions::PhiPotentialMode::kPotential;
    // 势函数模式只保留"退化/病态"闸门：J_φ 不可信时必须拦；
    // 死区闸门由连续的软参考 + 限位势取代。
    const bool usable = potential_mode ?
      (phi_info.status == ArmAngleStatus::kOk &&
      phi_info.conditioning >= options_.phi_cond_min) :
      (phi_info.status == ArmAngleStatus::kOk &&
      phi_info.conditioning >= options_.phi_cond_min &&
      std::fabs(dphi) <= options_.phi_deadband_rad);
    if (!usable) {
      // "次任务没干活"的显式信号：调用方判"臂角跟随失效"必须看它，
      // 否则退化/门控会被误读成"跟踪正常"。
      if (diag != nullptr) {
        diag->phi_gated = 1;
      }
    } else {
      const Eigen::Matrix<double, 1, srs_dh_model::kArmDoF> J_phi =
        armAngleJacobianMoveItQ(model_, q);
      // ★ 钳位：φ̇_des 不经过 qd 预算的显式钳位（对照 posture 是钳过的），
      //   不钳时大 Δψ 会把 box 预算吃光并污染主任务（实测 5.2 mm 残差 / 顶限位）。
      double phi_dot_des;
      if (potential_mode) {
        // U(ψ) = ½w_ref·wrap(ψ−ψ_des)² + 限位势；ψ̇_des = −∂U/∂ψ
        //   ∂U/∂ψ = −w_ref·dphi（dphi = wrap(ψ_des−ψ)）
        //   限位：ψ 低于 lo 时 ∂U/∂ψ 更负（推 ψ 增大），高于 hi 时更正。
        double grad = -options_.phi_potential_w_ref * dphi;
        const double lo =
          options_.phi_potential_psi_min + options_.phi_potential_margin;
        const double hi =
          options_.phi_potential_psi_max - options_.phi_potential_margin;
        if (phi_info.psi < lo) {
          grad -= options_.phi_potential_w_limit * (lo - phi_info.psi);
        }
        if (phi_info.psi > hi) {
          grad += options_.phi_potential_w_limit * (phi_info.psi - hi);
        }
        phi_dot_des = std::clamp(
          -grad, -options_.phi_qd_max_rad_s, options_.phi_qd_max_rad_s);
      } else {
        phi_dot_des = std::clamp(
          options_.k_phi * dphi,
          -options_.phi_qd_max_rad_s, options_.phi_qd_max_rad_s);
      }

      Eigen::Matrix<double, 6, 6> JJt_ns = JJt;
      JJt_ns.diagonal().array() += options_.posture_nullspace_damping;
      // J_φ,nᵀ = Nᵀ J_φᵀ = N J_φᵀ（N 对称）
      const Eigen::VectorXd Jphi_t = J_phi.transpose();
      const Eigen::VectorXd Jphi_n_t =
        Jphi_t - J.transpose() * JJt_ns.ldlt().solve(J * Jphi_t);

      H_.noalias() += options_.w_phi * (Jphi_n_t * Jphi_n_t.transpose());
      g_.noalias() -= options_.w_phi * Jphi_n_t * phi_dot_des;
    }
  }

  // ── ② box 约束 ─────────────────────────────────────────────────────────
  buildLimitBox(q, l_box_, u_box_);

  // ── ③ 求解 ─────────────────────────────────────────────────────────────
  QpProblem problem;
  problem.H = H_;
  problem.g = g_;
  problem.l_box = l_box_;
  problem.u_box = u_box_;
  // A/C/l/u 留空（0 尺寸）：本包当前只有 box 约束。

  const QpResult result = qp_->solve(problem);
  if (diag != nullptr) {
    diag->qp_status = result.status;
    diag->qp_iterations += result.iterations;
    diag->qp_time_us += result.setup_time_us + result.solve_time_us;
    diag->qp_hit_iteration_cap =
      diag->qp_hit_iteration_cap || result.hit_iteration_cap;
  }
  guard.hit = result.hit_iteration_cap;

  if (result.status != QpStatus::kSolved) {
    // 不写 qd_out：调用方据 false 走"保持"。**绝不返回半可信的解**——
    // 控制回路上宁可停住也不跳变。
    return false;
  }

  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    qd_out[i] = result.x[static_cast<Eigen::Index>(i)];
  }
  if (diag != nullptr && diag->solve_time_us <= 0.0) {
    // 速度级单次调用：整次耗时 ≈ QP 耗时（无外层迭代）。
    diag->solve_time_us = diag->qp_time_us;
  }
  return true;
}

FlexQpIkSolution FlexQpIk::solve(
  const Eigen::Isometry3d & T_target_model, const JointArray & q_hint,
  double phi_des)
{
  const auto t0 = std::chrono::steady_clock::now();
  FlexQpIkSolution out;
  out.q = q_hint;  // 失败时保持
  // 跨迭代累计：每次 solveVelocity 会累加 qp_time / qp_iterations。
  out.diag.qp_time_us = 0.0;
  out.diag.qp_iterations = 0;
  out.diag.qp_hit_iteration_cap = false;

  const auto & lo = model_.geometry().lower;
  const auto & up = model_.geometry().upper;

  // 迭代用的工作副本。起点若越限（理论上不应发生），先钳进限位再迭代——
  // 否则屏障约束会自相矛盾。返回时若一步都没走成，仍返回**未改动的** q_hint，
  // 因为"保持"的语义是原样保持调用方给的值。
  JointArray q = q_hint;
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    q[i] = std::clamp(q[i], lo[i], up[i]);
  }
  JointArray qd{};
  bool stepped = false;

  for (int it = 0; it < std::max(1, options_.max_iterations); ++it) {
    const Eigen::Isometry3d T_cur = model_.fkMoveItQ(q);
    const Eigen::Vector3d e_p = T_target_model.translation() - T_cur.translation();
    const Eigen::Matrix3d R_err =
      T_target_model.linear() * T_cur.linear().transpose();
    const double pos_err = e_p.norm();
    const double rot_err = rotationVector(R_err).norm();

    // 提前退出用**固定紧判据**（early_exit_*），不随速度放松——否则伺服下会
    // 变成"高速时提前停止解算、跟踪变差"。是否算"已收敛"由循环外的判据单独决定。
    //
    // ★ 若调用方传入了有限 phi_des，末端到位后仍可能要在零空间调臂角——
    //   此时不能早退，否则"固定末端、只改臂形"完全失效。
    const bool pose_reached =
      pos_err <= options_.early_exit_pos_tol_m &&
      rot_err <= options_.early_exit_rot_tol_rad;
    if (pose_reached) {
      if (!std::isfinite(phi_des)) {
        out.q = q;
        break;
      }
      const double phi_err =
        std::fabs(wrapAngleRad(phi_des - computeArmAngleMoveItQ(model_, q)));
      // 臂角软任务：误差进到容差再停（阈值见 options_.phi_exit_tol_rad）；
      // 更紧没有意义（次任务权重有限）。
      if (phi_err <= options_.phi_exit_tol_rad) {
        out.q = q;
        break;
      }
    }

    const Eigen::Matrix<double, 6, 1> twist = computeTwist(T_target_model, T_cur);

    // ★ 主任务门控：末端残差还大时不装配 ψ 次任务——防"末端未到位、次任务先抢
    //   box"（实测中把主任务拖坏的那条路径）。以 kAuto 传入 = "本步不管臂角"。
    const bool phi_gated_by_task =
      std::isfinite(phi_des) && pos_err > options_.phi_gate_pos_err_m;
    if (phi_gated_by_task) {
      out.diag.phi_gated = 1;
    }
    const double phi_for_step = phi_gated_by_task ? kAuto : phi_des;

    if (!solveVelocity(q, twist, qd, &out.diag, phi_for_step)) {
      // QP 未解出可用解：停止迭代，保留**已经走出来的**部分进度（若一步未走则
      // 走下面的"保持"分支）。已走的每一步都过 box 约束，故中间态始终安全。
      break;
    }

    for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
      q[i] = std::clamp(q[i] + qd[i] * options_.solve_step_s, lo[i], up[i]);
    }
    ++out.diag.iterations_used;
    stepped = true;
    out.q = q;
  }

  // ① 先算"到位没有"。用**固定**判据（converged_*）：它服务监督/故障语义、
  //    不参与控制。需要按任务速度缩放时，由调用方算好后写进这两个字段。
  {
    const Eigen::Isometry3d T_final = model_.fkMoveItQ(out.q);
    const double pos_err =
      (T_target_model.translation() - T_final.translation()).norm();
    const double rot_err = rotationVector(
      T_target_model.linear() * T_final.linear().transpose()).norm();
    out.converged = pos_err <= options_.converged_pos_tol_m &&
      rot_err <= options_.converged_rot_tol_rad;
  }

  // ② 再算"能不能用"。★ 判据是"有没有走出有效的一步"，**不是**"有没有到位"：
  //    伺服模式（max_iterations = 1）单帧位移被 qd_max 限幅，通常远达不到容差；
  //    若拿"到位"当可用性判据，就会每帧判失败、每帧回退，把臂冻住。
  //    （注意顺序：converged 参与 ok 的构成，故必须排在前面算。）
  out.ok = stepped || out.converged;
  if (!out.ok) {
    out.q = q_hint;
  }

  // ③ 诊断量一律描述**最终返回的 q**，保证调用方读到什么就对应什么。
  const Eigen::Isometry3d T_out = model_.fkMoveItQ(out.q);
  out.diag.pos_err_m = (T_target_model.translation() - T_out.translation()).norm();
  out.diag.rot_err_rad = rotationVector(
    T_target_model.linear() * T_out.linear().transpose()).norm();
  out.diag.worst_limit_margin = limitMargin(out.q);
  out.diag.solve_time_us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - t0).count();
  return out;
}

FlexQpIkSolution FlexQpIk::solveXyzRpy(
  const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy,
  const JointArray & q_hint, double phi_des)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = xyz;
  T.linear() = (
    Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return solve(T, q_hint, phi_des);
}

FlexQpIkSolution FlexQpIk::solveXyzQuat(
  const Eigen::Vector3d & xyz, const Eigen::Quaterniond & quat_xyzw,
  const JointArray & q_hint, double phi_des)
{
  Eigen::Quaterniond q = quat_xyzw;
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = xyz;
  T.linear() = q.toRotationMatrix();
  return solve(T, q_hint, phi_des);
}

double effectivePosTolerance(
  const SpeedScaledTolerance & st, double task_lin_speed)
{
  if (!st.enabled) {
    return st.pos_tol_min_m;
  }
  return std::clamp(
    st.pos_tol_min_m + st.pos_k_v_s * std::abs(task_lin_speed),
    st.pos_tol_min_m, st.pos_tol_max_m);
}

double effectiveRotTolerance(
  const SpeedScaledTolerance & st, double task_ang_speed)
{
  if (!st.enabled) {
    return st.rot_tol_min_rad;
  }
  return std::clamp(
    st.rot_tol_min_rad + st.rot_k_v_s * std::abs(task_ang_speed),
    st.rot_tol_min_rad, st.rot_tol_max_rad);
}

}  // namespace flex_qp_ik
