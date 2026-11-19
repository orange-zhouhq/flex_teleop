// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：flex_qp_ik_c.cpp
// 包角色：flex_qp_ik_c.h（稳定 C 接口）的实现——把裸 double 世界翻译成 C++ 世界
//
// 本文件是唯一允许同时看见 C 与 C++ 两侧的地方，边界纪律：
//   - 对外只暴露 flex_qp_ik_c.h 里声明的 `extern "C"` 符号；
//   - 绝不让异常、Eigen 类型或 std:: 类型穿过边界；
//   - 所有入口先做**指针与有限性校验**，再进 C++ 层。
//
// 异常纪律（与 flex_qp_ik.cpp 一致）：构造函数会抛 std::invalid_argument
// （限位未填），本层必须兜住并转成 NULL + last_error，绝不让异常穿过 C 边界
// ——那种情况下调用方是 C 代码，未捕获的 C++ 异常是未定义行为。
// =============================================================================

#include "flex_qp_ik_c.h"

#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <new>
#include <string>

#include <Eigen/Geometry>

#include "flex_qp_ik/arm_angle.hpp"
#include "flex_qp_ik/flex_qp_ik.hpp"

namespace
{

/// 线程局部的最近错误说明。C 接口没有句柄可挂错误时（如 create 失败）靠它。
thread_local std::string g_last_error;

flex_qp_ik_status fail(flex_qp_ik_status code, const std::string & message)
{
  g_last_error = message;
  return code;
}

void clearError()
{
  g_last_error.clear();
}

/// 4x4 行主序 double[16] → Eigen 刚体变换。
Eigen::Isometry3d toIsometry(const double m[16])
{
  Eigen::Matrix4d T;
  // 行主序：T(row, col) = m[row*4 + col]
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      T(r, c) = m[r * 4 + c];
    }
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.matrix() = T;
  return out;
}

/// Eigen 刚体变换 → 4x4 行主序 double[16]。
void fromIsometry(const Eigen::Isometry3d & T, double m[16])
{
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      m[r * 4 + c] = T.matrix()(r, c);
    }
  }
}

/// 数组是否全为有限值。
bool allFinite(const double * v, std::size_t n)
{
  for (std::size_t i = 0; i < n; ++i) {
    if (!std::isfinite(v[i])) {
      return false;
    }
  }
  return true;
}

/// 裸数组 ↔ JointArray。刻意用裸指针而非数组引用模板：
/// C 侧传进来的形参会退化成指针，模板参数无法推导。
flex_qp_ik::JointArray toJointArray(const double * src)
{
  flex_qp_ik::JointArray out{};
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out[i] = src[i];
  }
  return out;
}

void fromJointArray(const flex_qp_ik::JointArray & src, double * dst)
{
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    dst[i] = src[i];
  }
}

srs_dh_model::SrsGeometry toGeometry(const flex_qp_ik_geometry_t & g)
{
  srs_dh_model::SrsGeometry out;
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out.a[i] = g.a[i];
    out.alpha[i] = g.alpha[i];
    out.d[i] = g.d[i];
    out.lower[i] = g.lower[i];
    out.upper[i] = g.upper[i];
    out.mapping.q_sign[i] = g.q_sign[i];
    out.mapping.q_offset[i] = g.q_offset[i];
  }
  out.T_tc_link_dh_base = toIsometry(g.T_tc_link_dh_base);
  out.T_dh_flange_link7 = toIsometry(g.T_dh_flange_link7);
  return out;
}

flex_qp_ik::FlexQpIkOptions toOptions(const flex_qp_ik_options_t & o)
{
  flex_qp_ik::FlexQpIkOptions out;
  out.pos_gain_per_step = o.pos_gain_per_step;
  out.rot_gain_per_step = o.rot_gain_per_step;
  out.max_linear_vel = o.max_linear_vel;
  out.max_angular_vel = o.max_angular_vel;
  for (std::size_t i = 0; i < 6; ++i) {
    out.w_task[i] = o.w_task[i];
  }
  out.enable_adaptive_damping = o.enable_adaptive_damping != 0;
  out.damping_floor = o.damping_floor;
  out.damping_span = o.damping_span;
  out.damping_sigma_threshold = o.damping_sigma_threshold;
  out.enable_posture = o.enable_posture != 0;
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out.w_posture[i] = o.w_posture[i];
  }
  out.k_posture = o.k_posture;
  out.posture_nullspace_damping = o.posture_nullspace_damping;
  out.limit_barrier_gain = o.limit_barrier_gain;
  out.limit_margin_rad = o.limit_margin_rad;
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out.qd_max[i] = o.qd_max[i];
    out.qd_min[i] = o.qd_min[i];
  }
  out.max_iterations = o.max_iterations;
  out.solve_step_s = o.solve_step_s;
  out.early_exit_pos_tol_m = o.early_exit_pos_tol_m;
  out.early_exit_rot_tol_rad = o.early_exit_rot_tol_rad;
  out.converged_pos_tol_m = o.converged_pos_tol_m;
  out.converged_rot_tol_rad = o.converged_rot_tol_rad;
  out.w_phi = o.w_phi;
  out.k_phi = o.k_phi;
  // ABI ≥ 6。★ q_pref 直接逐元素拷贝即可：C 侧约定 NaN = 限位中心，
  //   与 C++ 的 kAuto 是同一个值，不需要哨兵转换。
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out.q_pref[i] = o.q_pref[i];
  }
  out.phi_qd_max_rad_s = o.phi_qd_max_rad_s;
  out.phi_deadband_rad = o.phi_deadband_rad;
  out.phi_gate_pos_err_m = o.phi_gate_pos_err_m;
  out.phi_cond_min = o.phi_cond_min;
  out.phi_exit_tol_rad = o.phi_exit_tol_rad;
  // ABI ≥ 7
  out.phi_potential_mode =
    static_cast<flex_qp_ik::FlexQpIkOptions::PhiPotentialMode>(o.phi_potential_mode);
  out.phi_potential_w_ref = o.phi_potential_w_ref;
  out.phi_potential_w_limit = o.phi_potential_w_limit;
  out.phi_potential_psi_min = o.phi_potential_psi_min;
  out.phi_potential_psi_max = o.phi_potential_psi_max;
  out.phi_potential_margin = o.phi_potential_margin;
  return out;
}

void fromOptions(const flex_qp_ik::FlexQpIkOptions & o, flex_qp_ik_options_t & out)
{
  out.pos_gain_per_step = o.pos_gain_per_step;
  out.rot_gain_per_step = o.rot_gain_per_step;
  out.max_linear_vel = o.max_linear_vel;
  out.max_angular_vel = o.max_angular_vel;
  for (std::size_t i = 0; i < 6; ++i) {
    out.w_task[i] = o.w_task[i];
  }
  out.enable_adaptive_damping = o.enable_adaptive_damping ? 1 : 0;
  out.damping_floor = o.damping_floor;
  out.damping_span = o.damping_span;
  out.damping_sigma_threshold = o.damping_sigma_threshold;
  out.enable_posture = o.enable_posture ? 1 : 0;
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out.w_posture[i] = o.w_posture[i];
  }
  out.k_posture = o.k_posture;
  out.posture_nullspace_damping = o.posture_nullspace_damping;
  out.limit_barrier_gain = o.limit_barrier_gain;
  out.limit_margin_rad = o.limit_margin_rad;
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out.qd_max[i] = o.qd_max[i];
    out.qd_min[i] = o.qd_min[i];
  }
  out.max_iterations = o.max_iterations;
  out.solve_step_s = o.solve_step_s;
  out.early_exit_pos_tol_m = o.early_exit_pos_tol_m;
  out.early_exit_rot_tol_rad = o.early_exit_rot_tol_rad;
  out.converged_pos_tol_m = o.converged_pos_tol_m;
  out.converged_rot_tol_rad = o.converged_rot_tol_rad;
  out.w_phi = o.w_phi;
  out.k_phi = o.k_phi;
  // ABI ≥ 6。★ 必须显式写：三个预设都是先 memset(0) 再调本函数，漏映射会让
  //   q_pref 变成 0.0（"偏好全零构型"，一个合法值、无法自检），
  //   phi_exit_tol_rad 变成 0（ψ 早退永不成立）。
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    out.q_pref[i] = o.q_pref[i];
  }
  out.phi_qd_max_rad_s = o.phi_qd_max_rad_s;
  out.phi_deadband_rad = o.phi_deadband_rad;
  out.phi_gate_pos_err_m = o.phi_gate_pos_err_m;
  out.phi_cond_min = o.phi_cond_min;
  out.phi_exit_tol_rad = o.phi_exit_tol_rad;
  // ABI ≥ 7
  out.phi_potential_mode = static_cast<int>(o.phi_potential_mode);
  out.phi_potential_w_ref = o.phi_potential_w_ref;
  out.phi_potential_w_limit = o.phi_potential_w_limit;
  out.phi_potential_psi_min = o.phi_potential_psi_min;
  out.phi_potential_psi_max = o.phi_potential_psi_max;
  out.phi_potential_margin = o.phi_potential_margin;
}

void fromDiagnostics(const flex_qp_ik::FlexQpIkDiagnostics & d, flex_qp_ik_diagnostics_t & out)
{
  out.qp_status = static_cast<int>(d.qp_status);
  out.qp_iterations = d.qp_iterations;
  out.qp_time_us = d.qp_time_us;
  out.pos_err_m = d.pos_err_m;
  out.rot_err_rad = d.rot_err_rad;
  out.worst_limit_margin = d.worst_limit_margin;
  out.iterations_used = d.iterations_used;
  out.damping_used = d.damping_used;
  out.sigma_min = d.sigma_min;
  out.qp_hit_iteration_cap = d.qp_hit_iteration_cap ? 1 : 0;
  out.qp_hit_iteration_cap_total = d.qp_hit_iteration_cap_total;
  out.calls_total = d.calls_total;
  out.solve_time_us = d.solve_time_us;
  // ABI ≥ 6：臂角次任务诊断。
  out.phi_fallback = d.phi_fallback;
  out.phi_conditioning = d.phi_conditioning;
  out.phi_gated = d.phi_gated;
}

/// 句柄的真实类型。`flex_qp_ik_t` 在头文件里是不完整类型，这里才完整。
///
/// 刻意**同时持有几何与选项**而不是只持有 FlexQpIk：这样 set_options 能在原地
/// 用新选项重建求解器（调用方持有的指针不变），且不丢失几何。
struct Handle
{
  srs_dh_model::SrsGeometry geometry{};
  /// **始终可用**——FK / 雅可比 / 限位余量都直接用它，不经过 FlexQpIk。
  /// 这样"只做运动学"的句柄不需要构造求解器，也不需要关节限位。
  srs_dh_model::SrsDhModel model;
  flex_qp_ik::FlexQpIkOptions options{};
  /// 仅 IK 句柄持有；运动学专用句柄为 null。
  std::unique_ptr<flex_qp_ik::FlexQpIk> ik{};

  Handle(srs_dh_model::SrsGeometry g, flex_qp_ik::FlexQpIkOptions o)
  : geometry(std::move(g)), model(geometry), options(std::move(o))
  {
    rebuild();
  }

  /// 运动学专用：不构造 QP，也不要求关节限位。
  explicit Handle(srs_dh_model::SrsGeometry g)
  : geometry(std::move(g)), model(geometry)
  {
  }

  /// 按当前 geometry/options 重建求解器（丢弃 QP 热启动状态）。
  void rebuild()
  {
    ik = std::make_unique<flex_qp_ik::FlexQpIk>(geometry, options);
  }

  bool hasIk() const { return ik != nullptr; }
};

}  // namespace

// ── 版本 ────────────────────────────────────────────────────────────────────

extern "C" const char * flex_qp_ik_version(void)
{
  return "0.1.0";
}

extern "C" int flex_qp_ik_abi_version(void)
{
  return FLEX_QP_IK_ABI_VERSION;
}

/// ★ 注意它必须在**匿名命名空间之外**：早先版本把它塞在了
///   `namespace { ... }` 里（紧跟同样是内部链接的 fromDiagnostics），
///   于是它拿到的是**内部链接**，`-fvisibility=hidden` 之下根本不导出——
///   症状是链接期 `undefined reference to flex_qp_ik_sizeof_diagnostics`。
///   只有"用外部 C 消费者去链接"才暴露得出来，纯看源码看不出来。
extern "C" size_t flex_qp_ik_sizeof_diagnostics(void)
{
  return sizeof(flex_qp_ik_diagnostics_t);
}

extern "C" const char * flex_qp_ik_last_error(void)
{
  return g_last_error.c_str();
}

// ── 默认值与 helper ─────────────────────────────────────────────────────────

extern "C" flex_qp_ik_options_t flex_qp_ik_default_options(void)
{
  // 由 C++ 侧默认值导出，保证两侧默认值**永远一致**（不手工抄一遍，
  // 那必然会随时间漂移）。
  const flex_qp_ik::FlexQpIkOptions def{};
  flex_qp_ik_options_t out;
  std::memset(&out, 0, sizeof(out));
  fromOptions(def, out);
  return out;
}

extern "C" flex_qp_ik_options_t flex_qp_ik_options_for_servo(double tick_s)
{
  flex_qp_ik_options_t out;
  std::memset(&out, 0, sizeof(out));
  fromOptions(flex_qp_ik::FlexQpIkOptions::forServo(tick_s), out);
  return out;
}

extern "C" flex_qp_ik_options_t flex_qp_ik_options_for_oneshot(void)
{
  flex_qp_ik_options_t out;
  std::memset(&out, 0, sizeof(out));
  fromOptions(flex_qp_ik::FlexQpIkOptions::forOneShot(), out);
  return out;
}

extern "C" void flex_qp_ik_geometry_identity(flex_qp_ik_geometry_t * geometry)
{
  if (geometry == nullptr) {
    return;
  }
  std::memset(geometry, 0, sizeof(*geometry));
  for (std::size_t i = 0; i < FLEX_QP_IK_DOF; ++i) {
    geometry->q_sign[i] = 1.0;
  }
  for (int i = 0; i < 4; ++i) {
    geometry->T_tc_link_dh_base[i * 4 + i] = 1.0;
    geometry->T_dh_flange_link7[i * 4 + i] = 1.0;
  }
}

namespace
{

/// RPY（ZYX 内旋，与 ROS 一致）→ 旋转矩阵。
Eigen::Matrix3d rpyToRotation(const double rpy[3])
{
  return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
}

void writeIsometry(const Eigen::Matrix3d & R, const double xyz[3], double T_out[16])
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T_out[r * 4 + c] = R(r, c);
    }
    T_out[r * 4 + 3] = xyz[r];
  }
  for (int c = 0; c < 3; ++c) {
    T_out[12 + c] = 0.0;
  }
  T_out[15] = 1.0;
}

}  // namespace

extern "C" void flex_qp_ik_geometry_set_transform(
  flex_qp_ik_geometry_t * geometry, flex_qp_ik_xform which,
  const double xyz[3], const double rpy[3])
{
  if (geometry == nullptr || xyz == nullptr || rpy == nullptr) {
    return;
  }
  double * dst = (which == FLEX_QP_IK_XFORM_DH_FLANGE_TO_LINK7)
    ? geometry->T_dh_flange_link7 : geometry->T_tc_link_dh_base;
  writeIsometry(rpyToRotation(rpy), xyz, dst);
}

extern "C" void flex_qp_ik_pose_from_xyzrpy(
  const double xyz[3], const double rpy[3], double T_out[16])
{
  if (xyz == nullptr || rpy == nullptr || T_out == nullptr) {
    return;
  }
  writeIsometry(rpyToRotation(rpy), xyz, T_out);
}

extern "C" void flex_qp_ik_pose_from_xyzquat(
  const double xyz[3], const double quat_xyzw[4], double T_out[16])
{
  if (xyz == nullptr || quat_xyzw == nullptr || T_out == nullptr) {
    return;
  }
  Eigen::Quaterniond q(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
  // 模长接近 0 时退化为单位旋转，避免 NaN 传播到下游。
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  writeIsometry(q.toRotationMatrix(), xyz, T_out);
}

extern "C" double flex_qp_ik_effective_pos_tolerance(
  int enabled, double tol_min_m, double tol_max_m, double k_v_s, double task_lin_speed)
{
  return flex_qp_ik::effectivePosTolerance(
    flex_qp_ik::SpeedScaledTolerance{
      enabled != 0, tol_min_m, tol_max_m, k_v_s, 0.0, 0.0, 0.0},
    task_lin_speed);
}

extern "C" double flex_qp_ik_effective_rot_tolerance(
  int enabled, double tol_min_rad, double tol_max_rad, double k_v_s, double task_ang_speed)
{
  return flex_qp_ik::effectiveRotTolerance(
    flex_qp_ik::SpeedScaledTolerance{
      enabled != 0, 0.0, 0.0, 0.0, tol_min_rad, tol_max_rad, k_v_s},
    task_ang_speed);
}

extern "C" const char * flex_qp_ik_qp_status_str(int qp_status)
{
  return flex_qp_ik::toString(static_cast<flex_qp_ik::QpStatus>(qp_status));
}

// ── 生命周期 ────────────────────────────────────────────────────────────────

extern "C" flex_qp_ik_t * flex_qp_ik_create(
  const flex_qp_ik_geometry_t * geometry,
  const flex_qp_ik_options_t * options,
  size_t options_size,
  flex_qp_ik_status * status)
{
  clearError();
  auto setStatus = [status](flex_qp_ik_status s) {
      if (status != nullptr) {
        *status = s;
      }
    };

  if (geometry == nullptr) {
    setStatus(fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "geometry 为空"));
    return nullptr;
  }
  // ★ 版本校验：头文件与 .so 错配时宁可拒绝，也不要按错误布局读结构体。
  if (options != nullptr && options_size != sizeof(flex_qp_ik_options_t)) {
    setStatus(fail(
      FLEX_QP_IK_ERR_INVALID_ARGUMENT,
      "options_size=" + std::to_string(options_size) + " 与库的 " +
      std::to_string(sizeof(flex_qp_ik_options_t)) +
      " 不符（头文件与 .so 版本不匹配）"));
    return nullptr;
  }
  if (!allFinite(geometry->a, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->alpha, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->d, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->lower, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->upper, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->q_sign, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->q_offset, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->T_tc_link_dh_base, 16) ||
    !allFinite(geometry->T_dh_flange_link7, 16))
  {
    setStatus(fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "geometry 含非有限值"));
    return nullptr;
  }

  try {
    const flex_qp_ik::FlexQpIkOptions opt =
      options != nullptr ? toOptions(*options) : flex_qp_ik::FlexQpIkOptions{};
    auto * handle = new Handle(toGeometry(*geometry), opt);
    setStatus(FLEX_QP_IK_OK);
    return reinterpret_cast<flex_qp_ik_t *>(handle);
  } catch (const std::exception & e) {
    setStatus(fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, std::string("构造失败: ") + e.what()));
    return nullptr;
  } catch (...) {
    // 绝不让异常穿过 C 边界——调用方是 C 代码的话那是未定义行为。
    setStatus(fail(FLEX_QP_IK_ERR_INTERNAL, "构造失败: 未知异常"));
    return nullptr;
  }
}

extern "C" const char * flex_qp_ik_mode_str(flex_qp_ik_mode mode)
{
  return flex_qp_ik::toString(
    mode == FLEX_QP_IK_MODE_ONESHOT ? flex_qp_ik::FlexQpIkMode::kOneShot
                                   : flex_qp_ik::FlexQpIkMode::kServo);
}

extern "C" flex_qp_ik_t * flex_qp_ik_create_with_mode(
  const flex_qp_ik_geometry_t * geometry, flex_qp_ik_mode mode, double tick_s,
  flex_qp_ik_status * status)
{
  const flex_qp_ik_options_t opt =
    mode == FLEX_QP_IK_MODE_ONESHOT ? flex_qp_ik_options_for_oneshot()
                                   : flex_qp_ik_options_for_servo(tick_s);
  return flex_qp_ik_create(geometry, &opt, sizeof(opt), status);
}

extern "C" flex_qp_ik_t * flex_qp_ik_create_kinematics_only(
  const flex_qp_ik_geometry_t * geometry, flex_qp_ik_status * status)
{
  clearError();
  auto setStatus = [status](flex_qp_ik_status s) {
      if (status != nullptr) {
        *status = s;
      }
    };
  if (geometry == nullptr) {
    setStatus(fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "geometry 为空"));
    return nullptr;
  }
  if (!allFinite(geometry->a, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->alpha, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->d, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->q_sign, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->q_offset, FLEX_QP_IK_DOF) ||
    !allFinite(geometry->T_tc_link_dh_base, 16) ||
    !allFinite(geometry->T_dh_flange_link7, 16))
  {
    setStatus(fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "geometry 含非有限值"));
    return nullptr;
  }
  // ★ 刻意**不校验 lower/upper**：运动学用不到限位，而"没填限位"正是
  //   运动学专用句柄要支持的场景（标定校验/奇异分析/可视化）。
  try {
    auto * h = new Handle(toGeometry(*geometry));
    setStatus(FLEX_QP_IK_OK);
    return reinterpret_cast<flex_qp_ik_t *>(h);
  } catch (const std::exception & e) {
    setStatus(fail(FLEX_QP_IK_ERR_INTERNAL, std::string("构造失败: ") + e.what()));
    return nullptr;
  } catch (...) {
    setStatus(fail(FLEX_QP_IK_ERR_INTERNAL, "构造失败: 未知异常"));
    return nullptr;
  }
}

/// 需要 QP 求解器的入口统一在这里挡一道。
static flex_qp_ik_status requireIk(Handle * h)
{
  if (h == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "句柄为空");
  }
  if (!h->hasIk()) {
    return fail(
      FLEX_QP_IK_ERR_INVALID_ARGUMENT,
      "该句柄是运动学专用（flex_qp_ik_create_kinematics_only），不含 IK 求解器；"
      "需要 IK 请用 flex_qp_ik_create_with_mode");
  }
  return FLEX_QP_IK_OK;
}

extern "C" void flex_qp_ik_destroy(flex_qp_ik_t * handle)
{
  delete reinterpret_cast<Handle *>(handle);
}

extern "C" flex_qp_ik_status flex_qp_ik_set_options(
  flex_qp_ik_t * handle, const flex_qp_ik_options_t * options, size_t options_size)
{
  clearError();
  if (handle == nullptr || options == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "句柄或 options 为空");
  }
  if (options_size != sizeof(flex_qp_ik_options_t)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "options_size 与库不符");
  }

  auto * h = reinterpret_cast<Handle *>(handle);
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return rc;
  }
  try {
    // 原地重建：调用方持有的指针不变。
    // ★ 代价是丢弃 QP 热启动状态，故**不要每帧调用**（改配置时才用）。
    h->options = toOptions(*options);
    h->rebuild();
    return FLEX_QP_IK_OK;
  } catch (const std::exception & e) {
    return fail(
      FLEX_QP_IK_ERR_INVALID_ARGUMENT, std::string("set_options 失败: ") + e.what());
  } catch (...) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, "set_options 失败: 未知异常");
  }
}

// ── 求解 ────────────────────────────────────────────────────────────────────

namespace
{

flex_qp_ik_status solveVelocityImpl(
  flex_qp_ik_t * handle,
  const double q[FLEX_QP_IK_DOF],
  const double twist_des[6],
  double phi_des,
  double qd_out[FLEX_QP_IK_DOF],
  flex_qp_ik_diagnostics_t * diagnostics)
{
  clearError();
  if (handle == nullptr || q == nullptr || twist_des == nullptr || qd_out == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "空指针");
  }
  if (!allFinite(q, FLEX_QP_IK_DOF) || !allFinite(twist_des, 6)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "q 或 twist_des 含非有限值");
  }

  auto * h = reinterpret_cast<Handle *>(handle);
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return rc;
  }
  const flex_qp_ik::JointArray q_arr = toJointArray(q);
  Eigen::Matrix<double, 6, 1> twist;
  for (int i = 0; i < 6; ++i) {
    twist[i] = twist_des[i];
  }

  flex_qp_ik::JointArray qd{};
  flex_qp_ik::FlexQpIkDiagnostics diag;
  bool ok = false;
  try {
    ok = h->ik->solveVelocity(q_arr, twist, qd, &diag, phi_des);
  } catch (const std::exception & e) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, std::string("solveVelocity 异常: ") + e.what());
  } catch (...) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, "solveVelocity 异常");
  }

  if (diagnostics != nullptr) {
    fromDiagnostics(diag, *diagnostics);
  }
  if (!ok) {
    return fail(FLEX_QP_IK_ERR_NOT_SOLVED, "QP 未解出可用解");
  }
  fromJointArray(qd, qd_out);
  return FLEX_QP_IK_OK;
}

flex_qp_ik_status solvePoseImpl(
  flex_qp_ik_t * handle, const double T_target[16], const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution, flex_qp_ik_diagnostics_t * diagnostics)
{
  clearError();
  if (handle == nullptr || T_target == nullptr || q_hint == nullptr || solution == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "空指针");
  }
  if (!allFinite(T_target, 16) || !allFinite(q_hint, FLEX_QP_IK_DOF)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "T_target 或 q_hint 含非有限值");
  }

  auto * h = reinterpret_cast<Handle *>(handle);
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return rc;
  }
  const Eigen::Isometry3d T = toIsometry(T_target);
  const flex_qp_ik::JointArray hint = toJointArray(q_hint);

  flex_qp_ik::FlexQpIkSolution sol;
  try {
    sol = h->ik->solve(T, hint, phi_des);
  } catch (const std::exception & e) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, std::string("solve 异常: ") + e.what());
  } catch (...) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, "solve 异常");
  }

  solution->ok = sol.ok ? 1 : 0;
  solution->converged = sol.converged ? 1 : 0;
  fromJointArray(sol.q, solution->q);
  solution->solve_time_us = sol.diag.solve_time_us;
  solution->pos_err_m = sol.diag.pos_err_m;
  solution->rot_err_rad = sol.diag.rot_err_rad;
  if (diagnostics != nullptr) {
    fromDiagnostics(sol.diag, *diagnostics);
  }
  return FLEX_QP_IK_OK;
}

}  // namespace

extern "C" flex_qp_ik_status flex_qp_ik_solve_velocity(
  flex_qp_ik_t * handle,
  const double q[FLEX_QP_IK_DOF],
  const double twist_des[6],
  double qd_out[FLEX_QP_IK_DOF],
  flex_qp_ik_diagnostics_t * diagnostics)
{
  return solveVelocityImpl(
    handle, q, twist_des, flex_qp_ik::kAuto, qd_out, diagnostics);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_velocity_phi(
  flex_qp_ik_t * handle,
  const double q[FLEX_QP_IK_DOF],
  const double twist_des[6],
  double phi_des,
  double qd_out[FLEX_QP_IK_DOF],
  flex_qp_ik_diagnostics_t * diagnostics)
{
  return solveVelocityImpl(handle, q, twist_des, phi_des, qd_out, diagnostics);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_pose(
  flex_qp_ik_t * handle, const double T_target[16], const double q_hint[FLEX_QP_IK_DOF],
  flex_qp_ik_solution_t * solution)
{
  return solvePoseImpl(handle, T_target, q_hint, flex_qp_ik::kAuto, solution, nullptr);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_pose_ex(
  flex_qp_ik_t * handle, const double T_target[16], const double q_hint[FLEX_QP_IK_DOF],
  flex_qp_ik_solution_t * solution, flex_qp_ik_diagnostics_t * diagnostics)
{
  return solvePoseImpl(handle, T_target, q_hint, flex_qp_ik::kAuto, solution, diagnostics);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_pose_phi(
  flex_qp_ik_t * handle,
  const double T_target[16],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution)
{
  return solvePoseImpl(handle, T_target, q_hint, phi_des, solution, nullptr);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_pose_phi_ex(
  flex_qp_ik_t * handle,
  const double T_target[16],
  const double q_hint[FLEX_QP_IK_DOF],
  double phi_des,
  flex_qp_ik_solution_t * solution,
  flex_qp_ik_diagnostics_t * diagnostics)
{
  return solvePoseImpl(handle, T_target, q_hint, phi_des, solution, diagnostics);
}

namespace
{

flex_qp_ik_status solveXyzRpyImpl(
  flex_qp_ik_t * handle,
  const double xyz[3], const double rpy[3],
  const double q_hint[FLEX_QP_IK_DOF], double phi_des,
  flex_qp_ik_solution_t * solution, flex_qp_ik_diagnostics_t * diagnostics)
{
  clearError();
  if (handle == nullptr || xyz == nullptr || rpy == nullptr ||
    q_hint == nullptr || solution == nullptr)
  {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "空指针");
  }
  if (!allFinite(xyz, 3) || !allFinite(rpy, 3) || !allFinite(q_hint, FLEX_QP_IK_DOF)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "xyz/rpy/q_hint 含非有限值");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return rc;
  }

  flex_qp_ik::FlexQpIkSolution sol;
  try {
    sol = h->ik->solveXyzRpy(
      Eigen::Vector3d(xyz[0], xyz[1], xyz[2]),
      Eigen::Vector3d(rpy[0], rpy[1], rpy[2]),
      toJointArray(q_hint), phi_des);
  } catch (const std::exception & e) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, std::string("solveXyzRpy 异常: ") + e.what());
  } catch (...) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, "solveXyzRpy 异常");
  }

  solution->ok = sol.ok ? 1 : 0;
  solution->converged = sol.converged ? 1 : 0;
  fromJointArray(sol.q, solution->q);
  solution->solve_time_us = sol.diag.solve_time_us;
  solution->pos_err_m = sol.diag.pos_err_m;
  solution->rot_err_rad = sol.diag.rot_err_rad;
  if (diagnostics != nullptr) {
    fromDiagnostics(sol.diag, *diagnostics);
  }
  return FLEX_QP_IK_OK;
}

flex_qp_ik_status solveXyzQuatImpl(
  flex_qp_ik_t * handle,
  const double xyz[3], const double quat_xyzw[4],
  const double q_hint[FLEX_QP_IK_DOF], double phi_des,
  flex_qp_ik_solution_t * solution, flex_qp_ik_diagnostics_t * diagnostics)
{
  clearError();
  if (handle == nullptr || xyz == nullptr || quat_xyzw == nullptr ||
    q_hint == nullptr || solution == nullptr)
  {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "空指针");
  }
  if (!allFinite(xyz, 3) || !allFinite(quat_xyzw, 4) ||
    !allFinite(q_hint, FLEX_QP_IK_DOF))
  {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "xyz/quat/q_hint 含非有限值");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return rc;
  }

  flex_qp_ik::FlexQpIkSolution sol;
  try {
    Eigen::Quaterniond quat(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
    sol = h->ik->solveXyzQuat(
      Eigen::Vector3d(xyz[0], xyz[1], xyz[2]), quat,
      toJointArray(q_hint), phi_des);
  } catch (const std::exception & e) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, std::string("solveXyzQuat 异常: ") + e.what());
  } catch (...) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, "solveXyzQuat 异常");
  }

  solution->ok = sol.ok ? 1 : 0;
  solution->converged = sol.converged ? 1 : 0;
  fromJointArray(sol.q, solution->q);
  solution->solve_time_us = sol.diag.solve_time_us;
  solution->pos_err_m = sol.diag.pos_err_m;
  solution->rot_err_rad = sol.diag.rot_err_rad;
  if (diagnostics != nullptr) {
    fromDiagnostics(sol.diag, *diagnostics);
  }
  return FLEX_QP_IK_OK;
}

}  // namespace

extern "C" flex_qp_ik_status flex_qp_ik_solve_xyzrpy(
  flex_qp_ik_t * handle,
  const double xyz[3], const double rpy[3],
  const double q_hint[FLEX_QP_IK_DOF], double phi_des,
  flex_qp_ik_solution_t * solution)
{
  return solveXyzRpyImpl(handle, xyz, rpy, q_hint, phi_des, solution, nullptr);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_xyzrpy_ex(
  flex_qp_ik_t * handle,
  const double xyz[3], const double rpy[3],
  const double q_hint[FLEX_QP_IK_DOF], double phi_des,
  flex_qp_ik_solution_t * solution, flex_qp_ik_diagnostics_t * diagnostics)
{
  return solveXyzRpyImpl(handle, xyz, rpy, q_hint, phi_des, solution, diagnostics);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_xyzquat(
  flex_qp_ik_t * handle,
  const double xyz[3], const double quat_xyzw[4],
  const double q_hint[FLEX_QP_IK_DOF], double phi_des,
  flex_qp_ik_solution_t * solution)
{
  return solveXyzQuatImpl(handle, xyz, quat_xyzw, q_hint, phi_des, solution, nullptr);
}

extern "C" flex_qp_ik_status flex_qp_ik_solve_xyzquat_ex(
  flex_qp_ik_t * handle,
  const double xyz[3], const double quat_xyzw[4],
  const double q_hint[FLEX_QP_IK_DOF], double phi_des,
  flex_qp_ik_solution_t * solution, flex_qp_ik_diagnostics_t * diagnostics)
{
  return solveXyzQuatImpl(
    handle, xyz, quat_xyzw, q_hint, phi_des, solution, diagnostics);
}

// ── 工具 ────────────────────────────────────────────────────────────────────

extern "C" double flex_qp_ik_arm_angle(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF])
{
  clearError();
  if (handle == nullptr || q == nullptr || !allFinite(q, FLEX_QP_IK_DOF)) {
    fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "句柄或 q 非法");
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  try {
    return flex_qp_ik::computeArmAngleMoveItQ(h->model, toJointArray(q));
  } catch (...) {
    fail(FLEX_QP_IK_ERR_INTERNAL, "arm_angle 异常");
    return std::numeric_limits<double>::quiet_NaN();
  }
}

extern "C" flex_qp_ik_status flex_qp_ik_arm_angle_info(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF],
  double * psi_out, int * status_out, double * conditioning_out)
{
  clearError();
  if (handle == nullptr || q == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "空指针");
  }
  if (!allFinite(q, FLEX_QP_IK_DOF)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "q 含非有限值");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  try {
    const flex_qp_ik::ArmAngleInfo info =
      flex_qp_ik::computeArmAngleMoveItQInfo(h->model, toJointArray(q));
    if (psi_out != nullptr) {
      *psi_out = info.psi;
    }
    if (status_out != nullptr) {
      *status_out = static_cast<int>(info.status);
    }
    if (conditioning_out != nullptr) {
      *conditioning_out = info.conditioning;
    }
    return FLEX_QP_IK_OK;
  } catch (...) {
    return fail(FLEX_QP_IK_ERR_INTERNAL, "arm_angle_info 异常");
  }
}

extern "C" flex_qp_ik_status flex_qp_ik_set_tolerances(
  flex_qp_ik_t * handle,
  double early_exit_pos_tol_m, double early_exit_rot_tol_rad,
  double converged_pos_tol_m, double converged_rot_tol_rad)
{
  clearError();
  if (handle == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "句柄为空");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return rc;
  }
  // ★ 不重建、不丢热启动：只改 4 个标量（读取点只有位姿级 solve 的早退/汇报）。
  if (!h->ik->setTolerances(
      early_exit_pos_tol_m, early_exit_rot_tol_rad,
      converged_pos_tol_m, converged_rot_tol_rad))
  {
    return fail(
      FLEX_QP_IK_ERR_INVALID_ARGUMENT,
      "容差非法：要求各值 > 0 且 converged_* >= early_exit_*（违反后者会让 "
      "solve() 出现永久保持——早退命中时 stepped=false 且 converged=false ⇒ ok=false）");
  }
  // 同步 Handle 里的选项副本：否则下次 rebuild（set_options）会把新值退回旧值。
  h->options.early_exit_pos_tol_m = early_exit_pos_tol_m;
  h->options.early_exit_rot_tol_rad = early_exit_rot_tol_rad;
  h->options.converged_pos_tol_m = converged_pos_tol_m;
  h->options.converged_rot_tol_rad = converged_rot_tol_rad;
  return FLEX_QP_IK_OK;
}

extern "C" flex_qp_ik_status flex_qp_ik_get_options(
  flex_qp_ik_t * handle, flex_qp_ik_options_t * options, size_t options_size)
{
  clearError();
  if (handle == nullptr || options == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "句柄或 options 为空");
  }
  if (options_size != sizeof(flex_qp_ik_options_t)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "options_size 与库不符");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  // 返回的是**句柄当前生效**的选项（含运行期 set_tolerances / set_posture_pref
  // 改过的值），不是构造时那份。
  fromOptions(h->options, *options);
  return FLEX_QP_IK_OK;
}

extern "C" flex_qp_ik_status flex_qp_ik_set_posture_pref(
  flex_qp_ik_t * handle, const double q_pref[FLEX_QP_IK_DOF],
  double k_posture, const double w_posture[FLEX_QP_IK_DOF])
{
  clearError();
  if (handle == nullptr || q_pref == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "句柄或 q_pref 为空");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return rc;
  }
  // q_pref 允许 NaN（= 该关节取限位中心），故不做 allFinite 检查；
  // 其余非法值（±inf）在 FlexQpIk 内会被当作显式目标，这里显式挡掉。
  for (int i = 0; i < FLEX_QP_IK_DOF; ++i) {
    if (std::isinf(q_pref[i])) {
      return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "q_pref 含 ±inf（NaN 才表示自动）");
    }
  }

  flex_qp_ik::JointArray pref{};
  for (int i = 0; i < FLEX_QP_IK_DOF; ++i) {
    pref[static_cast<std::size_t>(i)] = q_pref[i];
  }
  flex_qp_ik::JointArray kw = h->options.w_posture;
  if (w_posture != nullptr) {
    for (int i = 0; i < FLEX_QP_IK_DOF; ++i) {
      kw[static_cast<std::size_t>(i)] = w_posture[i];
    }
  }
  const double k = (k_posture > 0.0) ? k_posture : h->options.k_posture;

  // ★ 走 FlexQpIk::setPosturePref：它会刷新 q_pref_resolved_（posture 次任务
  //   真正读取的那份）。只写 options 是静默 no-op。
  h->ik->setPosturePref(pref, k, kw);
  // 同步 Handle 副本（同 set_tolerances 的理由）。
  h->options.q_pref = pref;
  h->options.k_posture = k;
  h->options.w_posture = kw;
  return FLEX_QP_IK_OK;
}

extern "C" flex_qp_ik_status flex_qp_ik_forward_kinematics(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF], double T_out[16])
{
  clearError();
  if (handle == nullptr || q == nullptr || T_out == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "空指针");
  }
  if (!allFinite(q, FLEX_QP_IK_DOF)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "q 含非有限值");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  // 用 Handle::model 而不是 h->ik->model()：运动学专用句柄没有 ik，
  // 而 FK 本来就不需要求解器。
  fromIsometry(h->model.fkMoveItQ(toJointArray(q)), T_out);
  return FLEX_QP_IK_OK;
}

extern "C" flex_qp_ik_status flex_qp_ik_jacobian(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF], double J[42])
{
  clearError();
  if (handle == nullptr || q == nullptr || J == nullptr) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "空指针");
  }
  if (!allFinite(q, FLEX_QP_IK_DOF)) {
    return fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "q 含非有限值");
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  const Eigen::Matrix<double, 6, srs_dh_model::kArmDoF> Jm =
    h->model.jacobianMoveItQ(toJointArray(q));
  // 行主序展开：J[row*7 + col]
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < static_cast<int>(srs_dh_model::kArmDoF); ++c) {
      J[r * 7 + c] = Jm(r, c);
    }
  }
  return FLEX_QP_IK_OK;
}

extern "C" double flex_qp_ik_limit_margin(
  flex_qp_ik_t * handle, const double q[FLEX_QP_IK_DOF])
{
  clearError();
  if (handle == nullptr || q == nullptr || !allFinite(q, FLEX_QP_IK_DOF)) {
    fail(FLEX_QP_IK_ERR_INVALID_ARGUMENT, "句柄或 q 非法");
    return -1.0;  // 负数同时表达"越限/不可判定"，与 limitMargin 语义一致
  }
  auto * h = reinterpret_cast<Handle *>(handle);
  // 限位余量需要关节限位——运动学专用句柄刻意允许不填限位，故这里要挡。
  if (const auto rc = requireIk(h); rc != FLEX_QP_IK_OK) {
    return -1.0;   // 与"越限/不可判定"同义（负值）
  }
  return h->ik->limitMargin(toJointArray(q));
}
