// SPDX-License-Identifier: BSD-3-Clause
//
// S-R-S 臂角（psi / 肘摆角）：与 srs_analytic_ik::computePsiFromTheta 同定义。
// 供 QP 零空间次任务使用；交付 .so 自包含，不依赖解析 IK 包。

#ifndef FLEX_QP_IK__ARM_ANGLE_HPP_
#define FLEX_QP_IK__ARM_ANGLE_HPP_

#include <array>
#include <cmath>

#include <Eigen/Core>

#include "srs_dh_model/srs_dh_model.hpp"
#include "srs_dh_model/srs_geometry.hpp"

namespace flex_qp_ik
{

/// ψ 的取值状态。
///
/// ★ 为什么需要它：几何退化（肩-肘-腕共线等）时**不返回 NaN**，而是回退成
///   `wrap(θ3)`——一个与 ψ 定义完全不同的量，且是有限值。调用方若只判
///   `isfinite(psi)` 会把回退值当真值用。更要命的是：调用方传的 `phi_des` 与
///   求解器内部算的 ψ 用的是同一个函数，退化时**两边返回同一个回退值**
///   ⇒ 误差恒为 0 ⇒ 零空间次任务静默变成 no-op，而所有诊断看起来都正常。
enum class ArmAngleStatus
{
  /// 正常算出（含"接近奇异但仍有定义"的情形，此时看 conditioning）。
  kOk = 0,
  /// 几何退化，返回值是回退量 `wrap(θ3)`，**不是臂角**。
  kFallbackDegenerate = 1,
};

/// ψ 的取值 + 可信度。
struct ArmAngleInfo
{
  /// 臂角（rad）；status==kFallbackDegenerate 时是回退量 wrap(θ3)。
  double psi{0.0};
  ArmAngleStatus status{ArmAngleStatus::kOk};
  /// **可观测性度量** ∈ [0, 1]：肩-肘-腕两平面法向夹角的 sin
  /// （即归一化法向叉积的范数）。0 = 两平面法向共线 ⇒ ψ 不可定义；
  /// 越小 ψ 越病态（差分雅可比越不可信）。退化时置 0。
  ///
  /// 用途：伺服回路据此决定"这一帧的 ψ 次任务靠不靠谱"。经验阈值取
  /// sin(5°)≈0.087 量级即可（肘离完全伸直约 5° 以内就不要再信 ψ）。
  double conditioning{0.0};
};

/// 由 DH 关节角反推臂角 psi（rad）+ 状态 + 可观测性。
ArmAngleInfo computeArmAngleDhInfo(
  const srs_dh_model::SrsGeometry & geometry,
  const std::array<double, srs_dh_model::kArmDoF> & theta_dh);

/// 由 DH 关节角反推臂角 psi（rad）。退化时回退为 wrap(θ3)。
///
/// 兼容入口：等价于 `computeArmAngleDhInfo(...).psi`。**新代码请用 Info 版**，
/// 否则拿不到"当前值是不是回退量"的信息。
double computeArmAngleDh(
  const srs_dh_model::SrsGeometry & geometry,
  const std::array<double, srs_dh_model::kArmDoF> & theta_dh);

/// 由 MoveIt 关节角反推臂角 + 状态 + 可观测性（先做 q→θ 映射）。
ArmAngleInfo computeArmAngleMoveItQInfo(
  const srs_dh_model::SrsDhModel & model,
  const std::array<double, srs_dh_model::kArmDoF> & q_moveit);

/// 由 MoveIt 关节角反推臂角（先做 q→θ 映射）。兼容入口，见上。
double computeArmAngleMoveItQ(
  const srs_dh_model::SrsDhModel & model,
  const std::array<double, srs_dh_model::kArmDoF> & q_moveit);

/// ∂ψ/∂q（MoveIt 约定），1×7 行向量。中心差分；奇异附近元仍有限。
Eigen::Matrix<double, 1, srs_dh_model::kArmDoF> armAngleJacobianMoveItQ(
  const srs_dh_model::SrsDhModel & model,
  const std::array<double, srs_dh_model::kArmDoF> & q_moveit,
  double delta_rad = 1e-6);

/// 把角误差折到 (−π, π]。
inline double wrapAngleRad(double err)
{
  return std::atan2(std::sin(err), std::cos(err));
}

}  // namespace flex_qp_ik

#endif  // FLEX_QP_IK__ARM_ANGLE_HPP_
