// SPDX-License-Identifier: BSD-3-Clause
//
// 臂角 psi：与 srs_analytic_ik::SrsAnalyticIk::computePsiFromTheta 同几何定义
//（虚拟零肩滚转构型 vs vs 真实肩-肘-腕平面夹角，绕肩-腕轴）。

#include "flex_qp_ik/arm_angle.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "srs_dh_model/dh_math.hpp"

namespace flex_qp_ik
{
namespace
{

int gc4FromTheta(const std::array<double, srs_dh_model::kArmDoF> & theta_dh)
{
  return theta_dh[3] >= 0.0 ? 1 : -1;
}

}  // namespace

ArmAngleInfo computeArmAngleDhInfo(
  const srs_dh_model::SrsGeometry & geometry,
  const std::array<double, srs_dh_model::kArmDoF> & theta_dh)
{
  using srs_dh_model::internal::dhMatrix;
  using srs_dh_model::internal::safeAcos;

  const double d_bs = geometry.d[0];
  const double d_se = geometry.d[2];
  const double d_ew = geometry.d[4];
  const double d_wf = geometry.d[6];

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    T = T * dhMatrix(
      theta_dh[i], geometry.alpha[i], geometry.a[i], geometry.d[i]);
  }
  const Eigen::Vector3d p_07 = T.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_07 = T.block<3, 3>(0, 0);
  const int GC4 = gc4FromTheta(theta_dh);

  const Eigen::Vector3d p_02(0.0, 0.0, d_bs);
  const Eigen::Vector3d p_67(0.0, 0.0, d_wf);
  const Eigen::Vector3d p_26 = p_07 - p_02 - R_07 * p_67;
  const double p_26_norm = p_26.norm();
  const double fallback_psi =
    std::atan2(std::sin(theta_dh[2]), std::cos(theta_dh[2]));
  // 退化回退：返回的不是臂角，state 明确标出来（调用方只判 isfinite 会漏）。
  const auto fallback = [&]() {
      return ArmAngleInfo{fallback_psi, ArmAngleStatus::kFallbackDegenerate, 0.0};
    };
  if (p_26_norm < 1e-12) {
    return fallback();
  }

  double elbow_angle = 0.0;
  const double cos_elbow =
    (p_26.squaredNorm() - d_se * d_se - d_ew * d_ew) / (2.0 * d_se * d_ew);
  if (!safeAcos(cos_elbow, elbow_angle)) {
    return fallback();
  }
  const double theta_4v = static_cast<double>(GC4) * elbow_angle;

  std::array<double, srs_dh_model::kArmDoF> theta_vs{};
  theta_vs[0] = std::atan2(p_26.y(), p_26.x());
  double phi_shoulder = 0.0;
  const double acos_in =
    (d_se * d_se + p_26.squaredNorm() - d_ew * d_ew) / (2.0 * d_se * p_26_norm);
  if (!safeAcos(acos_in, phi_shoulder)) {
    return fallback();
  }
  theta_vs[1] =
    std::atan2(p_26.head<2>().norm(), p_26.z()) -
    static_cast<double>(GC4) * phi_shoulder;
  theta_vs[2] = 0.0;
  theta_vs[3] = theta_4v;

  auto chainFk = [&](const std::array<double, srs_dh_model::kArmDoF> & thetas,
      std::size_t count) -> Eigen::Vector3d {
    Eigen::Matrix4d Tc = Eigen::Matrix4d::Identity();
    for (std::size_t i = 0; i < count; ++i) {
      Tc = Tc * dhMatrix(
        thetas[i], geometry.alpha[i], geometry.a[i], geometry.d[i]);
    }
    return Eigen::Vector3d(Tc(0, 3), Tc(1, 3), Tc(2, 3));
  };

  const Eigen::Vector3d p_02_v = chainFk(theta_vs, 2);
  const Eigen::Vector3d p_04_v = chainFk(theta_vs, 4);
  const Eigen::Vector3d p_04 = chainFk(theta_dh, 4);
  const Eigen::Vector3d p_06 = chainFk(theta_dh, 6);

  const double v_se_v_norm = (p_04_v - p_02_v).norm();
  const double v_sw_v_norm = (p_06 - p_02_v).norm();
  const double v_se_norm = (p_04 - p_02).norm();
  const double v_sw_norm = (p_06 - p_02).norm();
  if (
    v_se_v_norm < 1e-12 || v_sw_v_norm < 1e-12 || v_se_norm < 1e-12 ||
    v_sw_norm < 1e-12)
  {
    return fallback();
  }
  const Eigen::Vector3d v_se_v = (p_04_v - p_02_v) / v_se_v_norm;
  const Eigen::Vector3d v_sw_v = (p_06 - p_02_v) / v_sw_v_norm;
  Eigen::Vector3d v_sew_v = v_se_v.cross(v_sw_v);
  const double sew_v_norm = v_sew_v.norm();
  if (sew_v_norm < 1e-9) {
    return fallback();
  }
  v_sew_v /= sew_v_norm;

  const Eigen::Vector3d v_se = (p_04 - p_02) / v_se_norm;
  const Eigen::Vector3d v_sw = (p_06 - p_02) / v_sw_norm;
  Eigen::Vector3d v_sew = v_se.cross(v_sw);
  const double sew_norm = v_sew.norm();
  if (sew_norm < 1e-9) {
    return fallback();
  }
  v_sew /= sew_norm;

  const Eigen::Vector3d axis = p_26 / p_26_norm;
  const double sin_psi = axis.dot(v_sew_v.cross(v_sew));
  const double cos_psi = std::clamp(v_sew_v.dot(v_sew), -1.0, 1.0);
  // 可观测性 = 两平面法向夹角的 sin（两个叉积都已归一化，故范数即 sin）；
  // 取虚拟/真实两者的较小者（任一病态则整体不可信）。
  return ArmAngleInfo{
    std::atan2(sin_psi, cos_psi), ArmAngleStatus::kOk,
    std::min(sew_v_norm, sew_norm)};
}

double computeArmAngleDh(
  const srs_dh_model::SrsGeometry & geometry,
  const std::array<double, srs_dh_model::kArmDoF> & theta_dh)
{
  return computeArmAngleDhInfo(geometry, theta_dh).psi;
}

ArmAngleInfo computeArmAngleMoveItQInfo(
  const srs_dh_model::SrsDhModel & model,
  const std::array<double, srs_dh_model::kArmDoF> & q_moveit)
{
  return computeArmAngleDhInfo(model.geometry(), model.moveitQToDhTheta(q_moveit));
}

double computeArmAngleMoveItQ(
  const srs_dh_model::SrsDhModel & model,
  const std::array<double, srs_dh_model::kArmDoF> & q_moveit)
{
  return computeArmAngleDhInfo(model.geometry(), model.moveitQToDhTheta(q_moveit)).psi;
}

Eigen::Matrix<double, 1, srs_dh_model::kArmDoF> armAngleJacobianMoveItQ(
  const srs_dh_model::SrsDhModel & model,
  const std::array<double, srs_dh_model::kArmDoF> & q_moveit,
  double delta_rad)
{
  Eigen::Matrix<double, 1, srs_dh_model::kArmDoF> J =
    Eigen::Matrix<double, 1, srs_dh_model::kArmDoF>::Zero();
  const double h = std::max(delta_rad, 1e-9);
  for (std::size_t i = 0; i < srs_dh_model::kArmDoF; ++i) {
    auto qp = q_moveit;
    auto qm = q_moveit;
    qp[i] += h;
    qm[i] -= h;
    const double phi_p = computeArmAngleMoveItQ(model, qp);
    const double phi_m = computeArmAngleMoveItQ(model, qm);
    J(0, static_cast<Eigen::Index>(i)) = wrapAngleRad(phi_p - phi_m) / (2.0 * h);
  }
  return J;
}

}  // namespace flex_qp_ik
