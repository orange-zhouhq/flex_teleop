// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：srs_dh_model.cpp
// 包角色：SrsDhModel 成员实现——标准 DH FK 与 q↔theta 映射
//
// 本实现属于 S-R-S 7DOF 臂栈的几何执行层：
//   1) 用 internal::dhMatrix 按标准 DH 公式串联七节连杆；
//   2) JointMapping 连接 MoveIt 与 DH 零位/正方向；
//   3) fkMoveItQ 再左右乘固定变换，对齐 URDF / TCP 坐标系；
//   4) jacobianDh / jacobianMoveItQ 复用同一条链累乘产出 6×7 几何雅可比。
//
// 所有数值来自构造时的 SrsGeometry；此处无硬编码连杆长度。
// =============================================================================

#include "srs_dh_model/srs_dh_model.hpp"

#include <utility>

#include "srs_dh_model/dh_math.hpp"

namespace srs_dh_model
{

/// 接管几何参数所有权；后续 FK / 映射均只读引用 geometry_。
SrsDhModel::SrsDhModel(SrsGeometry geometry)
: geometry_(std::move(geometry))
{
}

const SrsGeometry & SrsDhModel::geometry() const
{
  return geometry_;
}

/// 纯 DH 正运动学：从 DH 基座到 DH 法兰。
///
/// 对 i = 0..6 累乘：
///   T ← T * Rot_z(θ_i) Trans_z(d_i) Trans_x(a_i) Rot_x(α_i)
/// 返回的 Isometry3d 矩阵等于该 4×4 齐次积（长度单位 m，角度 rad）。
Eigen::Isometry3d SrsDhModel::fkDh(
  const std::array<double, kArmDoF> & theta_dh) const
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (std::size_t i = 0; i < kArmDoF; ++i) {
    T = T * internal::dhMatrix(
      theta_dh[i], geometry_.alpha[i], geometry_.a[i], geometry_.d[i]);
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.matrix() = T;
  return out;
}

/// MoveIt → DH：线性映射，不做角度回绕（调用方负责合法关节范围）。
std::array<double, kArmDoF> SrsDhModel::moveitQToDhTheta(
  const std::array<double, kArmDoF> & q_moveit) const
{
  std::array<double, kArmDoF> theta{};
  for (std::size_t i = 0; i < kArmDoF; ++i) {
    // theta = s * q + offset；s∈{±1}，offset 为常值偏置（rad）。
    theta[i] = geometry_.mapping.q_sign[i] * q_moveit[i] + geometry_.mapping.q_offset[i];
  }
  return theta;
}

/// DH → MoveIt：先减去 offset，再用 atan2 取主值，最后乘回 q_sign。
///
/// 为何用 atan2(sin(·), cos(·))：
///   - 将任意实数角折到 (-π, π]，消除闭式解中 ±2π 等价解带来的数值跳变；
///   - 比简单 fmod 更稳健地处理负角与靠近 ±π 的边界。
/// 注意：当 q_sign = -1 时，主值角再取反，与正向映射互逆（在主值意义下）。
std::array<double, kArmDoF> SrsDhModel::dhThetaToMoveItQ(
  const std::array<double, kArmDoF> & theta_dh) const
{
  std::array<double, kArmDoF> q{};
  for (std::size_t i = 0; i < kArmDoF; ++i) {
    const double wrapped = std::atan2(
      std::sin(theta_dh[i] - geometry_.mapping.q_offset[i]),
      std::cos(theta_dh[i] - geometry_.mapping.q_offset[i]));
    q[i] = geometry_.mapping.q_sign[i] * wrapped;
  }
  return q;
}

/// 模型系 FK：固定基变换 × 纯 DH 链 × 固定法兰→link7 变换。
///
///   T_model_link7 = T_tc_link_dh_base * fkDh(θ(q)) * T_dh_flange_link7
///
/// 与 fkDh 相比，输出坐标系已对齐规划用的 tc / link7，可直接与 MoveIt 目标位姿比较。
Eigen::Isometry3d SrsDhModel::fkMoveItQ(
  const std::array<double, kArmDoF> & q_moveit) const
{
  return geometry_.T_tc_link_dh_base * fkDh(moveitQToDhTheta(q_moveit)) *
         geometry_.T_dh_flange_link7;
}

/// 纯 DH 链几何雅可比：与 fkDh 共用同一条累乘，参考点取法兰原点。
Eigen::Matrix<double, 6, kArmDoF> SrsDhModel::jacobianDh(
  const std::array<double, kArmDoF> & theta_dh) const
{
  internal::DhJointAxes axes;
  const Eigen::Matrix4d T_flange = internal::accumulateDhChain(
    theta_dh, geometry_.alpha, geometry_.a, geometry_.d, axes);

  // 参考点 = 法兰原点，与 fkDh 的输出位姿对齐。
  return internal::geometricJacobian(axes, T_flange.block<3, 1>(0, 3));
}

/// 模型系几何雅可比：先按 dh_base 系组装（参考点取 link7 原点），再换基、再套链式。
Eigen::Matrix<double, 6, kArmDoF> SrsDhModel::jacobianMoveItQ(
  const std::array<double, kArmDoF> & q_moveit) const
{
  internal::DhJointAxes axes;
  const Eigen::Matrix4d T_flange = internal::accumulateDhChain(
    moveitQToDhTheta(q_moveit), geometry_.alpha, geometry_.a, geometry_.d, axes);

  // ① 参考点必须取 **link7 原点**（= T_flange · T_dh_flange_link7 的平移），而不是
  //    法兰原点——两者线速度相差 ω × (T_dh_flange_link7 的平移)。取错会得到一个
  //    "位姿能对上、但速度对不上"的雅可比，在 T_dh_flange_link7 非单位阵时暴露。
  const Eigen::Matrix4d T_link7_dh_base = T_flange * geometry_.T_dh_flange_link7.matrix();
  Eigen::Matrix<double, 6, kArmDoF> J =
    internal::geometricJacobian(axes, T_link7_dh_base.block<3, 1>(0, 3));

  // ② 换基到模型系：v 与 ω 同为自由向量，同一个旋转变换即可。
  //    T_tc_link_dh_base 在 FK 里左乘，故其 linear() 正是"dh_base 分量 → 模型系分量"，
  //    直接左乘；转置会让所有列整体偏掉（首尾变换含大角度旋转时误差 O(1)）。
  internal::rotateJacobianToOuterFrame(J, geometry_.T_tc_link_dh_base.linear());

  // ③ q↔theta 链式：theta_i = s_i·q_i + o_i，故 dtheta_i/dq_i = s_i（±1）。
  for (std::size_t i = 0; i < kArmDoF; ++i) {
    J.col(i) *= geometry_.mapping.q_sign[i];
  }
  return J;
}

}  // namespace srs_dh_model
