// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：srs_geometry.hpp
// 包角色：S-R-S 7-DOF 手臂的标准 DH 几何与关节映射数据定义
//
// 在整机规划栈中，本文件描述“几何真相源”的数据结构：
//   - 连杆 DH 参数 (a, alpha, d) 与关节限位 (lower, upper) 全部由上层配置注入；
//   - 模型层不内置任何机器人硬编码长度/角度；
//   - JointMapping 衔接 MoveIt 关节角 q 与 DH 关节变量 theta；
//   - 首尾固定变换把 DH 链嵌入 URDF / 工具坐标系。
//
// 完整模型系正运动学链（列向量右乘）：
//   T_model_link7 = T_tc_link_dh_base * Π_{i=0..6} DH(θ_i) * T_dh_flange_link7
// =============================================================================

#ifndef SRS_DH_MODEL__SRS_GEOMETRY_HPP_
#define SRS_DH_MODEL__SRS_GEOMETRY_HPP_

#include <array>
#include <cstddef>

#include <Eigen/Geometry>

namespace srs_dh_model
{

/// 手臂主动自由度数目（S-R-S 冗余臂：7 个转动关节）。
inline constexpr std::size_t kArmDoF = 7;

/// MoveIt 关节角 q 与标准 DH 关节变量 theta 的线性映射。
///
/// 正向（MoveIt → DH）：
///   theta_dh[i] = q_sign[i] * q_moveit[i] + q_offset[i]
///
/// 说明：
///   - q_sign：±1，对齐 URDF 关节正方向与 DH 约定正方向；
///   - q_offset：常值偏置（rad），吸收“零位”定义差异（例如 URDF 零位 ≠ DH 零位）；
///   - 逆映射见 SrsDhModel::dhThetaToMoveItQ（含 atan2 主值回绕）。
struct JointMapping
{
  /// 各关节符号：通常为 +1 或 -1（无量纲）。
  std::array<double, kArmDoF> q_sign{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  /// 各关节偏置（rad）。
  std::array<double, kArmDoF> q_offset{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

/// S-R-S 臂的标准 DH 四参数集合，以及嵌入 URDF 的首尾固定变换。
///
/// 坐标系约定（示意）：
///   tool/control 系 --T_tc_link_dh_base--> DH 基座系
///       --Π DH(θ_i)--> DH 法兰系 --T_dh_flange_link7--> link7
///
/// 标准 DH 四参数对每个关节 i=0..6：
///   - a[i]：连杆长度（m）
///   - alpha[i]：连杆扭转（rad）
///   - d[i]：连杆偏距（m）；转动关节下为常量
///   关节变量 theta[i] 不存于此结构，由调用方在运行时提供。
struct SrsGeometry
{
  /// 连杆长度 a_i（m）。
  std::array<double, kArmDoF> a{};
  /// 连杆扭转 alpha_i（rad）。
  std::array<double, kArmDoF> alpha{};
  /// 连杆偏距 d_i（m）。
  std::array<double, kArmDoF> d{};
  /// 关节下限（MoveIt/URDF 关节角约定，rad）。
  std::array<double, kArmDoF> lower{};
  /// 关节上限（MoveIt/URDF 关节角约定，rad）。
  std::array<double, kArmDoF> upper{};
  /// q ↔ theta 映射参数。
  JointMapping mapping{};

  /// 从工具/控制参考系到 DH 基座系的固定刚体变换。
  /// 用于把闭式解/DH FK 结果对齐到规划用的 tc_link（或等价）坐标系。
  Eigen::Isometry3d T_tc_link_dh_base{Eigen::Isometry3d::Identity()};

  /// 从 DH 法兰系到 URDF link7 的固定刚体变换。
  /// 吸收 DH 法兰与 link7 之间的安装偏置；通常为单位阵。
  Eigen::Isometry3d T_dh_flange_link7{Eigen::Isometry3d::Identity()};
};

}  // namespace srs_dh_model

#endif  // SRS_DH_MODEL__SRS_GEOMETRY_HPP_
