// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：srs_dh_model.hpp
// 包角色：标准 DH 正运动学与 MoveIt 关节角 ↔ DH theta 映射接口
//
// 在 S-R-S 7DOF 臂栈中，SrsDhModel 是几何层对外的主要入口：
//   - fkDh：仅在 DH 坐标系内串联七节连杆（dh_base → dh_flange）；
//   - fkMoveItQ：先做 JointMapping，再套首尾固定变换，得到模型/规划系位姿；
//   - moveitQToDhTheta / dhThetaToMoveItQ：双向角映射（逆变换用 atan2 主值）；
//   - jacobianDh / jacobianMoveItQ：几何雅可比，分别与上述两个 FK 同参考系。
//
// 上游闭式 IK、臂角扫描、一致性校验均依赖本类与配置注入的 SrsGeometry。
// =============================================================================

#ifndef SRS_DH_MODEL__SRS_DH_MODEL_HPP_
#define SRS_DH_MODEL__SRS_DH_MODEL_HPP_

#include <array>

#include <Eigen/Geometry>

#include "srs_dh_model/srs_geometry.hpp"

namespace srs_dh_model
{

/// 持有一份 SrsGeometry，提供纯 DH FK 与 MoveIt 兼容 FK。
class SrsDhModel
{
public:
  /// 以几何参数构造模型（通常由 YAML/参数服务器填充后传入）。
  explicit SrsDhModel(SrsGeometry geometry);

  /// 只读访问内部几何（DH 参数、限位、映射、固定变换）。
  const SrsGeometry & geometry() const;

  /// 纯 DH 链正运动学：输入 DH 关节变量 theta_dh（rad），输出 dh_base → dh_flange。
  ///
  /// 计算：T = Π_{i=0..6} dhMatrix(theta_dh[i], alpha[i], a[i], d[i])
  /// 不含 T_tc_link_dh_base / T_dh_flange_link7。
  Eigen::Isometry3d fkDh(const std::array<double, kArmDoF> & theta_dh) const;

  /// MoveIt 关节角 → DH theta：
  ///   theta[i] = q_sign[i] * q_moveit[i] + q_offset[i]  （单位：rad）
  std::array<double, kArmDoF> moveitQToDhTheta(
    const std::array<double, kArmDoF> & q_moveit) const;

  /// DH theta → MoveIt 关节角（含主值回绕）：
  ///   先对 (theta - q_offset) 用 atan2(sin, cos) 折到 (-π, π]，
  ///   再乘 q_sign。这样避免 ±2π 漂移，并与符号约定一致。
  std::array<double, kArmDoF> dhThetaToMoveItQ(
    const std::array<double, kArmDoF> & theta_dh) const;

  /// 模型系正运动学（MoveIt q 输入）：
  ///   T = T_tc_link_dh_base * fkDh(moveitQToDhTheta(q)) * T_dh_flange_link7
  ///
  /// 与 fkDh 的区别：包含首尾固定链，输出直接落在规划/工具坐标系下的 link7 位姿。
  Eigen::Isometry3d fkMoveItQ(const std::array<double, kArmDoF> & q_moveit) const;

  /// 纯 DH 链几何雅可比（dh_base 系）。
  ///
  /// 6×7，行序 [v(0:3); ω(3:6)]，列 i 对应 ∂/∂theta_i，满足 [v; ω] = J · θ̇：
  ///   - v：**dh_flange 原点**的线速度（dh_base 系，m/s）；
  ///   - ω：dh_flange 的**空间角速度**（dh_base 系，rad/s，即 Ṙ·Rᵀ 的 vee）。
  ///
  /// 不含 T_tc_link_dh_base / T_dh_flange_link7，与 fkDh 同参考系、同参考点。
  /// 奇异位形下秩亏但元素有限；阻尼/正则化由调用方负责。
  Eigen::Matrix<double, 6, kArmDoF> jacobianDh(
    const std::array<double, kArmDoF> & theta_dh) const;

  /// 模型系几何雅可比（MoveIt q 输入），与 fkMoveItQ 同参考系、同参考点。
  ///
  /// 6×7，行序 [v(0:3); ω(3:6)]：
  ///   - v：**link7 原点**的线速度（模型/tc_link 系，m/s）；
  ///   - ω：link7 的**空间角速度**（模型/tc_link 系，rad/s）。
  ///
  /// ★ 角速度约定（**IK 对接时最易踩的坑**）：
  ///   ω 是空间（fixed-frame）角速度，与 log(R_tgt · R_curᵀ) 同系。
  ///   若姿态误差改用体坐标系形式 log(R_curᵀ · R_tgt)，则该误差向量需先左乘 R_curᵀ
  ///   才能与本雅可比的 ω 相加。
  ///
  /// 已含 q↔theta 链式缩放：theta_i = s_i·q_i + o_i ⇒ 列 i 乘 s_i。
  /// q_offset 是常值偏置，不影响雅可比。
  ///
  /// 参考点已取在 link7 原点，故 T_dh_flange_link7 的平移偏置已被正确计入
  /// （法兰原点与 link7 原点线速度不同，相差 ω × 偏置）。
  Eigen::Matrix<double, 6, kArmDoF> jacobianMoveItQ(
    const std::array<double, kArmDoF> & q_moveit) const;

private:
  /// 配置注入的几何与映射；构造后只读使用。
  SrsGeometry geometry_;
};

}  // namespace srs_dh_model

#endif  // SRS_DH_MODEL__SRS_DH_MODEL_HPP_
