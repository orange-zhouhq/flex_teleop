// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：dh_math.hpp
// 包角色：srs_dh_model 内部数学工具（header-only）
//
// 本包位于 S-R-S（Shoulder–Roll–Shoulder）7 自由度机械臂规划栈的底层：
//   - 向上为闭式 IK / 臂角参数化 / MoveIt 插件提供统一的标准 DH 齐次变换；
//   - 本文件不依赖具体连杆数值，只提供可复用的三角与矩阵原语。
//
// 内容概览：
//   - 圆周率常量、acos 数值保护常量；
//   - 标准 DH 单连杆齐次矩阵；
//   - 向量叉积反对称矩阵；
//   - 安全 acos（肘部伸直/折叠奇异边界）；
//   - DH 链一次累乘（同时产出各关节轴）与几何雅可比组装。
//
// 雅可比为何在此：L0 层原本只有 FK，解析雅可比长期缺失，导致数值 IK 只能靠
// 有限差分、力控只能退化为"位姿级左乘"而无法做关节级/阻抗级合成。此处补上
// 纯几何原语，供数值 IK、QP IK 与力控共用；本文件仍不依赖任何连杆数值。
// =============================================================================

#ifndef SRS_DH_MODEL__DH_MATH_HPP_
#define SRS_DH_MODEL__DH_MATH_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Geometry>

#include "srs_dh_model/srs_geometry.hpp"

namespace srs_dh_model
{
namespace internal
{

/// π（弧度）。DH 角、关节角、臂角均以弧度为单位。
inline constexpr double kPi = 3.14159265358979323846;

/// 将 acos 输入从 ±1 向内回退的步长，避免浮点越界导致 NaN。
inline constexpr double kClipStepback = 1e-4;

/// 标准 Denavit–Hartenberg（Craig/经典形式）单连杆齐次变换。
///
/// 公式（从关节 i 坐标系到关节 i+1，列向量右乘约定）：
///   T_i^{i+1} = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
///
/// 参数（单位）：
///   - theta：关节变量（绕 z_i，rad）
///   - alpha：连杆扭转角（绕 x_{i+1}，rad）
///   - a：连杆长度（沿 x_{i+1}，m）
///   - d：连杆偏距（沿 z_i，m）
///
/// 返回 4×4 齐次矩阵，表示在父坐标系下子坐标系的位姿。
inline Eigen::Matrix4d dhMatrix(double theta, double alpha, double a, double d)
{
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  // 旋转子块 R 与平移 [a*ct, a*st, d]^T，与上述标准 DH 展开一致。
  T(0, 0) = ct;
  T(0, 1) = -st * ca;
  T(0, 2) = st * sa;
  T(0, 3) = a * ct;
  T(1, 0) = st;
  T(1, 1) = ct * ca;
  T(1, 2) = -ct * sa;
  T(1, 3) = a * st;
  T(2, 0) = 0.0;
  T(2, 1) = sa;
  T(2, 2) = ca;
  T(2, 3) = d;
  return T;
}

/// 三维向量 a 的叉积反对称矩阵 [a]_×，满足 [a]_× b = a × b。
/// 用于闭式解中构造旋转轴相关的线性代数（单位：与 a 一致，通常为 m 或无量纲）。
inline Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d & a)
{
  Eigen::Matrix3d A;
  A << 0.0, -a.z(), a.y(),
    a.z(), 0.0, -a.x(),
    -a.y(), a.x(), 0.0;
  return A;
}

/// 将 x 钳制到 (-1+ε, 1-ε)，供 std::acos 安全调用；ε = kClipStepback。
inline double clampAcosInput(double x)
{
  return std::clamp(x, -1.0 + kClipStepback, 1.0 - kClipStepback);
}

/// 允许微小越界的 acos，用于 S-R-S 肘部完全伸直/折叠奇异边界。
///
/// 若 |x| 超出 1±eps（eps=1e-3），视为几何上不可达，返回 false；
/// 否则将 x 钳制后取 acos，结果 y 写入输出参数（单位：rad）。
inline bool safeAcos(double x, double & y)
{
  constexpr double eps = 1e-3;
  if (x < -1.0 - eps || x > 1.0 + eps) {
    return false;
  }
  y = std::acos(clampAcosInput(x));
  return true;
}

/// 一次 DH 链累乘中记录的关节轴集合（均在 dh_base 系）。
///
/// 标准 DH 下关节 i 的转轴就是 T_i 的 z 轴、且过 T_i 原点，于是：
///   z[i] = R(T_i) · e_z，p[i] = translation(T_i)。
/// 注意连杆偏距 d_i 是沿同一 z 轴平移（Trans_z(d_i) 在 Rot_z 之后），
/// 不改变轴线方向也不移动轴线，故取累乘**前**的 T_i 即正确。
struct DhJointAxes
{
  /// 关节 i 的转轴单位方向（dh_base 系）。
  std::array<Eigen::Vector3d, kArmDoF> z{};
  /// 关节 i 转轴上的一点（dh_base 系）。
  std::array<Eigen::Vector3d, kArmDoF> p{};
};

/// 累乘 DH 链，输出各关节轴与末端（dh_flange）位姿。
///
/// 对 i = 0..6 依次 T ← T · DH(θ_i, α_i, a_i, d_i)；每步累乘**前**把当前 T 的
/// z 轴与原点登记为关节 i 的轴线。返回 T_7，即 dh_base → dh_flange 的 4×4。
///
/// 与 fkDh 走同一条链，因此调用方若同时需要 FK 与雅可比，可用本函数一次拿到，
/// 避免重复累乘（200 Hz 回路里 FK+J 每 tick 都要算）。
inline Eigen::Matrix4d accumulateDhChain(
  const std::array<double, kArmDoF> & theta,
  const std::array<double, kArmDoF> & alpha,
  const std::array<double, kArmDoF> & a,
  const std::array<double, kArmDoF> & d,
  DhJointAxes & axes_out)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (std::size_t i = 0; i < kArmDoF; ++i) {
    axes_out.z[i] = T.block<3, 1>(0, 2);
    axes_out.p[i] = T.block<3, 1>(0, 3);
    T = T * dhMatrix(theta[i], alpha[i], a[i], d[i]);
  }
  return T;
}

/// 由关节轴集合与线速度参考点组装几何雅可比（dh_base 系）。
///
///   J.col(i) = [ z_i × (p_ref − p_i) ; z_i ]
///
/// 语义：[ v(p_ref) ; ω_flange ] = J · θ̇，其中
///   - v(p_ref)：参考点 p_ref 的线速度（dh_base 系）——**注意线速度与参考点绑定**，
///     取 T_7 原点得法兰原点速度，取 T_7·T_dh_flange_link7 原点得 link7 原点速度，
///     两者不同（相差 ω × 偏置）；
///   - ω_flange：法兰的**空间角速度**（dh_base 系，即 Ṙ·Rᵀ 的 vee），与参考点无关。
///
/// 奇异位形下 J 秩亏但各元素仍有限；阻尼/正则化由调用方负责，本函数不做保护。
inline Eigen::Matrix<double, 6, kArmDoF> geometricJacobian(
  const DhJointAxes & axes, const Eigen::Vector3d & p_ref)
{
  Eigen::Matrix<double, 6, kArmDoF> J;
  for (std::size_t i = 0; i < kArmDoF; ++i) {
    J.block<3, 1>(0, i) = axes.z[i].cross(p_ref - axes.p[i]);
    J.block<3, 1>(3, i) = axes.z[i];
  }
  return J;
}

/// 把以 dh_base 系表达的几何雅可比旋到外层的目标系（如模型/tc_link 系）。
///
/// v 与 ω 都是自由向量（不绑定原点），换系只是分量旋转：
///   [v'; ω'] = diag(R, R) · [v; ω]
///
/// ★ 传入的必须是 `T_外层_dh_base.linear()`，即**把 dh_base 系分量映到外层系分量**的
///   旋转，与 FK 串联方向一致（T_model = T_外层_dh_base · fkDh · …）。**不要传转置**：
///   方向搞反不会报错，只会得到一个"位姿对得上、速度全错"的雅可比，
///   偏差量级 O(1)（首尾变换含大角度旋转时尤其明显）。
///   故本函数刻意按"外层 ← dh_base"命名参数与函数，避免双向解释。
///
/// **线速度无需额外的参考点修正**——参考点已在 dh_base 系取对，这里只换分量基。
inline void rotateJacobianToOuterFrame(
  Eigen::Matrix<double, 6, kArmDoF> & J, const Eigen::Matrix3d & R_outer_from_dh_base)
{
  const Eigen::Matrix<double, 3, kArmDoF> Jv = R_outer_from_dh_base * J.topRows<3>();
  const Eigen::Matrix<double, 3, kArmDoF> Jw = R_outer_from_dh_base * J.bottomRows<3>();
  J.topRows<3>() = Jv;
  J.bottomRows<3>() = Jw;
}

}  // namespace internal
}  // namespace srs_dh_model

#endif  // SRS_DH_MODEL__DH_MATH_HPP_
