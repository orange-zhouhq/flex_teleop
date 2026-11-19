// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：rotation_utils.hpp
// 包角色：SO(3) 对数映射与向量限幅（header-only，仅 Eigen）
//
// 为什么单独成文件：**姿态误差的参考系约定是 IK 里最容易踩的坑**。
//   - jacobianMoveItQ 的 ω 是**空间**角速度，即与 log(R_tgt · R_curᵀ) 同系；
//   - 若这里返回体坐标系形式 log(R_curᵀ · R_tgt)，结果不会报错，只会表现为
//     "越接近目标转得越怪"，且偏差随姿态差增大而增大。
//   所以这一层单独实现、单独测，并在命名与注释里钉死约定。
// =============================================================================

#ifndef FLEX_QP_IK__ROTATION_UTILS_HPP_
#define FLEX_QP_IK__ROTATION_UTILS_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace flex_qp_ik
{

/// sinθ 的判零阈值。低于它认为 R 近似为 ±I（θ ≈ 0 或 θ ≈ π），需走特判分支。
inline constexpr double kSinThetaEps = 1e-12;

/// SO(3) 对数映射：由旋转矩阵取**旋转向量**（轴×角，rad）。
///
/// 参考系约定：返回的是 **R 所在参考系下的空间旋转向量**。若 R = R_tgt · R_curᵀ，
/// 则返回向量可直接与 `SrsDhModel::jacobianMoveItQ` 的角速度行相加/相减。
///
/// 数值处理（θ → 0 与 θ → π 两个退化点都要稳）：
///   - 用 `atan2(‖s‖, cosθ)` 而非 `acos(cosθ)` 求角：小角下 acos 有灾难性抵消，
///     atan2 没有，θ 小到 1e-12 仍精确。
///   - `s = vee((R − Rᵀ)/2) = sinθ · axis` 在任意角度都稳定，常规路径用
///     `s · (θ/‖s‖)` 还原（θ→0 时比值 → 1）。
///   - θ ≈ π 时 s → 0，轴方向改从 `R + I` 恢复（该矩阵此时半正定、秩 1，
///     其主列即轴方向，只差一个正负号——π 附近符号本就病态，取正即可）。
inline Eigen::Vector3d rotationVector(const Eigen::Matrix3d & R)
{
  const Eigen::Vector3d s(
    (R(2, 1) - R(1, 2)) * 0.5,
    (R(0, 2) - R(2, 0)) * 0.5,
    (R(1, 0) - R(0, 1)) * 0.5);

  const double s_norm = s.norm();
  const double cos_theta = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::atan2(s_norm, cos_theta);  // ∈ [0, π]

  if (s_norm > kSinThetaEps) {
    return s * (theta / s_norm);
  }

  // sinθ ≈ 0：θ ≈ 0 或 θ ≈ π，靠 cosθ 的符号区分。
  if (cos_theta > 0.0) {
    return s;  // θ ≈ 0：此时 θ·axis 与 sinθ·axis 相对差仅 θ²/6，直接用 s
  }

  // θ ≈ π：轴从 R + I 恢复。取平方范数最大的列，避免除零。
  const Eigen::Matrix3d A = R + Eigen::Matrix3d::Identity();
  int best = 0;
  for (int c = 1; c < 3; ++c) {
    if (A.col(c).squaredNorm() > A.col(best).squaredNorm()) {
      best = c;
    }
  }
  const Eigen::Vector3d axis = A.col(best);
  const double axis_norm = axis.norm();
  if (axis_norm < kSinThetaEps) {
    return Eigen::Vector3d::Zero();  // R 非旋转阵（含 NaN）时的兜底
  }
  return axis * (theta / axis_norm);
}

/// 向量按模长限幅：‖v‖ > max_norm 时等比缩到 max_norm，否则原样返回。
/// max_norm <= 0 视为不限幅（用于"关闭该方向限幅"的配置）。
///
/// 按值传入传出而非 Eigen::Ref：调用点常传 `vec.head<3>()` 这类 block，
/// 返回值语义免去 block↔Ref 的转换约束，也避免别名歧义。
inline Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d & v, double max_norm)
{
  if (max_norm <= 0.0) {
    return v;
  }
  const double n = v.norm();
  if (n > max_norm && n > 0.0) {
    return v * (max_norm / n);
  }
  return v;
}

}  // namespace flex_qp_ik

#endif  // FLEX_QP_IK__ROTATION_UTILS_HPP_
