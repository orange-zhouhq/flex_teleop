// SPDX-License-Identifier: Apache-2.0
//
// SE(3) 基础运算。零依赖实现，供 Interpolator 与测试使用。
//
// 约定：Pose 为行主序 4x4 齐次矩阵；四元数按 (x, y, z, w) 存。
//
#pragma once

#include "flex_teleop/types.hpp"

namespace flex_teleop {

/// 单位阵。
Pose identity();

/// 由位置与四元数构造。
Pose makePose(const double p[3], const double quat_xyzw[4]);

/// 取出平移。
void poseTranslation(const Pose& T, double out[3]);

/// 4x4 -> 四元数 (x, y, z, w)。旋转部分取左上 3x3。
void poseToQuaternion(const Pose& T, double out_xyzw[4]);

/// 四元数 -> 4x4。
Pose quaternionToPose(const double quat_xyzw[4], const double p[3]);

/// 归一化四元数（原地）。
void normalizeQuaternion(double q[4]);

/// 单位四元数球面插值。t in [0, 1]。
///
/// 内部处理符号一致性：若点积为负，翻转其一，避免走"长弧"。
/// 点积接近 1 时退化为线性插值 + 归一化（数值稳定）。
void slerp(const double qa[4], const double qb[4], double t, double out[4]);

/// SE(3) 插值：平移线性、旋转球面。
Pose interpolatePose(const Pose& Ta, const Pose& Tb, double alpha);

/// 两个姿态的旋转夹角（rad），用于残差与测试。
double rotationAngle(const Pose& Ta, const Pose& Tb);

/// 两个姿态的平移距离（m）。
double translationDistance(const Pose& Ta, const Pose& Tb);

}  // namespace flex_teleop
