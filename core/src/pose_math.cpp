// SPDX-License-Identifier: Apache-2.0
#include "flex_teleop/pose_math.hpp"

#include <cmath>

namespace flex_teleop {
namespace {

constexpr double kPi = 3.14159265358979323846;

inline double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

Pose identity() {
  Pose T{};
  for (int i = 0; i < 4; ++i) {
    T.m[i * 4 + i] = 1.0;
  }
  return T;
}

Pose makePose(const double p[3], const double quat_xyzw[4]) {
  return quaternionToPose(quat_xyzw, p);
}

void poseTranslation(const Pose& T, double out[3]) {
  out[0] = T.m[3];
  out[1] = T.m[7];
  out[2] = T.m[11];
}

void normalizeQuaternion(double q[4]) {
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-12) {
    const double inv = 1.0 / n;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
  } else {
    q[0] = q[1] = q[2] = 0.0;
    q[3] = 1.0;
  }
}

void poseToQuaternion(const Pose& T, double out[4]) {
  const double m00 = T.m[0], m01 = T.m[1], m02 = T.m[2];
  const double m10 = T.m[4], m11 = T.m[5], m12 = T.m[6];
  const double m20 = T.m[8], m21 = T.m[9], m22 = T.m[10];

  const double trace = m00 + m11 + m22;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;  // s = 4w
    out[3] = 0.25 * s;
    out[0] = (m21 - m12) / s;
    out[1] = (m02 - m20) / s;
    out[2] = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;  // s = 4x
    out[3] = (m21 - m12) / s;
    out[0] = 0.25 * s;
    out[1] = (m01 + m10) / s;
    out[2] = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;  // s = 4y
    out[3] = (m02 - m20) / s;
    out[0] = (m01 + m10) / s;
    out[1] = 0.25 * s;
    out[2] = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;  // s = 4z
    out[3] = (m10 - m01) / s;
    out[0] = (m02 + m20) / s;
    out[1] = (m12 + m21) / s;
    out[2] = 0.25 * s;
  }
  normalizeQuaternion(out);
}

Pose quaternionToPose(const double quat_xyzw[4], const double p[3]) {
  double q[4] = {quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]};
  normalizeQuaternion(q);

  const double px = (p != nullptr) ? p[0] : 0.0;
  const double py = (p != nullptr) ? p[1] : 0.0;
  const double pz = (p != nullptr) ? p[2] : 0.0;

  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double xx = x * x, yy = y * y, zz = z * z;
  const double xy = x * y, xz = x * z, yz = y * z;
  const double wx = w * x, wy = w * y, wz = w * z;

  Pose T{};
  T.m[0] = 1.0 - 2.0 * (yy + zz);
  T.m[1] = 2.0 * (xy - wz);
  T.m[2] = 2.0 * (xz + wy);
  T.m[3] = px;

  T.m[4] = 2.0 * (xy + wz);
  T.m[5] = 1.0 - 2.0 * (xx + zz);
  T.m[6] = 2.0 * (yz - wx);
  T.m[7] = py;

  T.m[8] = 2.0 * (xz - wy);
  T.m[9] = 2.0 * (yz + wx);
  T.m[10] = 1.0 - 2.0 * (xx + yy);
  T.m[11] = pz;

  T.m[12] = 0.0;
  T.m[13] = 0.0;
  T.m[14] = 0.0;
  T.m[15] = 1.0;
  return T;
}

void slerp(const double qa[4], const double qb[4], double t, double out[4]) {
  double a[4] = {qa[0], qa[1], qa[2], qa[3]};
  double b[4] = {qb[0], qb[1], qb[2], qb[3]};
  normalizeQuaternion(a);
  normalizeQuaternion(b);

  double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];

  // 符号一致性：q 与 -q 表示同一旋转，走短弧。
  if (dot < 0.0) {
    b[0] = -b[0];
    b[1] = -b[1];
    b[2] = -b[2];
    b[3] = -b[3];
    dot = -dot;
  }

  t = clamp(t, 0.0, 1.0);

  if (dot > 0.9995) {
    // 近似共线：线性插值后归一化，避免 sin 除零。
    for (int i = 0; i < 4; ++i) {
      out[i] = a[i] + t * (b[i] - a[i]);
    }
    normalizeQuaternion(out);
    return;
  }

  const double theta_0 = std::acos(dot);
  const double theta = theta_0 * t;
  const double sin_theta = std::sin(theta);
  const double sin_theta_0 = std::sin(theta_0);

  const double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
  const double s1 = sin_theta / sin_theta_0;

  for (int i = 0; i < 4; ++i) {
    out[i] = s0 * a[i] + s1 * b[i];
  }
  normalizeQuaternion(out);
}

Pose interpolatePose(const Pose& Ta, const Pose& Tb, double alpha) {
  const double t = clamp(alpha, 0.0, 1.0);

  double pa[3], pb[3], pc[3];
  poseTranslation(Ta, pa);
  poseTranslation(Tb, pb);
  for (int i = 0; i < 3; ++i) {
    pc[i] = pa[i] + t * (pb[i] - pa[i]);
  }

  double qa[4], qb[4], qc[4];
  poseToQuaternion(Ta, qa);
  poseToQuaternion(Tb, qb);
  slerp(qa, qb, t, qc);

  return quaternionToPose(qc, pc);
}

double rotationAngle(const Pose& Ta, const Pose& Tb) {
  double qa[4], qb[4];
  poseToQuaternion(Ta, qa);
  poseToQuaternion(Tb, qb);
  double dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
  dot = clamp(std::fabs(dot), -1.0, 1.0);
  return 2.0 * std::acos(dot);
}

double translationDistance(const Pose& Ta, const Pose& Tb) {
  double pa[3], pb[3];
  poseTranslation(Ta, pa);
  poseTranslation(Tb, pb);
  const double dx = pa[0] - pb[0];
  const double dy = pa[1] - pb[1];
  const double dz = pa[2] - pb[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace flex_teleop
