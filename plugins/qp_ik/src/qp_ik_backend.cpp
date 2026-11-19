// SPDX-License-Identifier: BSD-3-Clause
#include "flex_teleop/qp_ik_backend.hpp"

namespace flex_teleop {
namespace {

Eigen::Isometry3d toEigen(const Pose& p) {
  Eigen::Matrix4d m;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      m(r, c) = p.m[r * 4 + c];
    }
  }
  return Eigen::Isometry3d(m);
}

void fromEigen(const Eigen::Isometry3d& T, Pose& out) {
  const Eigen::Matrix4d m = T.matrix();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out.m[r * 4 + c] = m(r, c);
    }
  }
}

}  // namespace

QpIkBackend::QpIkBackend(const Config& cfg)
    : ik_(cfg.geometry,
          cfg.one_shot ? flex_qp_ik::FlexQpIkMode::kOneShot
                       : flex_qp_ik::FlexQpIkMode::kServo,
          cfg.tick_s) {}

bool QpIkBackend::solve(const Pose& T_target, const double q_hint[kDof],
                        const double q_lower[kDof], const double q_upper[kDof],
                        double dt_s, double q_out[kDof]) {
  // 限位由几何在构造时注入，这里的两组参数仅用于满足 IKBackend 契约。
  (void)q_lower;
  (void)q_upper;
  (void)dt_s;

  flex_qp_ik::JointArray q_hint_arr{};
  for (int i = 0; i < kDof; ++i) {
    q_hint_arr[i] = q_hint[i];
  }

  const flex_qp_ik::FlexQpIkSolution sol = ik_.solve(toEigen(T_target), q_hint_arr);
  diag_ = sol.diag;

  if (!sol.ok) {
    return false;  // 调用方据此 HOLD
  }
  for (int i = 0; i < kDof; ++i) {
    q_out[i] = sol.q[i];
  }
  return true;
}

void QpIkBackend::fk(const double q[kDof], Pose& T_out) const {
  flex_qp_ik::JointArray q_arr{};
  for (int i = 0; i < kDof; ++i) {
    q_arr[i] = q[i];
  }
  fromEigen(ik_.model().fkMoveItQ(q_arr), T_out);
}

}  // namespace flex_teleop
