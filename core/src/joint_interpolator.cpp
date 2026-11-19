// SPDX-License-Identifier: Apache-2.0
#include "flex_teleop/joint_interpolator.hpp"

namespace flex_teleop {
namespace {

inline double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void copyJoint(const double src[kDof], double dst[kDof]) {
  for (int i = 0; i < kDof; ++i) {
    dst[i] = src[i];
  }
}

}  // namespace

JointInterpolator::JointInterpolator(double fixed_delay_s)
    : fixed_delay_ns_(static_cast<std::int64_t>(fixed_delay_s * 1e9)) {
  if (fixed_delay_ns_ < 0) {
    fixed_delay_ns_ = 0;
  }
}

bool JointInterpolator::push(const double q[kDof], PoseId pose_id,
                             std::int64_t t_arrival_ns) {
  if ((has_a_ || has_b_) && pose_id <= max_seen_) {
    return false;
  }

  std::int64_t t = t_arrival_ns;
  if (has_b_ && t <= b_.t_ns) {
    t = b_.t_ns + 1;
  }

  a_ = b_;
  has_a_ = has_b_;
  copyJoint(q, b_.q);
  b_.id = pose_id;
  b_.t_ns = t;
  has_b_ = true;
  max_seen_ = pose_id;
  return true;
}

void JointInterpolator::seed(const double q[kDof], PoseId pose_id,
                             std::int64_t t_ns) {
  a_ = Keyframe{};
  b_ = Keyframe{};
  copyJoint(q, a_.q);
  a_.id = pose_id;
  a_.t_ns = t_ns;
  has_a_ = true;
  has_b_ = false;
  if (pose_id > max_seen_) {
    max_seen_ = pose_id;
  }
}

JointInterpSample JointInterpolator::sample(std::int64_t t_exec_ns) const {
  JointInterpSample s;

  if (!has_a_ && !has_b_) {
    return s;
  }

  const std::int64_t t_query = t_exec_ns - fixed_delay_ns_;

  if (!has_a_) {
    copyJoint(b_.q, s.q);
    s.id_a = s.id_b = b_.id;
    s.alpha = 0.0;
    s.valid = true;
    s.fresh = (t_query <= b_.t_ns);
    return s;
  }

  if (!has_b_) {
    copyJoint(a_.q, s.q);
    s.id_a = s.id_b = a_.id;
    s.alpha = 0.0;
    s.valid = true;
    s.fresh = (t_query <= a_.t_ns);
    return s;
  }

  const std::int64_t span = b_.t_ns - a_.t_ns;
  double alpha = 0.0;
  if (span > 0) {
    alpha = static_cast<double>(t_query - a_.t_ns) / static_cast<double>(span);
  } else if (t_query >= b_.t_ns) {
    alpha = 1.0;
  }
  alpha = clampd(alpha, 0.0, 1.0);

  for (int i = 0; i < kDof; ++i) {
    s.q[i] = a_.q[i] + alpha * (b_.q[i] - a_.q[i]);
  }
  s.id_a = a_.id;
  s.id_b = b_.id;
  s.alpha = alpha;
  s.valid = true;
  s.fresh = (t_query <= b_.t_ns);
  return s;
}

void JointInterpolator::reset() {
  a_ = Keyframe{};
  b_ = Keyframe{};
  has_a_ = false;
  has_b_ = false;
  max_seen_ = 0;
}

}  // namespace flex_teleop
