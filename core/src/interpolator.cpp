// SPDX-License-Identifier: Apache-2.0
#include "flex_teleop/interpolator.hpp"

#include "flex_teleop/pose_math.hpp"

namespace flex_teleop {
namespace {

inline double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

Interpolator::Interpolator(double fixed_delay_s)
    : fixed_delay_ns_(static_cast<std::int64_t>(fixed_delay_s * 1e9)) {
  if (fixed_delay_ns_ < 0) {
    fixed_delay_ns_ = 0;
  }
}

bool Interpolator::push(const Pose& T, PoseId pose_id, std::int64_t t_arrival_ns) {
  // 新帧判定只看 id：乱序或重复直接丢弃，不重排。
  if (has_b_ || has_a_) {
    if (pose_id <= max_seen_) {
      return false;
    }
  }

  // 时间单调保护：跨机时钟抖动可能让到达时刻倒退。
  std::int64_t t = t_arrival_ns;
  if (has_b_ && t <= b_.t_ns) {
    t = b_.t_ns + 1;  // 保证区间长度 > 0，避免除零
  }

  a_ = b_;
  has_a_ = has_b_;
  b_.T = T;
  b_.id = pose_id;
  b_.t_ns = t;
  has_b_ = true;
  max_seen_ = pose_id;
  return true;
}

InterpSample Interpolator::sample(std::int64_t t_exec_ns) const {
  InterpSample s;

  if (!has_a_ && !has_b_) {
    return s;  // valid=false
  }

  if (!has_a_) {
    // 只有首帧：以它为唯一目标，alpha=0。
    s.T = b_.T;
    s.id_a = b_.id;
    s.id_b = b_.id;
    s.alpha = 0.0;
    s.valid = true;
    s.fresh = (t_exec_ns - fixed_delay_ns_) <= b_.t_ns;
    return s;
  }

  // 有 A（无 B 时沿用 A 的时间，区间长度在下面单独处理）
  const std::int64_t t_query = t_exec_ns - fixed_delay_ns_;

  if (!has_b_) {
    s.T = a_.T;
    s.id_a = a_.id;
    s.id_b = a_.id;
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

  s.T = interpolatePose(a_.T, b_.T, alpha);
  s.id_a = a_.id;
  s.id_b = b_.id;
  s.alpha = alpha;
  s.valid = true;
  s.fresh = (t_query <= b_.t_ns);
  return s;
}

void Interpolator::reset() {
  a_ = Keyframe{};
  b_ = Keyframe{};
  has_a_ = false;
  has_b_ = false;
  max_seen_ = 0;
}

}  // namespace flex_teleop
