// SPDX-License-Identifier: Apache-2.0
//
// 关节空间插值器：与 Interpolator 同构（双缓存 + 固定延迟 + 三元组），
// 但作用于 kDof 维向量。用于 joint 模式（旁路 IK）。
//
#pragma once

#include <cstdint>

#include "flex_teleop/types.hpp"

namespace flex_teleop {

struct JointInterpSample {
  double q[kDof]{};
  PoseId id_a{0};
  PoseId id_b{0};
  double alpha{0.0};
  bool valid{false};
  bool fresh{false};
};

class JointInterpolator {
 public:
  explicit JointInterpolator(double fixed_delay_s);

  bool push(const double q[kDof], PoseId pose_id, std::int64_t t_arrival_ns);

  /// 模式切换时用当前关节角播种，绕过 id 单调检查。
  void seed(const double q[kDof], PoseId pose_id, std::int64_t t_ns);

  JointInterpSample sample(std::int64_t t_exec_ns) const;

  bool hasTarget() const { return has_a_ || has_b_; }
  PoseId lastId() const { return max_seen_; }
  void reset();

 private:
  struct Keyframe {
    double q[kDof]{};
    PoseId id{0};
    std::int64_t t_ns{0};
  };

  std::int64_t fixed_delay_ns_;
  Keyframe a_{};
  Keyframe b_{};
  bool has_a_{false};
  bool has_b_{false};
  PoseId max_seen_{0};
};

}  // namespace flex_teleop
