// SPDX-License-Identifier: Apache-2.0
//
// 双入口仲裁：pose 通路与 joint 通路共享同一个 pose_id 序列空间，
// 按最近一帧生效（id 大者胜出）。
//
// 仲裁规则见 docs/01-architecture.md「两个入口」。
// 模式切换时的平滑交接由 TeleopCore 负责，本类只做选择。
//
#pragma once

#include "flex_teleop/types.hpp"

namespace flex_teleop {

/// 仲裁输入帧。两个入口共用，靠 mode 区分。
struct MuxFrame {
  PoseId id{0};
  Mode mode{Mode::kHold};
  Pose T{};            ///< mode == kPose 时有效
  double q[kDof]{};    ///< mode == kJoint 时有效
  std::int64_t t_ns{0};
};

class SourceMux {
 public:
  /// 提交一帧。id 必须严格大于已接受的最后一帧，否则丢弃。
  bool submit(const MuxFrame& f) {
    if (has_ && f.id <= frame_.id) {
      return false;
    }
    frame_ = f;
    has_ = true;
    return true;
  }

  /// 取当前生效帧。
  bool current(MuxFrame& out) const {
    if (!has_) {
      return false;
    }
    out = frame_;
    return true;
  }

  bool has() const { return has_; }
  PoseId lastId() const { return frame_.id; }

  void reset() {
    frame_ = MuxFrame{};
    has_ = false;
  }

 private:
  MuxFrame frame_{};
  bool has_{false};
};

}  // namespace flex_teleop
