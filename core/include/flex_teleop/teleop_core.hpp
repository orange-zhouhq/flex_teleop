// SPDX-License-Identifier: Apache-2.0
//
// 遥操核心编排：插值 → 仲裁 → IK → 200Hz 关节流。
//
// 调用约定：
//   - submitPose / submitJoint / submitFeedback 由各自的接收线程调用（非实时）
//   - tick() 由 200Hz 定时器调用，单次不得阻塞、不得堆分配
//   - 单实例单线程使用；跨线程需要外部加锁（推荐无锁队列 + 单线程消费）
//
// 本类不含任何机器人模型、坐标标定或中间件代码。
//
#pragma once

#include <cstdint>

#include "flex_teleop/ik_backend.hpp"
#include "flex_teleop/interpolator.hpp"
#include "flex_teleop/joint_interpolator.hpp"
#include "flex_teleop/source_mux.hpp"
#include "flex_teleop/types.hpp"

namespace flex_teleop {

class TeleopCore {
 public:
  struct Config {
    Side side{Side::kLeft};
    double output_rate_hz{200.0};
    double fixed_delay_s{0.035};
    int degraded_hold_frames{20};
    int fault_hold_frames{100};
  };

  explicit TeleopCore(const Config& cfg);

  /// IK 后端必须在使用前注入；未注入时 pose 模式恒为 HOLD。
  void setIK(IKBackend* ik) { ik_ = ik; }

  /// 关节限位（rad）。默认 [-pi, pi]。
  void setJointLimits(const double lower[kDof], const double upper[kDof]);

  /// 提交 pose 目标帧。返回 false 表示乱序/重复被丢弃。
  bool submitPose(const Pose& T, PoseId id, std::int64_t t_arrival_ns);

  /// 提交关节目标帧（旁路 IK）。与 pose 通路共享 id 序列空间。
  bool submitJoint(const double q[kDof], PoseId id, std::int64_t t_arrival_ns);

  /// 提交关节反馈。
  void submitFeedback(const JointFeedback& fb);

  /// 200Hz 节拍。返回本拍关节流式目标，必发。
  JointStream tick(std::int64_t t_now_ns);

  const TeleopStatus& status() const { return status_; }

 private:
  JointStream holdOutput(JointStream out, State st);
  bool solvePose(const Pose& T_target, double q_out[kDof]);

  Config cfg_;
  IKBackend* ik_{nullptr};

  Interpolator pose_interp_;
  JointInterpolator joint_interp_;
  SourceMux mux_;

  double q_lower_[kDof]{};
  double q_upper_[kDof]{};
  double q_prev_[kDof]{};
  double q_fb_[kDof]{};
  bool has_q_prev_{false};

  TickSeq tick_seq_{0};
  PoseId pose_id_ack_{0};
  SrcTag last_src_{};
  int hold_count_{0};

  TeleopStatus status_{};
};

}  // namespace flex_teleop
