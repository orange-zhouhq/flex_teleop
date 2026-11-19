// SPDX-License-Identifier: Apache-2.0
#include "flex_teleop/teleop_core.hpp"

#include <chrono>
#include <cmath>

#include "flex_teleop/pose_math.hpp"

namespace flex_teleop {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

TeleopCore::TeleopCore(const Config& cfg)
    : cfg_(cfg),
      pose_interp_(cfg.fixed_delay_s),
      joint_interp_(cfg.fixed_delay_s) {
  for (int i = 0; i < kDof; ++i) {
    q_lower_[i] = -kPi;
    q_upper_[i] = kPi;
    q_prev_[i] = 0.0;
    q_fb_[i] = 0.0;
  }
  status_.side = cfg_.side;
  status_.state = State::kHold;
  status_.mode = Mode::kHold;
}

void TeleopCore::setJointLimits(const double lower[kDof], const double upper[kDof]) {
  for (int i = 0; i < kDof; ++i) {
    q_lower_[i] = lower[i];
    q_upper_[i] = upper[i];
  }
}

bool TeleopCore::submitPose(const Pose& T, PoseId id, std::int64_t t_arrival_ns) {
  if (!pose_interp_.push(T, id, t_arrival_ns)) {
    return false;
  }
  MuxFrame f;
  f.id = id;
  f.mode = Mode::kPose;
  f.T = T;
  f.t_ns = t_arrival_ns;
  return mux_.submit(f);
}

bool TeleopCore::submitJoint(const double q[kDof], PoseId id,
                             std::int64_t t_arrival_ns) {
  if (!joint_interp_.push(q, id, t_arrival_ns)) {
    return false;
  }
  MuxFrame f;
  f.id = id;
  f.mode = Mode::kJoint;
  for (int i = 0; i < kDof; ++i) {
    f.q[i] = q[i];
  }
  f.t_ns = t_arrival_ns;
  return mux_.submit(f);
}

void TeleopCore::submitFeedback(const JointFeedback& fb) {
  for (int i = 0; i < kDof; ++i) {
    q_fb_[i] = fb.q[i];
  }
}

bool TeleopCore::solvePose(const Pose& T_target, double q_out[kDof]) {
  if (ik_ == nullptr) {
    return false;
  }
  const double dt = 1.0 / (cfg_.output_rate_hz > 0.0 ? cfg_.output_rate_hz : 200.0);
  return ik_->solve(T_target, q_prev_, q_lower_, q_upper_, dt, q_out);
}

JointStream TeleopCore::holdOutput(JointStream out, State st) {
  ++hold_count_;
  if (hold_count_ > cfg_.fault_hold_frames) {
    st = State::kFault;
  } else if (hold_count_ > cfg_.degraded_hold_frames) {
    st = State::kDegraded;
  }

  out.mode = Mode::kHold;
  out.solver = Solver::kHold;
  out.src = last_src_;  // 冻结在最后成功的那一帧
  for (int i = 0; i < kDof; ++i) {
    out.q_cmd[i] = q_prev_[i];
  }

  status_.state = st;
  status_.mode = Mode::kHold;
  status_.src = last_src_;
  return out;
}

JointStream TeleopCore::tick(std::int64_t t_now_ns) {
  JointStream out;
  out.tick_seq = ++tick_seq_;
  out.side = cfg_.side;
  status_.stamp_ns = t_now_ns;
  status_.side = cfg_.side;

  MuxFrame f;
  if (!mux_.current(f)) {
    return holdOutput(out, State::kHold);
  }

  if (f.mode == Mode::kJoint) {
    const JointInterpSample s = joint_interp_.sample(t_now_ns);
    if (!s.valid || !s.fresh) {
      return holdOutput(out, State::kHold);
    }
    hold_count_ = 0;
    for (int i = 0; i < kDof; ++i) {
      q_prev_[i] = s.q[i];
      out.q_cmd[i] = s.q[i];
    }
    has_q_prev_ = true;
    out.mode = Mode::kJoint;
    out.solver = Solver::kHold;  // 未经求解
    out.src = SrcTag{s.id_a, s.id_b, s.alpha};
    last_src_ = out.src;

    status_.state = State::kTracking;
    status_.mode = Mode::kJoint;
    status_.src = out.src;
    if (s.id_b > pose_id_ack_) {
      pose_id_ack_ = s.id_b;
    }
    status_.pose_id_ack = pose_id_ack_;
    return out;
  }

  // pose 模式
  const InterpSample s = pose_interp_.sample(t_now_ns);
  if (!s.valid || !s.fresh) {
    return holdOutput(out, State::kHold);
  }

  const auto t0 = std::chrono::steady_clock::now();
  double q_sol[kDof];
  const bool ok = solvePose(s.T, q_sol);
  const auto t1 = std::chrono::steady_clock::now();
  const double us =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() * 1e-3;
  status_.solve_us = us;

  if (!ok) {
    return holdOutput(out, State::kHold);
  }

  hold_count_ = 0;
  for (int i = 0; i < kDof; ++i) {
    q_prev_[i] = q_sol[i];
    out.q_cmd[i] = q_sol[i];
  }
  has_q_prev_ = true;
  out.mode = Mode::kPose;
  out.solver = (ik_ != nullptr) ? ik_->kind() : Solver::kHold;
  out.src = SrcTag{s.id_a, s.id_b, s.alpha};
  last_src_ = out.src;

  status_.state = State::kTracking;
  status_.mode = Mode::kPose;
  status_.src = out.src;
  if (s.id_b > pose_id_ack_) {
    pose_id_ack_ = s.id_b;
  }
  status_.pose_id_ack = pose_id_ack_;

  // 误差：e_model 用 q_prev（本轮解），e_real 用真实反馈
  if (ik_ != nullptr) {
    Pose T_model{};
    Pose T_real{};
    ik_->fk(q_prev_, T_model);
    ik_->fk(q_fb_, T_real);
    status_.e_model.pos_m = translationDistance(s.T, T_model);
    status_.e_model.rot_rad = rotationAngle(s.T, T_model);
    status_.e_real.pos_m = translationDistance(s.T, T_real);
    status_.e_real.rot_rad = rotationAngle(s.T, T_real);
    for (int i = 0; i < kDof; ++i) {
      status_.lead_q[i] = q_prev_[i] - q_fb_[i];
    }
  }

  return out;
}

}  // namespace flex_teleop
