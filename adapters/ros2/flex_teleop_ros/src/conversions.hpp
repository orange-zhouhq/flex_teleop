// SPDX-License-Identifier: Apache-2.0
//
// ROS 2 消息 <-> 核心结构。只做搬运，不含任何算法逻辑。
//
#pragma once

#include <array>
#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <flex_teleop/types.hpp>
#include <flex_teleop_msgs/msg/joint_stream.hpp>
#include <flex_teleop_msgs/msg/teleop_status.hpp>

namespace flex_teleop_ros {

inline std::int64_t toNs(const builtin_interfaces::msg::Time& t) {
  return static_cast<std::int64_t>(t.sec) * 1000000000LL +
         static_cast<std::int64_t>(t.nanosec);
}

inline void toMsg(std::int64_t t_ns, builtin_interfaces::msg::Time& out) {
  out.sec = static_cast<std::int32_t>(t_ns / 1000000000LL);
  const std::int64_t rem = t_ns % 1000000000LL;
  out.nanosec = static_cast<std::uint32_t>(rem < 0 ? 0 : rem);
}

inline flex_teleop::Pose toPose(const std::array<double, 16>& m) {
  flex_teleop::Pose p;
  for (int i = 0; i < 16; ++i) {
    p.m[i] = m[i];
  }
  return p;
}

inline void fromArray7(const std::array<double, 7>& src, double dst[flex_teleop::kDof]) {
  for (int i = 0; i < flex_teleop::kDof; ++i) {
    dst[i] = src[i];
  }
}

inline void fromCore(const flex_teleop::JointStream& s, std::int64_t stamp_ns,
                     flex_teleop_msgs::msg::JointStream& out) {
  out.tick_seq = s.tick_seq;
  toMsg(stamp_ns, out.stamp);
  out.side = static_cast<std::uint8_t>(s.side);
  for (int i = 0; i < flex_teleop::kDof; ++i) {
    out.q_cmd[i] = s.q_cmd[i];
  }
  out.id_a = s.src.id_a;
  out.id_b = s.src.id_b;
  out.alpha = s.src.alpha;
  out.mode = static_cast<std::uint8_t>(s.mode);
  out.solver = static_cast<std::uint8_t>(s.solver);
}

inline void fromCore(const flex_teleop::TeleopStatus& st,
                     flex_teleop_msgs::msg::TeleopStatus& out) {
  toMsg(st.stamp_ns, out.stamp);
  out.side = static_cast<std::uint8_t>(st.side);
  out.state = static_cast<std::uint8_t>(st.state);
  out.mode = static_cast<std::uint8_t>(st.mode);
  out.pose_id_ack = st.pose_id_ack;
  out.id_a = st.src.id_a;
  out.id_b = st.src.id_b;
  out.alpha = st.src.alpha;
  out.e_model_pos_m = st.e_model.pos_m;
  out.e_model_rot_rad = st.e_model.rot_rad;
  out.e_real_pos_m = st.e_real.pos_m;
  out.e_real_rot_rad = st.e_real.rot_rad;
  for (int i = 0; i < flex_teleop::kDof; ++i) {
    out.lead_q[i] = st.lead_q[i];
  }
  out.solve_us = st.solve_us;
}

}  // namespace flex_teleop_ros
