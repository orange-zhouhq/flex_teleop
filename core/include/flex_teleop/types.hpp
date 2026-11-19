// SPDX-License-Identifier: Apache-2.0
//
// flex_teleop 核心数据结构。
//
// 本文件是 docs/04-interface.md 的 C++ 落地，字段一一对应。
// 不依赖任何中间件、数学库或机器人模型；Pose 用裸 double[16] 表示，
// 便于跨语言绑定与零拷贝传输。
//
#pragma once

#include <cstdint>

namespace flex_teleop {

using PoseId = std::uint64_t;
using TickSeq = std::uint64_t;

/// 4x4 齐次变换，**行主序**：m[row * 4 + col]。
struct Pose {
  double m[16];
};

/// 机械臂侧别。
enum class Side : std::uint8_t { kLeft = 0, kRight = 1 };

/// 200Hz 关节流的工作模式。
enum class Mode : std::uint8_t {
  kPose = 0,   ///< 经 IK 求解
  kJoint = 1,  ///< 上层关节目标直通（旁路 IK）
  kHold = 2,   ///< 无新目标，保持上一拍
};

/// 本拍使用的求解后端。
enum class Solver : std::uint8_t {
  kQp = 0,
  kAnalytic = 1,
  kNumerical = 2,
  kHold = 3,
};

/// 遥操状态机。
enum class State : std::uint8_t {
  kTracking = 0,
  kHold = 1,
  kDegraded = 2,
  kFault = 3,
};

/// 关节自由度。
constexpr int kDof = 7;

/// 溯源三元组：本拍输出对应的上游 pose 区间与区间内比例。
///
/// 这是 flex_teleop 帧对齐的核心——单个 id 无法表达"两个源帧之间"，
/// 必须用 (id_a, id_b, alpha) 才能无损还原插值状态。
struct SrcTag {
  PoseId id_a{0};
  PoseId id_b{0};
  double alpha{0.0};  ///< [0, 1]
};

/// 框架输出 A：200Hz 关节流式目标。
struct JointStream {
  TickSeq tick_seq{0};
  Side side{Side::kLeft};
  double q_cmd[kDof]{};
  SrcTag src{};
  Mode mode{Mode::kHold};
  Solver solver{Solver::kHold};
};

/// 框架输入 C：关节反馈。
struct JointFeedback {
  TickSeq tick_seq{0};
  Side side{Side::kLeft};
  double q[kDof]{};
  double qd[kDof]{};
  double tau[kDof]{};  ///< 全 0 表示驱动器不提供力矩
};

/// 位姿误差（米 / 弧度）。
struct PoseError {
  double pos_m{0.0};
  double rot_rad{0.0};
};

/// 框架输出 B：状态上报（10–50Hz）。
///
/// 三个误差量必须分开：e_model 回答"算法对不对"，e_real 回答"系统到没到"，
/// lead_q 回答"伺服跟不跟"。只报 e_model 会把伺服滞后误判为跟踪良好。
struct TeleopStatus {
  std::int64_t stamp_ns{0};
  Side side{Side::kLeft};
  State state{State::kHold};
  Mode mode{Mode::kHold};
  PoseId pose_id_ack{0};   ///< 已完成下发的最大 id（不等于已执行到位）
  SrcTag src{};
  PoseError e_model{};     ///< r - FK(q_prev)
  PoseError e_real{};      ///< r - FK(q_fb)
  double lead_q[kDof]{};   ///< q_cmd - q_fb
  double solve_us{0.0};
};

}  // namespace flex_teleop
