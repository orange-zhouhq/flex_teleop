// SPDX-License-Identifier: Apache-2.0
//
// 帧对齐插值器：30Hz 稀疏目标 -> 任意高频可采样目标。
//
// 设计要点（见 docs/02-frame-alignment.md）：
//   1. 双缓存 (A, B) + 固定延迟，输出 (id_a, id_b, alpha) 三元组；
//   2. 位置线性插值、姿态球面插值；
//   3. 只认 pose_id 判新旧，时间戳仅用于算 alpha；
//   4. 单线程使用（典型为 200Hz 定时器），无堆分配。
//
// alpha 语义：t_query = t_exec - fixed_delay 落在 [t_a, t_b] 内的比例。
// alpha 被钳在 [0, 1]；若 t_query > t_b 说明上游断流，fresh 置 false，
// 由上层转 HOLD——**不要**靠把 alpha 钳到 1 来"继续追目标"。
//
#pragma once

#include <cstdint>

#include "flex_teleop/types.hpp"

namespace flex_teleop {

/// 一次采样结果。
struct InterpSample {
  Pose T{};              ///< 插值后的目标位姿
  PoseId id_a{0};        ///< 区间左端
  PoseId id_b{0};        ///< 区间右端
  double alpha{0.0};     ///< [0, 1]
  bool valid{false};     ///< 至少有一帧可用
  bool fresh{false};     ///< 未超时（t_query <= t_b）
};

class Interpolator {
 public:
  /// @param fixed_delay_s 固定延迟（秒）。建议 >= 1.05 / 输入频率。
  explicit Interpolator(double fixed_delay_s);

  /// 推入一帧。
  /// @return false 表示被丢弃（id 非递增，即乱序或重复）
  bool push(const Pose& T, PoseId pose_id, std::int64_t t_arrival_ns);

  /// 在 t_exec_ns 时刻采样目标。
  InterpSample sample(std::int64_t t_exec_ns) const;

  bool hasTarget() const { return has_a_ || has_b_; }
  bool hasInterval() const { return has_a_ && has_b_; }

  PoseId lastId() const { return max_seen_; }
  double fixedDelaySec() const { return fixed_delay_ns_ * 1e-9; }

  void reset();

 private:
  struct Keyframe {
    Pose T{};
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
