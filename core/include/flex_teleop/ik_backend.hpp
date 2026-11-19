// SPDX-License-Identifier: Apache-2.0
//
// IK 后端抽象。框架核心不包含任何具体 IK 算法。
//
// 实现方可以是：解析解、数值解、速度级 QP（如 proxsuite 系），
// 或任何外部 C ABI 库的薄封装。
//
// 硬性要求：
//   - solve() 在 200Hz 实时路径上调用，单次不得堆分配
//   - 不得读写全局状态，所有前值经参数传入
//   - 失败返回 false，由 TeleopCore 决定 HOLD
//
#pragma once

#include "flex_teleop/types.hpp"

namespace flex_teleop {

class IKBackend {
 public:
  virtual ~IKBackend() = default;

  /// 位置级求解。
  /// @param T_target  基座系下末端目标
  /// @param q_hint    关节初值（通常为上一拍解）
  /// @param q_lower   关节下限（rad）
  /// @param q_upper   关节上限（rad）
  /// @param dt_s      控制周期（供速度级后端使用；位置级可忽略）
  /// @param q_out     解（rad），仅当返回 true 时有效
  virtual bool solve(const Pose& T_target,
                     const double q_hint[kDof],
                     const double q_lower[kDof],
                     const double q_upper[kDof],
                     double dt_s,
                     double q_out[kDof]) = 0;

  /// 正运动学。用于计算 e_model / e_real。
  virtual void fk(const double q[kDof], Pose& T_out) const = 0;

  /// 后端类别，用于 JointStream.solver 上报。默认按 QP 处理。
  virtual Solver kind() const { return Solver::kQp; }

  virtual const char* name() const = 0;
};

}  // namespace flex_teleop
