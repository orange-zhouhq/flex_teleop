// SPDX-License-Identifier: BSD-3-Clause
//
// 参考 IK 后端：S-R-S 7 轴速度级 QP（flex_qp_ik）。
//
// 本适配层只做三件事：
//   1. Pose(double[16]) <-> Eigen::Isometry3d
//   2. double[7] <-> flex_qp_ik::JointArray
//   3. 把 FlexQpIkSolution 映射为 IKBackend 的 bool 契约
//
// 不引入任何机器人型号或标定假设——几何由使用方通过 Config 注入。
//
#pragma once

#include "flex_qp_ik/flex_qp_ik.hpp"
#include "flex_teleop/ik_backend.hpp"
#include "srs_dh_model/srs_dh_model.hpp"

namespace flex_teleop {

class QpIkBackend : public IKBackend {
 public:
  struct Config {
    /// S-R-S DH 几何。**必须已填 lower/upper**，否则构造函数抛异常。
    srs_dh_model::SrsGeometry geometry;

    /// 控制周期（秒）。kServo 模式下决定积分步长，200Hz → 5e-3。
    double tick_s{5e-3};

    /// true = kOneShot（离线求点，高成功率但最坏单帧数毫秒）
    /// false = kServo（200Hz 回路，每拍一次 QP + 一步积分）
    bool one_shot{false};
  };

  explicit QpIkBackend(const Config& cfg);

  bool solve(const Pose& T_target,
             const double q_hint[kDof],
             const double q_lower[kDof],
             const double q_upper[kDof],
             double dt_s,
             double q_out[kDof]) override;

  void fk(const double q[kDof], Pose& T_out) const override;

  Solver kind() const override { return Solver::kQp; }
  const char* name() const override { return "srs_qp_ik"; }

  /// 最近一次求解的诊断（迭代次数、耗时、是否撞上限等）。
  const flex_qp_ik::FlexQpIkDiagnostics& diagnostics() const { return diag_; }

 private:
  flex_qp_ik::FlexQpIk ik_;
  flex_qp_ik::FlexQpIkDiagnostics diag_{};
};

}  // namespace flex_teleop
