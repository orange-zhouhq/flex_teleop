// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：proxqp_solver.hpp
// 包角色：QpSolver 的 ProxQP（proxsuite）后端
//
// 选型理由：BSD-2 许可（与本源码 BSD-3 兼容）、纯头文件（无运行时 .so，debian
// 打包只多一个 build_depend）、小规模稠密 QP 是它的主场、热启动友好。
// 实测 7 DOF 纯 box 约束 QP：首次 init+solve 约 29 μs，热启动平均约 9 μs。
//
// ProxSuite 是**纯头文件库**：CMake 里 `proxsuite::proxsuite` 是 INTERFACE target，
// 不需要链接任何库。本头文件用 Pimpl 把 proxsuite 挡在 .cpp 内，上层不必
// 引入它的 include 路径。
// =============================================================================

#ifndef FLEX_QP_IK__PROXQP_SOLVER_HPP_
#define FLEX_QP_IK__PROXQP_SOLVER_HPP_

#include <memory>

#include "flex_qp_ik/qp_solver.hpp"

namespace flex_qp_ik
{

/// ProxQP 后端。线程不安全，单实例串行复用。
class ProxQpSolver : public QpSolver
{
public:
  /// 求解器参数。默认值面向 200 Hz 伺服回路：容差够用、迭代上限小。
  struct Settings
  {
    /// 最大迭代数。7 DOF box 约束问题通常 2~5 次迭代收敛；给 100 是安全上限。
    int max_iterations{100};
    /// 绝对/相对精度容差。
    ///
    /// 取 1e-6 而非 1e-8：这是**速度级** QP，决策变量是 rad/s，1e-6 rad/s
    /// 远低于任何物理意义；而 1e-8 会让求解器在病态问题上耗尽迭代
    /// （MuJoCo 位姿遍历实测出现 PROXQP_MAX_ITER_REACHED 与数值退化）。
    /// 用一点点精度换稳健性，这笔交易划算。
    double eps_abs{1e-6};
    double eps_rel{1e-6};
    /// 是否记录内部计时（QpResult 的 *_time_us 依赖它，默认开）。
    bool compute_timings{true};
    /// 是否打开 proxsuite 自身日志（只在排查时开，会严重拖慢）。
    bool verbose{false};
  };

  ProxQpSolver();
  explicit ProxQpSolver(const Settings & settings);
  ~ProxQpSolver() override;

  ProxQpSolver(const ProxQpSolver &) = delete;
  ProxQpSolver & operator=(const ProxQpSolver &) = delete;

  void init(int n, int m_eq, int m_in) override;
  void reset() override;
  QpResult solve(const QpProblem & problem) override;
  const char * name() const override { return "proxqp"; }

  /// 修改参数。已 init 的实例会同步到内部求解器。
  void setSettings(const Settings & settings);

  /// 读取当前参数。
  const Settings & settings() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flex_qp_ik

#endif  // FLEX_QP_IK__PROXQP_SOLVER_HPP_
