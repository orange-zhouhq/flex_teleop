// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：proxqp_solver.cpp
// 包角色：QpSolver 的 ProxQP（proxsuite）后端实现
//
// 两处**必须**遵守的 ProxSuite 约束（实测确认，违反会抛异常而非静默出错）：
//   1) H 必须**逐位对称**。Model::is_valid 用
//      `H.isApprox(H.transpose(), machine_epsilon)` 判定，相对容差 2.2e-16，
//      即"双边同一累加顺序"级别。下三角置零、或 1e-8 量级的不对称，都会直接抛
//      std::invalid_argument。故本后端在装配前显式对称化。
//   2) 装配用 box_constraints=true 的 init/update 重载（本包所有约束都是简单边界，
//      正好走 ProxQP 的 box 快路径）。l_box/u_box 允许 nullopt。
//
// ★ 实时安全：ProxSuite 用异常报错。本后端的 solve() 全程 try/catch，把异常转成
//   kNumericalError 返回，**绝不向 200 Hz 定时器回调抛出**——上层设计原则是
//   "宁可停住也不跳变"，不允许求解器异常打断控制回路。
// =============================================================================

#include "flex_qp_ik/proxqp_solver.hpp"

#include <exception>
#include <string>
#include <utility>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace flex_qp_ik
{
namespace
{

using ProxQp = proxsuite::proxqp::dense::QP<double>;
using proxsuite::nullopt;
using proxsuite::optional;

/// ProxSuite 状态 → 本包公共状态。
QpStatus mapStatus(proxsuite::proxqp::QPSolverOutput s)
{
  using proxsuite::proxqp::QPSolverOutput;
  switch (s) {
    case QPSolverOutput::PROXQP_SOLVED:
      return QpStatus::kSolved;
    case QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
      // 原问题不可行，返回的是 L2 意义下最接近可行的点。它不是满足约束的解，
      // 对 IK 而言不可直接使用，故与"不可行"同等对待，由上层决定降级策略。
      return QpStatus::kPrimalInfeasible;
    case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
      return QpStatus::kPrimalInfeasible;
    case QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
      return QpStatus::kDualInfeasible;
    case QPSolverOutput::PROXQP_MAX_ITER_REACHED:
      return QpStatus::kMaxIterations;
    case QPSolverOutput::PROXQP_NOT_RUN:
    default:
      return QpStatus::kUnknown;
  }
}

/// 空矩阵/向量 → nullopt；非空 → 引用。空用 0 尺寸表示（见 QpProblem 约定）。
template <typename T>
optional<T> optOf(const T & v)
{
  if (v.size() == 0) {
    return nullopt;
  }
  return optional<T>(v);
}

}  // namespace

struct ProxQpSolver::Impl
{
  ProxQpSolver::Settings settings{};
  std::unique_ptr<ProxQp> qp{};
  int n{0};
  int m_eq{0};
  int m_in{0};
  /// 首次 solve 走完整 init（建立预处理缓存），其后走 update（保留热启动）。
  bool warmed_up{false};
  /// 对称化缓冲（n 很小，7~14 阶，复用避免每帧分配）。
  Eigen::MatrixXd h_sym{};
};

ProxQpSolver::ProxQpSolver()
: impl_(std::make_unique<Impl>())
{
}

ProxQpSolver::ProxQpSolver(const Settings & settings)
: impl_(std::make_unique<Impl>())
{
  setSettings(settings);
}

ProxQpSolver::~ProxQpSolver() = default;

void ProxQpSolver::setSettings(const Settings & settings)
{
  impl_->settings = settings;
  if (impl_->qp) {
    impl_->qp->settings.eps_abs = settings.eps_abs;
    impl_->qp->settings.eps_rel = settings.eps_rel;
    impl_->qp->settings.max_iter = settings.max_iterations;
    impl_->qp->settings.verbose = settings.verbose;
    impl_->qp->settings.compute_timings = settings.compute_timings;
  }
}

const ProxQpSolver::Settings & ProxQpSolver::settings() const
{
  return impl_->settings;
}

void ProxQpSolver::init(int n, int m_eq, int m_in)
{
  if (n <= 0 || m_eq < 0 || m_in < 0) {
    throw std::invalid_argument("ProxQpSolver::init 收到非法尺寸");
  }
  impl_->n = n;
  impl_->m_eq = m_eq;
  impl_->m_in = m_in;
  // box_constraints = true：本包全部约束都是简单边界，走 ProxQP 的 box 快路径。
  impl_->qp = std::make_unique<ProxQp>(n, m_eq, m_in, /*box_constraints=*/true);
  impl_->h_sym.resize(n, n);
  setSettings(impl_->settings);
  impl_->warmed_up = false;
}

void ProxQpSolver::reset()
{
  impl_->warmed_up = false;
}

QpResult ProxQpSolver::solve(const QpProblem & problem)
{
  QpResult out;

  if (!impl_->qp) {
    out.status = QpStatus::kNumericalError;
    return out;
  }

  std::string message;
  if (!problem.validate(message)) {
    out.status = QpStatus::kNumericalError;
    return out;
  }
  const int n = impl_->n;
  if (problem.H.rows() != n || problem.g.size() != n) {
    out.status = QpStatus::kNumericalError;
    return out;
  }
  // box 允许整体缺省（空 = 无 box 约束，按 QpProblem 的约定用 0 尺寸表示）；
  // 但一旦给出就必须是 n 维。
  if (problem.l_box.size() != 0 && problem.l_box.size() != n) {
    out.status = QpStatus::kNumericalError;
    return out;
  }
  if (problem.u_box.size() != 0 && problem.u_box.size() != n) {
    out.status = QpStatus::kNumericalError;
    return out;
  }

  // ★ H 显式对称化。数学上 H 本就是对称的（JᵀWJ，W 为对角），但 Eigen 的浮点
  //   结果是否**逐位**对称取决于内部累加顺序，属"碰巧成立"；而 ProxSuite 的判据
  //   是机器精度的相对容差，一旦不成立就抛异常。这里花 n² 次运算换掉整类风险。
  impl_->h_sym.noalias() = 0.5 * (problem.H + problem.H.transpose());

  try {
    // 空约束一律以 nullopt 传入（0 尺寸的 Eigen 对象不能直接当 MatRef 用）。
    const optional<proxsuite::proxqp::dense::MatRef<double>> H(impl_->h_sym);
    const optional<proxsuite::proxqp::dense::VecRef<double>> g(problem.g);
    const auto A = optOf(problem.A);
    const auto b = optOf(problem.b);
    const auto C = optOf(problem.C);
    const auto l = optOf(problem.l);
    const auto u = optOf(problem.u);
    const auto l_box = optOf(problem.l_box);
    const auto u_box = optOf(problem.u_box);

    if (!impl_->warmed_up) {
      impl_->qp->init(H, g, A, b, C, l, u, l_box, u_box);
      impl_->warmed_up = true;
    } else {
      // ★ 更新预处理器。
      //
      // 一度为了省时间传 false（理由是"200 Hz 下 H 每帧只缓慢变化"）——那个假设
      // 在**伺服**下成立，在**一次性求解**下完全不成立：位姿遍历里每次求解的构型
      // 都不同，J 变化剧烈，冻结的首帧预处理器不再适用。MuJoCo 遍历实测暴露：
      // 首步即败的比例随增益单调暴涨（步长 3e-2 时 142/400，步长 1e-2 时
      // 360/400），且 QP 状态出现 primal_infeasible —— 而 box 约束经中点保护后
      // **恒为可行**，primal infeasible 在精确算术下不可能，只能是数值退化。
      //
      // 代价可忽略：n=7~14 的稠密问题，预处理是 O(n³)，远小于一次求解本身。
      impl_->qp->update(H, g, A, b, C, l, u, l_box, u_box, /*update_preconditioner=*/true);
    }
    impl_->qp->solve();

    const auto & info = impl_->qp->results.info;
    out.status = mapStatus(info.status);
    out.x = impl_->qp->results.x;
    out.objective = info.objValue;
    out.iterations = static_cast<int>(info.iter);
    out.setup_time_us = impl_->settings.compute_timings ? info.setup_time : 0.0;
    out.solve_time_us = impl_->settings.compute_timings ? info.solve_time : 0.0;
    out.primal_infeasible_count = (out.status == QpStatus::kPrimalInfeasible) ? 1 : 0;
    // 撞上限单独标出来：语义是"算力不够没算完"，与"问题无解"完全不同。
    out.hit_iteration_cap = (out.status == QpStatus::kMaxIterations);
  } catch (const std::exception &) {
    // 求解器异常不得穿出实时回路。作废热启动状态，下一帧重新冷启动，
    // 避免"坏状态被热启动反复继承"。
    impl_->warmed_up = false;
    out.status = QpStatus::kNumericalError;
    out.x = Eigen::VectorXd::Zero(n);
  } catch (...) {
    impl_->warmed_up = false;
    out.status = QpStatus::kNumericalError;
    out.x = Eigen::VectorXd::Zero(n);
  }

  return out;
}

}  // namespace flex_qp_ik
