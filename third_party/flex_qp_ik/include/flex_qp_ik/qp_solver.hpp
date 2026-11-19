// SPDX-License-Identifier: BSD-3-Clause
//
// =============================================================================
// 文件：qp_solver.hpp
// 包角色：L0 层 QP 求解器抽象（仅 Eigen，无 ROS、无具体后端）
//
// 存在的意义：把**求解器选型**挡在接口后面。上层（FlexQpIk / 未来的双臂协调层）
// 只认 QpSolver，换 ProxQP / OSQP / qpOASES / 自研都不影响上层代码。
//
// 约定（**热启动是 200 Hz 回路能跑到 μs 级的关键**）：
//   - 同一个 solver 实例**串行复用**，尺寸在 init 后固定；
//   - solve() 内部保留上一次解作为热启动初值；
//   - 需要冷启动时显式调 reset()，或重新 init()。
// =============================================================================

#ifndef FLEX_QP_IK__QP_SOLVER_HPP_
#define FLEX_QP_IK__QP_SOLVER_HPP_

#include <memory>
#include <string>

#include <Eigen/Core>

namespace flex_qp_ik
{

/// 求解状态。取值刻意对齐 ProxQP / OSQP 的公共子集，便于换后端而不改上层判据。
enum class QpStatus
{
  kSolved,            // 达到精度容差的最优解
  kPrimalInfeasible,  // 原问题不可行（约束互相矛盾）
  kDualInfeasible,    // 对偶不可行（目标无下界）
  kMaxIterations,     // 迭代耗尽，解不可信
  kNumericalError,    // 数值失败（NaN 输入、分解失败等）
  kUnknown,
};

/// 状态的可读描述（日志 / 诊断字符串）。
const char * toString(QpStatus status);

/// 标准 QP 数据：
///
///   min  ½ xᵀH x + gᵀx
///   s.t. A x = b                （m_eq 行）
///        l ≤ C x ≤ u            （m_in 行）
///        l_box ≤ x ≤ u_box      （n 行）
///
/// 空约束一律用 **0 行或 0 列**的矩阵表示，不用 NaN 哨兵——各后端对 NaN 的
/// 处理不一致，用空矩阵语义唯一。
struct QpProblem
{
  Eigen::MatrixXd H;      // n×n 对称半正定（本包始终给满阵，不依赖三角只读约定）
  Eigen::VectorXd g;      // n
  Eigen::MatrixXd A;      // m_eq×n
  Eigen::VectorXd b;      // m_eq
  Eigen::MatrixXd C;      // m_in×n
  Eigen::VectorXd l;      // m_in
  Eigen::VectorXd u;      // m_in
  Eigen::VectorXd l_box;  // n
  Eigen::VectorXd u_box;  // n

  /// 维度一致性自检。不一致时返回 false 并把原因写入 message。
  bool validate(std::string & message) const;
};

/// 一次 solve 的结果与诊断。
struct QpResult
{
  QpStatus status{QpStatus::kUnknown};
  /// 解。kSolved 时有效；其他状态为最后一次迭代值（仅供诊断，不可直接使用）。
  Eigen::VectorXd x;
  double objective{0.0};
  int iterations{0};
  /// 数据装配 + 预处理耗时 [μs]。
  double setup_time_us{0.0};
  /// 迭代耗时 [μs]。
  double solve_time_us{0.0};
  /// 迭代中观察到原问题不可行的次数，用于区分"一直不可行"与"偶发数值抖动"。
  int primal_infeasible_count{0};
  /// **是否撞上迭代上限**（`kMaxIterations`）。
  ///
  /// ★ 这是"这一帧没跟上"的最直接指标：撞上限意味着求解器在给定算力内没算完，
  ///   不是问题无解。对伺服而言它会导致该帧走"保持"，所以调用方应当**统计它的
  ///   发生率**——上实机验证实时性时，这个计数比耗时更能说明问题。
  bool hit_iteration_cap{false};
};

/// QP 求解器抽象接口。
class QpSolver
{
public:
  virtual ~QpSolver() = default;

  /// 按固定尺寸分配工作区（含预处理缓存）。尺寸变化必须重新调用。
  /// @param n     决策变量维数
  /// @param m_eq  等式约束行数（0 表示无）
  /// @param m_in  通用不等式约束行数（0 表示无）
  virtual void init(int n, int m_eq, int m_in) = 0;

  /// 清掉热启动状态：下一次 solve() 等价冷启动。
  virtual void reset() = 0;

  /// 求解。problem 的尺寸必须与最近的 init() 一致，否则返回 kNumericalError。
  virtual QpResult solve(const QpProblem & problem) = 0;

  /// 后端名（日志 / 诊断用）。
  virtual const char * name() const = 0;
};

/// 按名字构造后端。当前支持 "proxqp"（大小写不敏感）。
/// 未知名字抛 std::invalid_argument——配置错误应当立即暴露，不静默降级。
std::unique_ptr<QpSolver> makeQpSolver(const std::string & backend);

/// 当前编译进本包的后端名列表（供 yaml 校验与日志打印）。
std::string availableQpBackends();

}  // namespace flex_qp_ik

#endif  // FLEX_QP_IK__QP_SOLVER_HPP_
