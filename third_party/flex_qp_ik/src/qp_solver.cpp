// SPDX-License-Identifier: BSD-3-Clause
//
// 文件：qp_solver.cpp
// 包角色：QpSolver 抽象层的公共实现（状态字符串、数据自检、后端工厂）。

#include "flex_qp_ik/qp_solver.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include "flex_qp_ik/proxqp_solver.hpp"

namespace flex_qp_ik
{

const char * toString(QpStatus status)
{
  switch (status) {
    case QpStatus::kSolved:
      return "solved";
    case QpStatus::kPrimalInfeasible:
      return "primal_infeasible";
    case QpStatus::kDualInfeasible:
      return "dual_infeasible";
    case QpStatus::kMaxIterations:
      return "max_iterations";
    case QpStatus::kNumericalError:
      return "numerical_error";
    case QpStatus::kUnknown:
    default:
      return "unknown";
  }
}

bool QpProblem::validate(std::string & message) const
{
  const Eigen::Index n = g.size();
  if (n <= 0) {
    message = "g 为空";
    return false;
  }

  // H：n×n
  if (H.rows() != n || H.cols() != n) {
    message = "H 不是 n×n（n = g.size()）";
    return false;
  }
  if (!H.allFinite() || !g.allFinite()) {
    message = "H 或 g 含非有限值";
    return false;
  }

  // 等式：A 与 b 必须同时为空或同时为 m_eq 行
  const Eigen::Index m_eq = b.size();
  if (A.size() == 0 && m_eq != 0) {
    message = "b 非空但 A 为空";
    return false;
  }
  if (A.size() != 0 && (A.rows() != m_eq || A.cols() != n)) {
    message = "A 不是 m_eq×n";
    return false;
  }
  if (A.size() != 0 && (!A.allFinite() || !b.allFinite())) {
    message = "A 或 b 含非有限值";
    return false;
  }

  // 不等式：C/l/u 必须同时为空或同时为 m_in 行
  const Eigen::Index m_in = l.size();
  if (C.size() == 0 && m_in != 0) {
    message = "l 非空但 C 为空";
    return false;
  }
  if (C.size() != 0 && (C.rows() != m_in || C.cols() != n)) {
    message = "C 不是 m_in×n";
    return false;
  }
  if (u.size() != m_in) {
    message = "l 与 u 长度不一致";
    return false;
  }
  if (C.size() != 0 && (!C.allFinite() || !l.allFinite() || !u.allFinite())) {
    message = "C、l 或 u 含非有限值";
    return false;
  }
  if (m_in != 0 && (l.array() > u.array()).any()) {
    message = "存在 l > u 的不等式约束行";
    return false;
  }

  // box：l_box/u_box 必须同时为空或同时为 n 维
  if (l_box.size() != u_box.size()) {
    message = "l_box 与 u_box 长度不一致";
    return false;
  }
  if (l_box.size() != 0) {
    if (l_box.size() != n) {
      message = "l_box 不是 n 维";
      return false;
    }
    if (!l_box.allFinite() || !u_box.allFinite()) {
      message = "l_box 或 u_box 含非有限值";
      return false;
    }
    if ((l_box.array() > u_box.array()).any()) {
      message = "存在 l_box > u_box 的维度（区间为空）";
      return false;
    }
  }

  message.clear();
  return true;
}

std::unique_ptr<QpSolver> makeQpSolver(const std::string & backend)
{
  std::string key;
  key.reserve(backend.size());
  for (const char c : backend) {
    key.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }

  if (key == "proxqp" || key == "proxsuite") {
    return std::make_unique<ProxQpSolver>();
  }

  throw std::invalid_argument(
          "未知 QP 后端 '" + backend + "'；可用：" + availableQpBackends());
}

std::string availableQpBackends()
{
  return "proxqp";
}

}  // namespace flex_qp_ik
