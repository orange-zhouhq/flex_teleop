// SPDX-License-Identifier: BSD-3-Clause
//
// QpIkBackend 冒烟测试：用一组**通用** S-R-S 几何（非任何具体产品参数）
// 验证接口串通：构造 -> FK -> solve -> FK 回代。
//
#include <cmath>
#include <cstdio>

#include "flex_teleop/pose_math.hpp"
#include "flex_teleop/qp_ik_backend.hpp"

using namespace flex_teleop;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                      \
  do {                                                                   \
    ++g_checks;                                                          \
    if (!(cond)) {                                                       \
      std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
      ++g_failures;                                                      \
    }                                                                    \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                            \
  do {                                                                   \
    ++g_checks;                                                          \
    const double _a = (a), _b = (b);                                     \
    if (std::fabs(_a - _b) > (tol)) {                                    \
      std::printf("  FAIL %s:%d  |%.9f - %.9f| > %g\n", __FILE__,        \
                  __LINE__, _a, _b, static_cast<double>(tol));           \
      ++g_failures;                                                      \
    }                                                                    \
  } while (0)

constexpr double kPi = 3.14159265358979323846;

/// 通用 S-R-S 构型（球形腕，尺寸取常见量级，不代表任何产品）。
srs_dh_model::SrsGeometry genericSrs() {
  srs_dh_model::SrsGeometry g;
  g.a     = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  g.alpha = {0.0, kPi / 2, -kPi / 2, kPi / 2, -kPi / 2, kPi / 2, 0.0};
  g.d     = {0.30, 0.0, 0.30, 0.0, 0.20, 0.0, 0.10};
  for (int i = 0; i < 7; ++i) {
    g.lower[i] = -2.9;
    g.upper[i] = 2.9;
  }
  return g;
}

void testFkSanity() {
  std::printf("[fk sanity]\n");
  QpIkBackend::Config cfg;
  cfg.geometry = genericSrs();
  cfg.one_shot = true;
  QpIkBackend ik(cfg);

  double q[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Pose T{};
  ik.fk(q, T);

  // 零位末端应位于基座附近，模长在合理范围
  double p[3];
  poseTranslation(T, p);
  const double norm = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  CHECK(norm > 0.05);
  CHECK(norm < 1.5);
}

void testSolveRoundTrip() {
  std::printf("[solve round trip]\n");
  QpIkBackend::Config cfg;
  cfg.geometry = genericSrs();
  cfg.one_shot = true;  // 一次性模式：单次调用内部迭代到位
  QpIkBackend ik(cfg);

  const double q_target[7] = {0.2, -0.5, 0.3, -0.8, 0.4, 0.6, -0.2};
  Pose T_target{};
  ik.fk(q_target, T_target);
  CHECK(translationDistance(T_target, identity()) > 0.05);

  const double q_hint[7] = {0, 0, 0, 0, 0, 0, 0};
  double q_lower[7], q_upper[7];
  for (int i = 0; i < 7; ++i) {
    q_lower[i] = -2.9;
    q_upper[i] = 2.9;
  }

  double q_out[7] = {0};
  const bool ok = ik.solve(T_target, q_hint, q_lower, q_upper, 5e-3, q_out);
  CHECK(ok);

  if (ok) {
    Pose T_solved{};
    ik.fk(q_out, T_solved);
    const double pos_err = translationDistance(T_target, T_solved);
    const double rot_err = rotationAngle(T_target, T_solved);
    std::printf("  pos_err=%.6f m  rot_err=%.6f rad\n", pos_err, rot_err);
    CHECK(pos_err < 2e-3);
    CHECK(rot_err < 2e-3);
  }
}

void testServoSteps() {
  std::printf("[servo converges over ticks]\n");
  QpIkBackend::Config cfg;
  cfg.geometry = genericSrs();
  cfg.one_shot = false;  // 伺服模式：每拍一小步
  QpIkBackend ik(cfg);

  const double q_target[7] = {0.1, 0.4, -0.3, 0.5, -0.2, 0.3, 0.1};
  Pose T_target{};
  ik.fk(q_target, T_target);

  double q[7] = {0, 0, 0, 0, 0, 0, 0};
  double q_lower[7], q_upper[7];
  for (int i = 0; i < 7; ++i) {
    q_lower[i] = -2.9;
    q_upper[i] = 2.9;
  }

  double err = 0.0;
  for (int k = 0; k < 200; ++k) {
    double q_next[7];
    if (!ik.solve(T_target, q, q_lower, q_upper, 5e-3, q_next)) {
      continue;  // 保持
    }
    for (int i = 0; i < 7; ++i) {
      q[i] = q_next[i];
    }
    Pose T_now{};
    ik.fk(q, T_now);
    err = translationDistance(T_target, T_now);
    if (err < 1e-3) {
      break;
    }
  }
  std::printf("  final pos_err=%.6f m after servo ticks\n", err);
  CHECK(err < 2e-3);
}

}  // namespace

int main() {
  testFkSanity();
  testSolveRoundTrip();
  testServoSteps();

  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
