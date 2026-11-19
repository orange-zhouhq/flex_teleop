// SPDX-License-Identifier: Apache-2.0
//
// Interpolator 单元测试。零第三方依赖，直接可执行。
//
#include <cmath>
#include <cstdio>

#include "flex_teleop/interpolator.hpp"
#include "flex_teleop/pose_math.hpp"

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

constexpr std::int64_t kMs = 1000000LL;
constexpr std::int64_t kPeriod = 33333333LL;  // 30Hz
constexpr double kDelayS = 0.035;             // 1.05 / 30

Pose poseAt(double x, double y, double z) {
  const double p[3] = {x, y, z};
  const double q[4] = {0.0, 0.0, 0.0, 1.0};
  return makePose(p, q);
}

Pose poseRotZ(double angle_rad, double x = 0.0, double y = 0.0, double z = 0.0) {
  const double p[3] = {x, y, z};
  const double half = angle_rad * 0.5;
  const double q[4] = {0.0, 0.0, std::sin(half), std::cos(half)};
  return makePose(p, q);
}

void testEmpty() {
  std::printf("[empty]\n");
  Interpolator it(kDelayS);
  const InterpSample s = it.sample(0);
  CHECK(!s.valid);
  CHECK(!s.fresh);
  CHECK(!it.hasTarget());
}

void testFirstFrame() {
  std::printf("[first frame]\n");
  Interpolator it(kDelayS);
  CHECK(it.push(poseAt(1, 2, 3), 100, 0));
  CHECK(it.hasTarget());
  CHECK(!it.hasInterval());

  const InterpSample s = it.sample(0);
  CHECK(s.valid);
  CHECK(s.id_a == 100);
  CHECK(s.id_b == 100);
  CHECK_NEAR(s.alpha, 0.0, 1e-12);

  double p[3];
  poseTranslation(s.T, p);
  CHECK_NEAR(p[0], 1.0, 1e-9);
  CHECK_NEAR(p[2], 3.0, 1e-9);
}

void testAlphaAndDelay() {
  std::printf("[alpha / fixed delay]\n");
  Interpolator it(kDelayS);
  it.push(poseAt(0, 0, 0), 100, 0);
  it.push(poseAt(0.3, 0, 0), 101, kPeriod);

  // t_exec = 50ms -> t_query = 15ms -> alpha = 15 / 33.333 = 0.45
  {
    const InterpSample s = it.sample(50 * kMs);
    CHECK(s.valid);
    CHECK(s.fresh);
    CHECK(s.id_a == 100);
    CHECK(s.id_b == 101);
    CHECK_NEAR(s.alpha, 15.0 / 33.333333, 1e-4);
    double p[3];
    poseTranslation(s.T, p);
    CHECK_NEAR(p[0], 0.3 * (15.0 / 33.333333), 1e-4);
  }

  // t_query 正好落在 B 上：alpha = 1，仍 fresh
  {
    const InterpSample s = it.sample(kPeriod + 35 * kMs);
    CHECK_NEAR(s.alpha, 1.0, 1e-9);
    CHECK(s.fresh);
  }

  // 超过 B：钳到 1，但 fresh=false（上游断流）
  {
    const InterpSample s = it.sample(kPeriod + 40 * kMs);
    CHECK_NEAR(s.alpha, 1.0, 1e-9);
    CHECK(!s.fresh);
  }

  // 早于 A：钳到 0，仍 fresh
  {
    const InterpSample s = it.sample(0);
    CHECK_NEAR(s.alpha, 0.0, 1e-9);
    CHECK(s.fresh);
  }
}

void testRejectOutOfOrder() {
  std::printf("[out-of-order / duplicate]\n");
  Interpolator it(kDelayS);
  CHECK(it.push(poseAt(0, 0, 0), 100, 0));
  CHECK(it.push(poseAt(1, 0, 0), 101, kPeriod));

  CHECK(!it.push(poseAt(9, 9, 9), 100, 2 * kPeriod));  // 重复
  CHECK(!it.push(poseAt(9, 9, 9), 99, 2 * kPeriod));   // 更旧
  CHECK(it.push(poseAt(2, 0, 0), 105, 2 * kPeriod));   // 跳号允许

  const InterpSample s = it.sample(2 * kPeriod + 35 * kMs);
  CHECK(s.id_b == 105);
  CHECK(it.lastId() == 105);
}

void testTimeRegressionGuard() {
  std::printf("[time regression guard]\n");
  Interpolator it(kDelayS);
  it.push(poseAt(0, 0, 0), 100, 100 * kMs);
  CHECK(it.push(poseAt(1, 0, 0), 101, 50 * kMs));  // 到达时刻倒退

  const InterpSample s = it.sample(100 * kMs + 35 * kMs);
  CHECK(s.valid);
  CHECK(s.alpha >= 0.0 && s.alpha <= 1.0);
}

void testRotationSlerp() {
  std::printf("[rotation slerp]\n");
  const Pose a = poseRotZ(0.0);
  const Pose b = poseRotZ(3.14159265358979323846 / 2.0);  // 90 deg

  const Pose mid = interpolatePose(a, b, 0.5);
  CHECK_NEAR(rotationAngle(a, mid), 3.14159265358979323846 / 4.0, 1e-6);
  CHECK_NEAR(rotationAngle(mid, b), 3.14159265358979323846 / 4.0, 1e-6);

  const Pose q1 = interpolatePose(a, b, 0.0);
  CHECK_NEAR(rotationAngle(q1, a), 0.0, 1e-9);
  const Pose q2 = interpolatePose(a, b, 1.0);
  CHECK_NEAR(rotationAngle(q2, b), 0.0, 1e-9);
}

void testQuaternionSignInvariance() {
  std::printf("[quaternion sign invariance]\n");
  const double half = 0.6;
  const double q[4] = {0.0, 0.0, std::sin(half), std::cos(half)};
  const double qn[4] = {0.0, 0.0, -std::sin(half), -std::cos(half)};  // -q，同一旋转

  const Pose a = quaternionToPose(q, nullptr);
  const Pose b = quaternionToPose(qn, nullptr);
  CHECK_NEAR(rotationAngle(a, b), 0.0, 1e-9);

  // 跨符号插值不应走长弧：总转角为 2*half，一半即 half
  const Pose c = poseRotZ(0.0);
  const Pose mid = interpolatePose(c, b, 0.5);
  CHECK_NEAR(rotationAngle(c, mid), half, 1e-6);
}

void testReset() {
  std::printf("[reset]\n");
  Interpolator it(kDelayS);
  it.push(poseAt(0, 0, 0), 100, 0);
  it.reset();
  CHECK(!it.hasTarget());
  CHECK(!it.sample(0).valid);
  CHECK(it.lastId() == 0);
}

}  // namespace

int main() {
  testEmpty();
  testFirstFrame();
  testAlphaAndDelay();
  testRejectOutOfOrder();
  testTimeRegressionGuard();
  testRotationSlerp();
  testQuaternionSignInvariance();
  testReset();

  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
