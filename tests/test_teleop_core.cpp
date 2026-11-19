// SPDX-License-Identifier: Apache-2.0
//
// TeleopCore 编排测试：仲裁、模式切换、状态机、溯源标签冻结。
//
#include <cmath>
#include <cstdio>

#include "flex_teleop/pose_math.hpp"
#include "flex_teleop/teleop_core.hpp"

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

/// 玩具 IK：把位姿平移直接当作关节角，FK 取其逆。
/// 只为验证编排逻辑，不代表任何真实运动学。
class ToyIK : public IKBackend {
 public:
  bool solve(const Pose& T_target, const double q_hint[kDof],
             const double q_lower[kDof], const double q_upper[kDof],
             double dt_s, double q_out[kDof]) override {
    (void)q_hint; (void)q_lower; (void)q_upper; (void)dt_s;
    if (fail_next) {
      fail_next = false;
      return false;
    }
    for (int i = 0; i < kDof; ++i) {
      q_out[i] = 0.0;
    }
    q_out[0] = T_target.m[3];
    q_out[1] = T_target.m[7];
    q_out[2] = T_target.m[11];
    return true;
  }

  void fk(const double q[kDof], Pose& T_out) const override {
    double p[3] = {q[0], q[1], q[2]};
    double quat[4] = {0.0, 0.0, 0.0, 1.0};
    T_out = makePose(p, quat);
  }

  const char* name() const override { return "toy"; }

  bool fail_next{false};
};

Pose poseAt(double x, double y, double z) {
  double p[3] = {x, y, z};
  double q[4] = {0.0, 0.0, 0.0, 1.0};
  return makePose(p, q);
}

TeleopCore makeCore() {
  TeleopCore::Config cfg;
  cfg.fixed_delay_s = 0.035;
  cfg.degraded_hold_frames = 3;
  cfg.fault_hold_frames = 10;
  return TeleopCore(cfg);
}

void testNoInput() {
  std::printf("[no input]\n");
  TeleopCore core = makeCore();
  const JointStream s = core.tick(0);
  CHECK(s.mode == Mode::kHold);
  CHECK(s.solver == Solver::kHold);
  CHECK(core.status().state == State::kHold);
}

void testPoseTracking() {
  std::printf("[pose tracking]\n");
  TeleopCore core = makeCore();
  ToyIK ik;
  core.setIK(&ik);

  core.submitPose(poseAt(0, 0, 0), 1, 0);
  core.submitPose(poseAt(0.3, 0, 0), 2, kPeriod);

  // t_query = 15ms -> alpha = 0.45 -> x = 0.135
  const JointStream s = core.tick(50 * kMs);
  CHECK(s.mode == Mode::kPose);
  CHECK(s.solver == Solver::kQp);
  CHECK(s.src.id_a == 1);
  CHECK(s.src.id_b == 2);
  CHECK_NEAR(s.src.alpha, 0.45, 1e-3);
  CHECK_NEAR(s.q_cmd[0], 0.3 * 0.45, 1e-3);
  CHECK(core.status().state == State::kTracking);
  CHECK(core.status().pose_id_ack == 2);
}

void testJointBypass() {
  std::printf("[joint bypass]\n");
  TeleopCore core = makeCore();
  ToyIK ik;
  core.setIK(&ik);

  double q0[kDof] = {0, 0, 0, 0, 0, 0, 0};
  double q1[kDof] = {1, 0, 0, 0, 0, 0, 0};
  core.submitJoint(q0, 10, 0);
  core.submitJoint(q1, 11, kPeriod);

  const JointStream s = core.tick(50 * kMs);
  CHECK(s.mode == Mode::kJoint);
  CHECK_NEAR(s.q_cmd[0], 0.45, 1e-3);
  CHECK(s.src.id_a == 10);
  CHECK(s.src.id_b == 11);
}

void testModeSwitch() {
  std::printf("[mode switch]\n");
  TeleopCore core = makeCore();
  ToyIK ik;
  core.setIK(&ik);

  core.submitPose(poseAt(0, 0, 0), 1, 0);
  core.submitPose(poseAt(1, 0, 0), 2, kPeriod);
  CHECK(core.tick(50 * kMs).mode == Mode::kPose);

  // 切到关节通路，id 继续递增（共享序列空间）
  double q[kDof] = {2, 0, 0, 0, 0, 0, 0};
  core.submitJoint(q, 3, 2 * kPeriod);
  core.submitJoint(q, 4, 3 * kPeriod);

  const JointStream s = core.tick(4 * kPeriod);
  CHECK(s.mode == Mode::kJoint);
  CHECK(s.src.id_a == 3);
}

void testHoldFreezesTag() {
  std::printf("[hold freezes src tag]\n");
  TeleopCore core = makeCore();
  ToyIK ik;
  core.setIK(&ik);

  core.submitPose(poseAt(0, 0, 0), 1, 0);
  core.submitPose(poseAt(0.3, 0, 0), 2, kPeriod);
  const JointStream ok = core.tick(50 * kMs);
  CHECK(ok.mode == Mode::kPose);
  const SrcTag good = ok.src;

  // 远超 B 的时刻 -> fresh=false -> HOLD，标签必须冻结
  const JointStream held = core.tick(10 * kPeriod);
  CHECK(held.mode == Mode::kHold);
  CHECK(held.src.id_a == good.id_a);
  CHECK(held.src.id_b == good.id_b);
  CHECK_NEAR(held.src.alpha, good.alpha, 1e-12);
}

void testDegradedEscalation() {
  std::printf("[degraded -> fault]\n");
  TeleopCore core = makeCore();
  ToyIK ik;
  core.setIK(&ik);

  core.submitPose(poseAt(0, 0, 0), 1, 0);
  core.submitPose(poseAt(0.3, 0, 0), 2, kPeriod);

  const std::int64_t late = 10 * kPeriod;
  core.tick(late);  // 1st hold
  CHECK(core.status().state == State::kHold);
  core.tick(late);
  core.tick(late);
  core.tick(late);  // 4th hold > degraded(3)
  CHECK(core.status().state == State::kDegraded);

  for (int i = 0; i < 10; ++i) {
    core.tick(late);
  }
  CHECK(core.status().state == State::kFault);
}

void testSolverFailureHolds() {
  std::printf("[solver failure holds]\n");
  TeleopCore core = makeCore();
  ToyIK ik;
  core.setIK(&ik);
  ik.fail_next = true;

  core.submitPose(poseAt(0, 0, 0), 1, 0);
  core.submitPose(poseAt(0.3, 0, 0), 2, kPeriod);

  const JointStream s = core.tick(50 * kMs);
  CHECK(s.mode == Mode::kHold);
  CHECK(s.solver == Solver::kHold);
}

void testTicksAreSequential() {
  std::printf("[tick_seq]\n");
  TeleopCore core = makeCore();
  core.submitPose(poseAt(0, 0, 0), 1, 0);
  const TickSeq a = core.tick(0).tick_seq;
  const TickSeq b = core.tick(1).tick_seq;
  CHECK(b == a + 1);
}

void testErrorReporting() {
  std::printf("[e_model / e_real]\n");
  TeleopCore core = makeCore();
  ToyIK ik;
  core.setIK(&ik);

  core.submitPose(poseAt(0, 0, 0), 1, 0);
  core.submitPose(poseAt(0.3, 0, 0), 2, kPeriod);

  // 反馈停在原点：e_real 应等于目标 x，e_model 应接近 0
  JointFeedback fb{};
  core.submitFeedback(fb);

  core.tick(50 * kMs);
  const TeleopStatus& st = core.status();
  CHECK(st.e_model.pos_m < 1e-6);
  CHECK_NEAR(st.e_real.pos_m, 0.3 * 0.45, 1e-3);
  CHECK_NEAR(st.lead_q[0], 0.3 * 0.45, 1e-3);
}

}  // namespace

int main() {
  testNoInput();
  testPoseTracking();
  testJointBypass();
  testModeSwitch();
  testHoldFreezesTag();
  testDegradedEscalation();
  testSolverFailureHolds();
  testTicksAreSequential();
  testErrorReporting();

  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
