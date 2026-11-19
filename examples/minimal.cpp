// SPDX-License-Identifier: Apache-2.0
//
// 最小可运行示例：30Hz 目标 -> 200Hz 关节流。
//
// 用一个玩具 IK 演示编排流程（位姿平移直接映射为关节角），
// 不代表任何真实运动学。真实部署请实现自己的 IKBackend。
//
#include <cmath>
#include <cstdio>

#include "flex_teleop/pose_math.hpp"
#include "flex_teleop/teleop_core.hpp"

using namespace flex_teleop;

namespace {

class ToyIK : public IKBackend {
 public:
  bool solve(const Pose& T_target, const double q_hint[kDof],
             const double q_lower[kDof], const double q_upper[kDof],
             double dt_s, double q_out[kDof]) override {
    (void)q_hint; (void)q_lower; (void)q_upper; (void)dt_s;
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

  Solver kind() const override { return Solver::kQp; }
  const char* name() const override { return "toy"; }
};

Pose poseAt(double x, double y, double z) {
  double p[3] = {x, y, z};
  double quat[4] = {0.0, 0.0, 0.0, 1.0};
  return makePose(p, quat);
}

}  // namespace

int main() {
  TeleopCore::Config cfg;
  cfg.side = Side::kLeft;
  cfg.output_rate_hz = 200.0;
  cfg.fixed_delay_s = 0.035;  // 1.05 / 30Hz

  TeleopCore core(cfg);
  ToyIK ik;
  core.setIK(&ik);

  constexpr std::int64_t kPeriod200 = 5000000LL;   // 5ms
  constexpr std::int64_t kPeriod30 = 33333333LL;   // 33.3ms
  constexpr double kPi = 3.14159265358979323846;

  PoseId next_id = 1;
  std::int64_t next_input_t = 0;

  std::printf("%6s %10s %10s %8s %8s %s\n",
              "t(ms)", "mode", "q0", "id_a", "id_b", "alpha");

  for (int k = 0; k < 400; ++k) {
    const std::int64_t t = k * kPeriod200;

    // 30Hz 输入：正弦轨迹
    while (t >= next_input_t) {
      const double x = 0.25 * std::sin(2.0 * kPi * static_cast<double>(next_input_t) * 1e-9);
      core.submitPose(poseAt(x, 0.0, 0.0), next_id++, next_input_t);
      next_input_t += kPeriod30;
    }

    const JointStream s = core.tick(t);

    if (k % 10 == 0) {
      std::printf("%6.0f %10s %10.4f %8llu %8llu %.3f\n",
                  static_cast<double>(t) * 1e-6,
                  s.mode == Mode::kPose ? "POSE"
                                        : (s.mode == Mode::kJoint ? "JOINT" : "HOLD"),
                  s.q_cmd[0],
                  static_cast<unsigned long long>(s.src.id_a),
                  static_cast<unsigned long long>(s.src.id_b),
                  s.src.alpha);
    }
  }

  const TeleopStatus& st = core.status();
  std::printf("\nfinal state=%d  ack=%llu  e_model=%.4f m  e_real=%.4f m\n",
              static_cast<int>(st.state),
              static_cast<unsigned long long>(st.pose_id_ack),
              st.e_model.pos_m, st.e_real.pos_m);
  return 0;
}
