// SPDX-License-Identifier: Apache-2.0
//
// 200Hz 节拍基准 + 实时路径无堆分配验证。
//
// 通过全局重载 operator new/delete 统计测量窗口内的分配次数；
// tick 与 submit* 在稳态下必须为 0。
//
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

#include "flex_teleop/pose_math.hpp"
#include "flex_teleop/teleop_core.hpp"

using namespace flex_teleop;

namespace {
std::atomic<std::size_t> g_allocs{0};
}  // namespace

void* operator new(std::size_t n) {
  g_allocs.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(n)) {
    return p;
  }
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {

constexpr std::int64_t kPeriod200 = 5000000LL;   // 5 ms
constexpr std::int64_t kPeriod30 = 33333333LL;   // 33.3 ms
constexpr int kIterations = 200000;              // 1000 s @ 200Hz
constexpr double kPi = 3.14159265358979323846;

class ToyIK : public IKBackend {
 public:
  bool solve(const Pose& T, const double[7], const double[7], const double[7],
             double, double q_out[7]) override {
    for (int i = 0; i < kDof; ++i) {
      q_out[i] = 0.0;
    }
    q_out[0] = T.m[3];
    q_out[1] = T.m[7];
    q_out[2] = T.m[11];
    return true;
  }
  void fk(const double q[kDof], Pose& T_out) const override {
    double p[3] = {q[0], q[1], q[2]};
    double quat[4] = {0.0, 0.0, 0.0, 1.0};
    T_out = makePose(p, quat);
  }
  const char* name() const override { return "toy"; }
};

Pose poseAt(double x) {
  double p[3] = {x, 0.0, 0.0};
  double quat[4] = {0.0, 0.0, 0.0, 1.0};
  return makePose(p, quat);
}

double percentile(std::vector<double>& v, double p) {
  if (v.empty()) return 0.0;
  const std::size_t idx =
      static_cast<std::size_t>(p * static_cast<double>(v.size() - 1));
  return v[idx];
}

}  // namespace

int main() {
  TeleopCore::Config cfg;
  cfg.fixed_delay_s = 0.035;
  cfg.output_rate_hz = 200.0;

  TeleopCore core(cfg);
  ToyIK ik;
  core.setIK(&ik);

  // 预热（避免首次分支/缓存效应计入）
  {
    PoseId id = 1;
    std::int64_t next_in = 0;
    for (int k = 0; k < 2000; ++k) {
      const std::int64_t t = k * kPeriod200;
      while (t >= next_in) {
        core.submitPose(poseAt(0.0), id++, next_in);
        next_in += kPeriod30;
      }
      core.tick(t);
    }
  }

  std::vector<double> tick_us;
  tick_us.reserve(kIterations);

  PoseId id = 10000;
  std::int64_t next_in = 0;

  const std::size_t alloc_before = g_allocs.load();
  for (int k = 0; k < kIterations; ++k) {
    const std::int64_t t = k * kPeriod200;

    while (t >= next_in) {
      const double x = 0.25 * std::sin(2.0 * kPi * static_cast<double>(next_in) * 1e-9);
      core.submitPose(poseAt(x), id++, next_in);
      next_in += kPeriod30;
    }

    const auto t0 = std::chrono::steady_clock::now();
    core.tick(t);
    const auto t1 = std::chrono::steady_clock::now();
    tick_us.push_back(
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()) *
        1e-3);
  }
  const std::size_t alloc_after = g_allocs.load();

  std::vector<double> sorted = tick_us;
  std::sort(sorted.begin(), sorted.end());

  double sum = 0.0;
  for (double v : tick_us) sum += v;

  std::printf("=== flex_teleop tick benchmark ===\n");
  std::printf("iterations      : %d\n", kIterations);
  std::printf("mean            : %.3f us\n", sum / static_cast<double>(tick_us.size()));
  std::printf("p50             : %.3f us\n", percentile(sorted, 0.50));
  std::printf("p90             : %.3f us\n", percentile(sorted, 0.90));
  std::printf("p99             : %.3f us\n", percentile(sorted, 0.99));
  std::printf("p99.9           : %.3f us\n", percentile(sorted, 0.999));
  std::printf("max             : %.3f us\n", sorted.back());
  std::printf("budget (200Hz)  : 5000.000 us\n");
  std::printf("headroom @ p99  : %.1f x\n",
              5000.0 / (percentile(sorted, 0.99) > 0.0 ? percentile(sorted, 0.99) : 1.0));
  std::printf("heap allocs     : %zu (must be 0)\n", alloc_after - alloc_before);
  return (alloc_after == alloc_before) ? 0 : 2;
}
