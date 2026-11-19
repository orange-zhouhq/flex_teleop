# 06 · 自测报告

> 版本：0.1.0 · 测试日期：2026-11-19
> 平台：x86-64 / Linux 6.8.0-138-generic（**非 RT 内核**）/ GCC Release / CPU governor = `powersave`

本报告只陈述**已经跑过的验证**，并在第 7 节明确列出**没有验证的部分**。
一个只讲"测试全绿"的报告没有价值——重要的是知道哪些地方还没测。

---

## 1. 结论摘要

| 项 | 结果 | 判断 |
|---|---|---|
| 单元测试 | 82 项，0 失败 | ✅ |
| tick 耗时 p99 | 0.45–0.52 μs（预算 5000 μs） | ✅ 余量约 10000× |
| tick 耗时 p99.9 | 1.1–1.3 μs | ✅ |
| tick 耗时 max | 3.6–4.4 ms（**环境所致**，见 3.2） | ⚠️ |
| 实时路径堆分配 | 0 次 / 200000 次调用 | ✅ |
| valgrind | 0 errors, 0 leaks | ✅ |
| cppcheck | 0 告警 | ✅ |
| ROS 2 端到端（joint 模式） | 通过 | ✅ |
| ROS 2 端到端（pose 模式） | **未测** | ❌ |

**总体判断**：核心算法与实时性达标；**实机与 pose 模式尚未验证**，
当前不宜视为"已可用于生产"，适合作为参考实现与二次开发基线。

---

## 2. 测试矩阵

| # | 测试 | 方法 | 覆盖内容 | 结果 |
|---|---|---|---|---|
| 1 | `test_interpolator` | 43 项断言 | α 边界、乱序/重复丢弃、时间倒退保护、四元数符号一致性、slerp 中点、超时判定、reset | ✅ |
| 2 | `test_teleop_core` | 32 项断言 | 双入口仲裁、模式切换、HOLD 冻结溯源标签、DEGRADED→FAULT 升级、求解失败保持、误差上报、tick 递增 | ✅ |
| 3 | `test_qp_backend` | 7 项断言 | FK 合理性、one_shot 圆整精度、servo 多拍收敛 | ✅ |
| 4 | `bench_tick` | 200000 次 | 耗时分布、堆分配计数 | ✅ |
| 5 | valgrind | 全测试 | 内存泄漏 / 越界 | ✅ |
| 6 | cppcheck | core + plugins | 静态缺陷 | ✅ |
| 7 | ROS 2 冒烟 | rclpy 脚本 | 30Hz `JointTarget` → 200Hz `JointStream` | ✅ |

**未覆盖**见第 7 节。

---

## 3. 性能基准

### 3.1 耗时分布（200000 次 tick，三次独立运行）

| 指标 | 运行 1 | 运行 2 | 运行 3 |
|---|---|---|---|
| mean | 0.276 μs | — | — |
| p50 | 0.225 μs | — | — |
| p99 | 0.446 μs | 0.495 μs | 0.523 μs |
| p99.9 | 0.996 μs | 1.293 μs | 1.177 μs |
| **max** | **3.93 ms** | **4.44 ms** | **3.64 ms** |

配置：`fixed_delay = 35 ms`，200Hz 输出，玩具 IK（无 QP）。

### 3.2 max 离群值的归因

三次运行 max 都落在 3.6–4.4 ms，**与代码无关**：

- 内核为 `6.8.0-138-generic`，**非 PREEMPT_RT**
- CPU governor 为 `powersave`（降频策略，升频时有唤醒延迟）

p99.9 与 max 相差三个数量级，这是非实时内核的典型特征。
**在 RT 内核 + `performance` governor 下需复测**，本报告不对此作保证。

> 这也印证了工程上的取舍：实时性验证必须看 **p99.9**，
> 单看 max 会被调度噪声淹没，单看 mean 会掩盖风险。

### 3.3 实时路径无堆分配

通过全局重载 `operator new/delete` 统计：

```
heap allocs: 0 (must be 0)      ← 200000 次 tick + 对应 submit* 全程
```

该性质由设计保证：`Interpolator` / `JointInterpolator` / `SourceMux` / `TeleopCore`
全部为固定大小成员，无 `std::vector` 增长、无 `std::string` 构造。

---

## 4. 内存与静态检查

```
valgrind --leak-check=full ./build/test_teleop_core
  All heap blocks were freed -- no leaks are possible
  ERROR SUMMARY: 0 errors from 0 contexts

cppcheck --enable=warning,performance,portability core/ plugins/
  （无输出 = 无告警）
```

---

## 5. QP 后端冒烟

| 项 | 结果 |
|---|---|
| FK 合理性（零位末端模长） | ✅ 在 0.05–1.5 m 内 |
| one_shot 圆整 | pos_err = 1.9e-5 m，rot_err = 1.6e-5 rad |
| servo 多拍收敛 | 200 拍后 pos_err = 1.97e-3 m |

**注意**：使用的是**通用 S-R-S 测试几何**（球形腕，d = 0.30/0.30/0.20/0.10），
不是任何产品参数。真实几何下的精度与稳定性**未验证**。

---

## 6. ROS 2 端到端

```
mode=1 q0=+0.1927 id_a=1 id_b=2 alpha=0.927
mode=1 q0=+0.2077 id_a=2 id_b=3 alpha=0.077   ← 跨帧，id 2→3
mode=1 q0=+0.2228 id_a=2 id_b=3 alpha=0.228
...
```

验证点：
- 30Hz 输入被正确插值到 200Hz
- 溯源三元组随 id 推进
- α 跨帧时正确翻转（0.927 → 0.077）
- `solver=HOLD` 在 joint 模式下为预期值

**仅验证了 joint 旁路路径**；pose 路径（经 IK）未做 ROS 2 端到端。

---

## 7. 未验证项（重要）

以下项**没有做过任何验证**，不应假定其可用：

| # | 未验证项 | 风险 |
|---|---|---|
| 1 | **实机验证** | 所有测试使用玩具 IK 与合成数据；真实 URDF、真实驱动器、真实网络均未接入 |
| 2 | **pose 模式端到端** | ROS 2 只测了 joint 旁路，`pose_target` → IK → `joint_stream` 全链未跑通 |
| 3 | **长时间稳定性** | 最长运行为 1000 秒仿真；无小时级/天级运行数据 |
| 4 | **多线程** | core 假设单线程；适配层用 `std::mutex` 保护，但**未做 TSAN 竞态检测** |
| 5 | **测试覆盖率** | 未采集行/分支覆盖率，实际覆盖比例未知 |
| 6 | **跨平台** | 仅 x86-64 + GCC；ARM/aarch64、Clang 未验证 |
| 7 | **QP 精度对比** | 未与 `flex_qp_ik` 原仓库的完整测试套件对照 |
| 8 | **跨机网络** | ROS 2 测试为同机通信，未验证真实网络抖动与丢包 |
| 9 | **时钟不同源场景** | 未构造 VR 与机器人时钟漂移/回跳的故障注入测试 |
| 10 | **异常路径** | 驱动器断连、反馈中断、IK 持续失败到 FAULT 的实际行为未验证 |
| 11 | **RT 内核** | 未在 PREEMPT_RT + performance governor 下测延迟上限 |
| 12 | **内存占用** | 未测各对象的实际内存足迹 |

---

## 8. 已知限制

1. **外环不闭合**：`q_fb` 不参与目标生成，因此伺服稳态误差不会被 IK 修正。
   这是有意的设计（见 03 文档），但意味着**末端精度取决于伺服本身**。
2. **固定延迟补偿**：33–35 ms 的延迟是主动引入的，遥操手感的上限由此决定。
3. **模式切换不平滑**：切到另一条通路时若插值器无新鲜数据，直接 HOLD 等新帧。
   未实现"用当前关节角播种"的平滑交接。
4. **单臂实例**：`TeleopCore` 是单臂的，双臂需两个实例 + 外部协调。

---

## 9. 复现命令

```bash
# 核心测试
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure

# 基准
./build/bench_tick

# 内存检查
valgrind --leak-check=full ./build/test_teleop_core

# 静态分析
cppcheck --enable=warning,performance,portability core/ plugins/

# QP 后端
cmake -S . -B build-qp -DFLEX_TELEOP_WITH_QP_IK=ON
cmake --build build-qp -j
./build-qp/test_qp_backend
```

---

## 10. 下一步验证计划

按优先级：

1. 在 RT 内核 + `performance` governor 下复测 `bench_tick`（确认 max）
2. 接入真实几何与 URDF，跑 pose 模式端到端
3. 加入 TSAN 构建，验证适配层锁的正确性
4. 采集覆盖率（gcov/lcov）
5. 跨机网络故障注入（丢包、乱序、延迟抖动）
