# 200Hz 关节流式目标。框架输出 A，对应 docs/04-interface.md § JointStream。
#
# 连续流：每拍必发，不"变化才发"。

uint64 tick_seq
builtin_interfaces/Time stamp

uint8 SIDE_LEFT = 0
uint8 SIDE_RIGHT = 1
uint8 side

# 关节指令（rad）
float64[7] q_cmd

# ── 溯源三元组：本拍对应的上游 pose 区间与区间内比例 ──────────────────
# 这是帧对齐的核心。单靠 pose_id 无法表达"两个源帧之间"。
uint64 id_a
uint64 id_b
float64 alpha

# ── 工作模式 ────────────────────────────────────────────────────────
uint8 MODE_POSE  = 0
uint8 MODE_JOINT = 1
uint8 MODE_HOLD  = 2
uint8 mode

# ── 本拍求解后端 ────────────────────────────────────────────────────
uint8 SOLVER_QP        = 0
uint8 SOLVER_ANALYTIC  = 1
uint8 SOLVER_NUMERICAL = 2
uint8 SOLVER_HOLD      = 3
uint8 solver
