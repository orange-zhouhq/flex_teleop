# 关节目标（旁路 IK 的入口）。
# 对应 docs/04-interface.md § JointTarget。
#
# 与 pose 通路**共享同一 pose_id 序列空间**，保证模式切换不断档。

uint64 pose_id
builtin_interfaces/Time stamp

uint8 SIDE_LEFT = 0
uint8 SIDE_RIGHT = 1
uint8 side

# 关节目标（rad）
float64[7] q
