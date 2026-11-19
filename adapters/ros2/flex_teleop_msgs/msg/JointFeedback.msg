# 关节反馈。对应 docs/04-interface.md § JointFeedback。
#
# tick_seq 必须与 JointStream 同拍配对；若驱动器频率低于 200Hz，
# 由适配层做零阶保持并标出实际对应的拍号。

uint64 tick_seq
builtin_interfaces/Time stamp

uint8 SIDE_LEFT = 0
uint8 SIDE_RIGHT = 1
uint8 side

# 实际关节角（rad）
float64[7] q
# 关节速度（rad/s）
float64[7] qd
# 关节力矩（N·m）；全 0 表示驱动器不提供
float64[7] tau
