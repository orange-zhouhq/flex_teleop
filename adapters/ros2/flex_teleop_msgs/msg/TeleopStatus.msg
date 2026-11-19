# 状态上报（10–50Hz）。框架输出 B，对应 docs/04-interface.md § TeleopStatus。
#
# 三个误差量必须分开：e_model 回答"算法对不对"，e_real 回答"系统到没到"，
# lead_q 回答"伺服跟不跟"。只报 e_model 会把伺服滞后误判为跟踪良好。

builtin_interfaces/Time stamp

uint8 SIDE_LEFT = 0
uint8 SIDE_RIGHT = 1
uint8 side

# 0 TRACKING / 1 HOLD / 2 DEGRADED / 3 FAULT
uint8 STATE_TRACKING = 0
uint8 STATE_HOLD     = 1
uint8 STATE_DEGRADED = 2
uint8 STATE_FAULT    = 3
uint8 state

# 同 JointStream.mode
uint8 mode

# 已完成下发的最大 pose_id。**不等于已执行到位**。
uint64 pose_id_ack

# 当前流的溯源三元组
uint64 id_a
uint64 id_b
float64 alpha

# r - FK(q_prev)
float64 e_model_pos_m
float64 e_model_rot_rad

# r - FK(q_fb)
float64 e_real_pos_m
float64 e_real_rot_rad

# q_cmd - q_fb
float64[7] lead_q

# 本拍求解耗时（微秒）
float64 solve_us
