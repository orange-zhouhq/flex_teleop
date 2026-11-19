# 转换后的目标位姿（基座系）。
# 对应 docs/04-interface.md § PoseFrame。
#
# 由坐标变换节点产出；消费方只用 pose_id 判定新旧，stamp 仅作诊断。

# 源头分配的帧身份。单调递增，全链只读，任何环节不得重生。
uint64 pose_id

# 采集侧时间。仅用于延迟诊断，不参与任何对齐判定。
builtin_interfaces/Time stamp

# 0 = left, 1 = right
uint8 SIDE_LEFT = 0
uint8 SIDE_RIGHT = 1
uint8 side

# 4x4 齐次变换，**行主序**：t_base_tcp[row * 4 + col]
float64[16] t_base_tcp

# 夹爪映射（透传，不参与控制）
float64 pinch
