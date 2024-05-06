# 定义消息常量
uint8 MAINTAIN_VELOCITY = 0  # 路径模式为保持加速度
uint8 REACH_POINT = 1            # 路径模式为到达固定点

# 消息通用内容
Header header                # 消息头
CurvePoint[] points          # 路径点，包括生成路径和延展路径
float64 point_margin         # 路径点之间的间隔（单位为米）
uint8 mode                   # 路径模式，分为了MAINTAIN_VELOCITY和REACH_POINT两种
int32 vehicle_position_index # 车辆中心点当前处于路径上对应的位置
bool reverse_allowing        # 是否允许倒车

# 模式MAINTAIN_VELOCITY下的特定内容
float64 expected_acceleration    # 期望保持的加速度
float64 velocity_limitation_max  # 能够到达的最高速度
float64 velocity_limitation_min  # 能够到达的最低速度

# 模式REACH_POINT下的特定内容
float64 goal_velocity  # 目标点的速度
int32 goal_index     # 目标点在路径中的下标
 