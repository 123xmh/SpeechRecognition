# EquipmentState.msg

# 弹药状态 (动态类型/数量)
string[] ammo_types      # 弹药类型 ["穿甲弹", "高爆弹", ...]
uint8[] ammo_counts      # 对应数量 [数量1, 数量2, ...]

# 光电转台状态 (弧度制)
float32 gimbal_pitch     # 俯仰角(弧度)
float32 gimbal_yaw       # 方位角(弧度)
bool gimbal_is_active    # 转台是否激活

# 车辆核心状态
float64 longitude        # 经度(WGS84)
float64 latitude         # 纬度(WGS84)
float64 altitude         # 海拔高度(米)
float64 ground_altitude  # 对地高度(米)(仅无人机有效，其他平台填0)
float32 heading          # 朝向(弧度，0为正北)
float32 speed            # 车速(km/h)
float32 fuel_level       # 油量(0.0-1.0)
float32 battery_level    # 电量(0.0-1.0)

# 车辆姿态 (三轴弧度)
float32 roll             # 横滚角(弧度)
float32 pitch            # 俯仰角(弧度)
float32 yaw              # 偏航角(弧度)

# 告警系统
string[] warnings        # 当前活动告警 ["引擎过热", "电池低电量", ...]

# 通用状态容器 (保留扩展性)
string[] state_names     # 其他状态名称
float32[] state_values   # 对应数值