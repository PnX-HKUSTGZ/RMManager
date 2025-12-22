# 命令头 0x0201
# 长度 13

# 本机器人 ID
uint8 robot_id
# 等级
uint8 level
# 当前血量
uint16 current_hp
# 血量上限
uint16 max_hp
# 射击热量冷却值/s
uint16 heat_cooling_rate
# 热量上限
uint16 heat_max
# 底盘功率上限
uint16 chassis_power_limit
# gimbal 口输出状态 (1: 24V输出)
uint8 gimbal_power
# chassis 口输出状态 (1: 输出)
uint8 chassis_power
# shooter 口输出状态 (1: 输出)
uint8 shooter_power
