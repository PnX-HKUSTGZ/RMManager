# 命令头 0x0104
# 长度 3

# 判罚等级 (1:双方黄牌, 2:黄牌, 3:红牌, 4:判负)
uint8 penalty_level
# 犯规机器人 ID (如红1为1, 蓝1为101)
uint8 robot_id
# 对应判罚等级的违规次数
uint8 violation_count
