# 命令头 0x020D
# 长度 6 (不是5)
# 哨兵决策数据

# bit 0-10: 标远程兑换充能量、哨兵机器人成功兑换的允许发弹量 (11位)
# 开启为0，成功兑换后为实际值
uint16 allow_bullet_count

# bit 11-14: 哨兵机器人成功远程兑换允许发弹量的次数 (4位)
# 开启为0，成功兑换后递增
uint8 exchange_bullet_count

# bit 15-18: 哨兵机器人成功远程兑换血量的次数 (4位)
# 开启为0，成功兑换后递增
uint8 exchange_hp_count

# bit 19: 哨兵机器人当前是否可以确认免费复活
# 1:可以确认免费复活, 0:不可以
bool free_revive_available

# bit 20: 哨兵机器人当前是否可以兑换立即复活
# 1:可以兑换立即复活, 0:不可以
bool immediate_revive_available

# bit 21-30: 哨兵机器人当前兑换即时复活需要花费的金币数 (10位)
uint16 immediate_revive_coin_cost

# --- offset 4, 2字节 ---

# bit 0 (sentry_info_2): 哨兵当前是否处于脱战状态
bool is_out_of_combat

# bit 1-11 (sentry_info_2): 队伍 17mm 允许发弹量的剩余可兑换数
uint16 remaining_exchangeable_17mm

# bit 12-13 (sentry_info_2): 哨兵当前姿态 (2位)
# 1:进攻姿态, 2:防御姿态, 3:移动姿态
uint8 sentry_attitude

# bit 14 (sentry_info_2): 己方能量机关是否能够被进入在激活状态
# 1:当前可激活, 0:不可激活
bool energy_gear_available
