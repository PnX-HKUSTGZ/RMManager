
# 命令头 0x0001
# 长度 11 

# 1 RoboMaster 机甲大师超级对抗赛 
# 2 RoboMaster 机甲大师高校单项赛 
# 3 ICRA RoboMaster高校人工智能挑战赛 
# 4 RoboMaster机甲大师高校联盟赛3V3对抗  
# 5 RoboMaster 机甲大师高校联盟赛步兵对抗 
uint8 game_type

# 0 未开始比赛 
# 1 准备阶段 
# 2 十五秒裁判系统自检阶段 
# 3 五秒倒计时 
# 4 比赛中 
# 5 比赛结算中 
uint8 game_stage

# 单位 s
uint16 rest_time
uint64 unix_time