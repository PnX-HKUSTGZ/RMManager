# 命令头 0x0101
# 长度 4

# 己方与资源区不重叠的补给区占领状态 (1:已占领)
uint8 supply_area_1
# 己方与资源区重叠的补给区占领状态 (1:已占领)
uint8 supply_area_2
# 己方补给区占领状态 (仅RMUL)
uint8 supply_area_own
# 己方小能量机关状态 (0:未激活, 1:已激活, 2:正在激活)
uint8 small_energy_status
# 己方大能量机关状态 (0:未激活, 1:已激活, 2:正在激活)
uint8 large_energy_status
# 己方中央高地占领状态 (1:被己方占领, 2:被对方占领)
uint8 center_highland_status
# 己方梯形高地占领状态 (1:已占领)
uint8 trapezoid_highland_status
# 对方飞镖最后击中己方前哨站/时间 (0-420s)
uint16 dart_hit_time
# 对方飞镖最后击中的目标 (1:前哨站, 2:基地固定, 3:基地随机固定, 4:基地随机移动, 5:基地末端移动)
uint8 dart_target_type
# 中心增益点占领状态 (仅RMUL)
uint8 center_gain_status
# 己方堡垒增益点占领状态 (0:未占领, 1:己方占领, 2:对方占领, 3:双方占领)
uint8 fort_gain_status
# 己方前哨站增益点占领状态
uint8 outpost_gain_status
# 己方基地增益点占领状态
uint8 base_gain_status
