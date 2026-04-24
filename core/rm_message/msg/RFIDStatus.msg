# 命令头 0x0209
# 长度 5
# 说明: 所有字段都是位标志(0或1)，除非特别说明

# bit 0: 己方基地增益点 RFID 卡
bool base_gain
# bit 1: 己方中央高地增益点
bool own_center_highland
# bit 2: 对方中央高地增益点
bool enemy_center_highland
# bit 3: 己方梯形高地增益点
bool own_trapezoid_highland
# bit 4: 对方梯形高地增益点
bool enemy_trapezoid_highland

# bit 5-8: 己方/对方地形跨越增益点（飞坡）4位位图
# bit 5: 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
bool terrain_crossing_ramp_own_near_front
# bit 6: 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
bool terrain_crossing_ramp_own_near_back
# bit 7: 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
bool terrain_crossing_ramp_enemy_near_front
# bit 8: 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
bool terrain_crossing_ramp_enemy_near_back

# bit 9-12: 地形跨越增益点（中央高地）4位位图
# bit 9: 己方地形跨越增益点（中央高地下方）
bool terrain_crossing_highland_own_below
# bit 10: 己方地形跨越增益点（中央高地上方）
bool terrain_crossing_highland_own_above
# bit 11: 对方地形跨越增益点（中央高地下方）
bool terrain_crossing_highland_enemy_below
# bit 12: 对方地形跨越增益点（中央高地上方）
bool terrain_crossing_highland_enemy_above

# bit 13-16: 地形跨越增益点（公路区）4位位图
# bit 13: 己方地形跨越增益点（公路下方）
bool terrain_crossing_road_own_below
# bit 14: 己方地形跨越增益点（公路上方）
bool terrain_crossing_road_own_above
# bit 15: 对方地形跨越增益点（公路区下方）
bool terrain_crossing_road_enemy_below
# bit 16: 对方地形跨越增益点（公路区上方）
bool terrain_crossing_road_enemy_above

# bit 17: 己方堡垒增益点
bool own_fort
# bit 18: 己方前哨站增益点
bool own_outpost
# bit 19: 己方与资源区不重叠的补给区 / RMUL 补给区
bool own_supply_area_non_overlap
# bit 20: 己方与资源区重叠的补给区
bool own_supply_area_overlap
# bit 21: 己方装配增益点
bool own_assembly_gain
# bit 22: 对方装配增益点
bool enemy_assembly_gain
# bit 23: 中央增益点（仅 RMUL 适用）
bool center_rmul
# bit 24: 对方堡垒增益点
bool enemy_fort
# bit 25: 对方前哨站增益点
bool enemy_outpost

# bit 26: 己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
bool own_road_below
# bit 27: 己方地形跨越增益点（隧道）（靠近己方一侧公路区中间）
bool own_road_middle
# bit 28: 己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
bool own_road_above
# bit 29: 己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
bool own_trapezoid_low
# bit 30: 己方地形跨越增益点（隧道）（靠近己方梯形高地较中间）
bool own_trapezoid_middle
# bit 31: 己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）
bool own_trapezoid_high

# offset 4, bit 0: 对方地形跨越增益点（隧道）（靠近对方公路一侧下方）
bool enemy_road_below
# offset 4, bit 1: 对方地形跨越增益点（隧道）（靠近对方公路一侧中间）
bool enemy_road_middle
# offset 4, bit 2: 对方地形跨越增益点（隧道）（靠近对方公路一侧上方）
bool enemy_road_above
# offset 4, bit 3: 对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
bool enemy_trapezoid_low
# offset 4, bit 4: 对方地形跨越增益点（隧道）（靠近对方梯形高地较中间）
bool enemy_trapezoid_middle
# offset 4, bit 5: 对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
bool enemy_trapezoid_high
