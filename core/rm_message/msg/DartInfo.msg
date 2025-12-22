# 命令头 0x0105
# 长度 3

# 己方飞镖发射剩余时间（秒）
uint8 dart_remaining_time
# 最近一次击中目标类型 (1-5同FieldEvents)
uint8 target_type
# 对方最近被击中累计次数 (0-4次)
uint8 hit_count
# 飞镖此时选定的打击目标 (0:前哨站/未选, 1:基地固定, 2:基地随机固定, 3:随机移动, 4:末端移动)
uint8 selected_target
