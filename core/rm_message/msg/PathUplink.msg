# 命令头 0x0307
# 长度 103

# 意图 (1:攻, 2:守, 3:移)
uint8 intention
# 起点X
float32 start_x
# 起点Y
float32 start_y
# X轴增量数组 (49字节)
uint8[49] delta_x
# Y轴增量数组 (49字节)
uint8[49] delta_y
