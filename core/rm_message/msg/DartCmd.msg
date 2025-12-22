# 命令头 0x020A
# 长度 6

# 飞镖站状态 (1:关闭, 2:开启中, 0:已开启)
uint8 dart_station_status
# 保留
uint8 reserved
# 切换击打目标倒计时
uint16 target_switch_countdown
# 最后发射指令倒计时
uint16 last_shoot_countdown
