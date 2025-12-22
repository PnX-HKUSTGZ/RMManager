# 命令头 0x0207
# 长度 7

# 弹丸类型
# bit 1: 17mm, bit 2: 42mm
uint8 ammo_type
# 发射机构 (1: 17mm机构, 3: 42mm机构)
uint8 shoot_mechanism
# 射速 (Hz)
uint8 shoot_speed
# 初速度 (float, m/s)
float32 initial_velocity
