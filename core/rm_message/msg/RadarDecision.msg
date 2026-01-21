# 命令头 0x020E
# 长度 1
# 雷达决策数据

# bit 0-1: 雷达是否拥有双倍易伤确认的机会 (2位)
# 开启为0，数值为雷达拥有双倍易伤确认的机会，至多为2
uint8 double_vulnerable_count

# bit 2: 对方是否正在被雷达双倍易伤
# 0: 对方未被雷达双倍易伤
# 1: 对方正在被雷达双倍易伤
bool enemy_vulnerable

# bit 3-4: 己方加密等级（即对方干扰雷达等级）(2位)
# 开启为1，最高为3
uint8 encryption_level

# bit 5: 当前是否可以修改密钥
# 1: 可修改, 0: 不可修改
bool can_modify_key
