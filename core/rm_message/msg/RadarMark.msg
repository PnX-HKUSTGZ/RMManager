# 命令头 0x020C
# 长度 2
# 雷达标记进度数据

# bit 0: 对方 1 号英雄机器人易伤/标识
bool enemy_hero_vulnerable
# bit 1: 对方 2 号工程机器人易伤/标识
bool enemy_engineer_vulnerable
# bit 2: 对方 3 号步兵机器人易伤/标识
bool enemy_infantry3_vulnerable
# bit 3: 对方 4 号步兵机器人易伤/标识
bool enemy_infantry4_vulnerable
# bit 4: 对方空中机器人特殊标识/情况
bool enemy_aerial_special_mark
# bit 5: 对方哨兵机器人易伤/标识
bool enemy_sentry_vulnerable
# bit 6: 己方 1 号英雄机器人特殊标识/情况
bool own_hero_special_mark
# bit 7: 己方 2 号工程机器人特殊标识/情况
bool own_engineer_special_mark
# bit 8: 己方 3 号步兵机器人特殊标识/情况
bool own_infantry3_special_mark
# bit 9: 己方 4 号步兵机器人特殊标识/情况
bool own_infantry4_special_mark
# bit 10: 己方空中机器人特殊标识/情况
bool own_aerial_special_mark
# bit 11: 己方哨兵机器人特殊标识/情况
bool own_sentry_special_mark
