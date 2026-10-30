# 大寰 PGC-140-50 夹爪状态 / 大寰电缸状态
# 话题: /ecat/left_grip/state, /ecat/right_grip/state

shm_msgs/Header header   # 时间戳

uint16 init_state        # 1: 初始化成功
uint16 grip_state        # 0: 运动中  1: 到位  2: 夹持中  3: 掉落
uint16 error_code        # 错误码
uint16 homed             # 1: 已回零

float64 pos              # 实际位置, 单位: m
float64 vel              # 实际速度, 单位: m/s
float64 cur              # 实际电流, 单位: A
