# 大寰 PGC-140-50 夹爪控制指令 / 大寰电缸控制指令
# 话题: /ecat/left_grip/cmd, /ecat/right_grip/cmd
#
# === 大寰夹爪使用方法 ===
#   整机回零时夹爪同步回零，之后给位置(行程)和力(夹持力)即可
#   pos   范围 [0, 0.05]   单位 m
#   force 范围 [0, 100]    单位 N
#   vel   范围 [0, 0.75]   单位 m/s    (注：demo 实际用 <=0.1 m/s)
#
# === 大寰电缸使用方法 ===
#   非推压模式: mode=0  给位置(行程) + 力(推压力) [+ 速度/加速度]
#   推压模式:   mode=10 给推压行程 + 力(推压力) [+ 速度/加速度]
#   备注: cur 字段在电缸推压模式下被复用为加速度 acc
#   pos 范围 [0, 0.05]    单位 m
#   force 范围 [41, 100]  单位 N
#   vel 范围 [0, 0.01]    单位 m/s
#   acc 范围 [0, 3]       单位 m/s^2  (复用 cur 字段)

shm_msgs/Header header   # 时间戳

uint16 init              # 1: 有效
uint16 mode              # 0: position/velocity/torque  10: 推压模式(电缸)
uint16 stop              # 1: 停止  0: 可运动
uint16 reset             # 1: 有效
uint16 homing            # 1: 发送回零

float64 pos              # 目标位置, 单位: m
float64 vel              # 目标速度, 单位: m/s
float64 force            # 目标力,   单位: N
float64 cur              # 目标电流, 单位: A   (电缸推压模式下复用为加速度 m/s^2)
