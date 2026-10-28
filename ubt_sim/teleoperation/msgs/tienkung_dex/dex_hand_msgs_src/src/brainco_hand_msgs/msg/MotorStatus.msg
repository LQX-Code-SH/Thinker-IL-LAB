# 强脑 Revo2 灵巧手 —— 6 电机状态反馈
#
# sdk 侧只读取 positions（backends/real/hand.py::_on_status），
# 其余字段按驱动器常见上报补齐，仿真端暂不回填。
# 位置量纲：1 = 完全伸直，1000 = 完全弯曲。
int32[6] positions
int32[6] speeds
int32[6] currents
