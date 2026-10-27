# 强脑 Revo2 灵巧手 —— 6 电机位置指令
#
# 字段布局按 xhumanoid_sdk 的 backends/real/hand.py 实测用法重建：
#   msg.mode / msg.positions[i] / msg.speeds[i] / msg.currents[i]
#   msg.pwms[i] / msg.durations[i]（每项 6 个电机）
# 位置量纲：1 = 完全伸直，1000 = 完全弯曲。
int32 mode
int32[6] positions
int32[6] speeds
int32[6] currents
int32[6] pwms
int32[6] durations
