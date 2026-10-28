# 因时 RH56 灵巧手 —— 关节实际角度反馈
#
# joint_names[13] 是通道布局的权威来源；joint_values[13] 与之一一对应，
# 量纲与 SetAngle 相同：单位 0.1 度，通道定义见 SetAngle.msg。
string[13] joint_names
int32[13] joint_values
