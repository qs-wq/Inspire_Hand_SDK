# 百分比实际值（0.0~100.0）
# 用于角度实际值(angleAct)按各指角度范围换算成百分比后发布；0%=握紧、100%=张开
std_msgs/Header header
int32 hand_id
float32[6] joint_values
string[6] joint_names
