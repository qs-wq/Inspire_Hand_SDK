# 百分比设置命令（0.0~100.0，超范围自动裁剪）
# 复用于 位置/速度/力/电流 的百分比设置话题；具体寄存器由控制器配置决定
# 位置百分比：换算后写入 angleSet（非 posSet）；0%=握紧、100%=张开
std_msgs/Header header
int32 hand_id
float32[6] joint_values
