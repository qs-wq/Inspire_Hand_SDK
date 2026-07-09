# 角度设置（RH56H1 手册表35，单位 0.1°；角度越大越张开）
# 四指(0-3): 870~1690 或 -1(不动作)；拇指弯曲(4): 950~1350 或 -1；拇指旋转(5): 700~1700
# 注意：2000 是 posSet(电缸位置)上限，不是 angleSet；超范围时驱动会自动裁剪
std_msgs/Header header
int32 hand_id
int32[6] joint_values
