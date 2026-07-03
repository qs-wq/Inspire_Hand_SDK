# TouchData1.msg（RH56DFX 无触觉硬件，接口占位，订阅可见但无数据）
std_msgs/Header header
int32[5] finger_forces   # 法向力
int32[5] finger_tangentials # 切向力
int32[5] finger_angles   # 切向力方向（顺时针360度，无接触时返回65535）
int32[5] finger_proximity    # 接近觉（0-16000000）
int32[9] palm_data    # palm_data1-3掌左，palm_data4-6掌中，palm_data7-9掌右
