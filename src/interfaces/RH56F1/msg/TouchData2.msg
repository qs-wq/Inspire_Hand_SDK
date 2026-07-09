# RH56H1 触觉 version2（压阻式）数据
# 手指顺序: pinky/ring/middle/index/thumb
# 每指: tip_end 指端 2*2=4 个 int16, tip_touch 指尖 6*5=30 个 int16, force 合力 x/y/z 三个 float32
# 掌心: palm_touch 15*6=90 个 int16
std_msgs/Header header

int16[] pinky_tip_end
int16[] pinky_tip_touch
float32 pinky_force_x
float32 pinky_force_y
float32 pinky_force_z

int16[] ring_tip_end
int16[] ring_tip_touch
float32 ring_force_x
float32 ring_force_y
float32 ring_force_z

int16[] middle_tip_end
int16[] middle_tip_touch
float32 middle_force_x
float32 middle_force_y
float32 middle_force_z

int16[] index_tip_end
int16[] index_tip_touch
float32 index_force_x
float32 index_force_y
float32 index_force_z

int16[] thumb_tip_end
int16[] thumb_tip_touch
float32 thumb_force_x
float32 thumb_force_y
float32 thumb_force_z

int16[] palm_touch
