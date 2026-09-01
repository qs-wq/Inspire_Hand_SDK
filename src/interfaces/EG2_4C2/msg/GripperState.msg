# EG2-4C2 夹爪状态（USB-CAN 串口 Modbus 地址 1060~1065 共 6 个寄存器；与 demo 中 STATUS_BASE 起 4 个状态寄存器 + error/status 寄存器一致）
std_msgs/Header header
int32 hand_id
int32 force_act
int32 open_len_act
int32 current_act
int32 temp_c
uint32 error_code
uint32 status
