# 力控夹取：hand_id=1, speed=600, force=800
ros2 service call /gripper/force_mode_grasp eg5cd1_interfaces/srv/ForceModeGrasp "{hand_id: 1, speed: 600, force: 800}"

# 力控张开：force 为负
ros2 service call /gripper/force_mode_open eg5cd1_interfaces/srv/ForceModeOpen "{hand_id: 1, speed: 600, force: -400}"

# 触控夹取
ros2 service call /gripper/touch_mode_grasp eg5cd1_interfaces/srv/TouchModeGrasp "{hand_id: 1, speed: 600, force: 800}"

# 触控张开（力度仍为正，由寄存器区分）
ros2 service call /gripper/touch_mode_open eg5cd1_interfaces/srv/TouchModeOpen "{hand_id: 1, speed: 600, force: 800}"
