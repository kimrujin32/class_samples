import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robot/rh_plus_7_axis_robot_arm/install/rh_plus_7_axis_robot_arm_description'
