import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/keisuke-kawanishi/robotics/public/mobile_robot_system/install/zed_heading_publisher'
