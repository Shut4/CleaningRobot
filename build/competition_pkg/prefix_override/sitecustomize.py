import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ros2/ros2_lecture_ws/src/7_lectures/install/competition_pkg'
