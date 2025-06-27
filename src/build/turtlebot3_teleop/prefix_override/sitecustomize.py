import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dingo/tb3_autonomy/src/install/turtlebot3_teleop'
