import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dingo/Programming/tb3_autonomy/install/turtlebot3_example'
