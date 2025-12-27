import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kunal-jazzy/vision_autonomous_rover/install/rover_control'
