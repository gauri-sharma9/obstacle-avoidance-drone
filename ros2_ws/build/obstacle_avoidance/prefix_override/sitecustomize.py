import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gauri/.gemini/antigravity/scratch/obstacle-avoidance-drone/ros2_ws/install/obstacle_avoidance'
