import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aperture/pilgrim/src/pilgrim_lidar_noise/install/pilgrim_lidar_noise'
