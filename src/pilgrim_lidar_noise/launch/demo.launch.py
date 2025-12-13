import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

'''
   Launches Pilgrim demo.
'''
def generate_launch_description():
    pkg_path = get_package_share_directory('pilgrim_lidar_noise')
    world_path = os.path.join(pkg_path, 'worlds', 'demo.world')
    models_path = os.path.join(pkg_path, 'models')

    # link Gazebo to planetary assets
    set_gzbo = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # ------- rover LiDAR planetary storm demo (in Gazebo classic) ------ #
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',    # initialize ROS 2
            '-s', 'libgazebo_ros_factory.so'  # enable rover spawning
        ],
        output='screen'
    )
    
    return LaunchDescription([
        set_gzbo,
        gazebo
    ])
