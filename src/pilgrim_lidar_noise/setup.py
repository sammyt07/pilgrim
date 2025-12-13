from setuptools import setup

pkg_name = 'pilgrim_lidar_noise'

setup(
    name=pkg_name,
    version='0.1.0',
    packages=[pkg_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + pkg_name]),
        (f'share/{pkg_name}', ['package.xml']),
        (f'share/{pkg_name}/launch', ['launch/demo.launch.py']),
        (f'share/{pkg_name}/worlds', ['worlds/demo.world']),

        # --- install TurtleBot model sdfs --- #
        (f'share/{pkg_name}/models/turtlebot3_burger', [
            'models/turtlebot3_burger/model.sdf',
            'models/turtlebot3_burger/model-1_4.sdf',
            'models/turtlebot3_burger/model.config',
        ]),

        # --- install TurtleBot mesh deps --- #
        (f'share/{pkg_name}/models/turtlebot3_burger/meshes', [
            'models/turtlebot3_burger/meshes/burger_base.dae',
            'models/turtlebot3_burger/meshes/lds.dae',
            'models/turtlebot3_burger/meshes/tire.dae',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='freeman',
    maintainer_email='sammy.l.tribble@gmail.com',
    description='A ROS 2 LiDAR noise injection node with sample analysis.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'storm_lidar_node = pilgrim_lidar_noise.storm_lidar_node:main',
            'lidar_env_comp = pilgrim_lidar_noise.lidar_env_comp:main',
        ],
    },
)
