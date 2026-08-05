from setuptools import setup

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml'],
    ),
    (
        'share/' + package_name + '/launch',
        ['launch/sim.launch.py'],
    ),
    (
        'share/' + package_name + '/config',
        [],
    ),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gauri',
    maintainer_email='your_email@example.com',
    description='Obstacle Avoidance Drone Framework',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_detector = obstacle_avoidance.obstacle_detector:main',
            'radar_node = obstacle_avoidance.radar_node:main',
            'camera_detector_node = obstacle_avoidance.camera_detector_node:main',
            'lidar_detector_node = obstacle_avoidance.lidar_detector_node:main',
            'lidar_sector_node = obstacle_avoidance.lidar_sector_node:main',
            'fusion_node = obstacle_avoidance.fusion_node:main',
            'avoidance_node = obstacle_avoidance.avoidance_node:main',
            'mavlink_obstacle_node = obstacle_avoidance.mavlink_obstacle_node:main',
            'mission_commander_node = obstacle_avoidance.mission_commander_node:main',
        ],
    },
)