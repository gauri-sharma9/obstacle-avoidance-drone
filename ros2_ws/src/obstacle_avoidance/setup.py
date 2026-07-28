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
        [],
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = obstacle_avoidance.obstacle_detector:main',
        ],
    },
)