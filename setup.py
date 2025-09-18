from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot4_mujoco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'assets/meshes'), glob('assets/meshes/*')),
        (os.path.join('share', package_name), glob('*.xml')),
    ],
    install_requires=['setuptools', 'numpy', 'mujoco'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 package for Turtlebot4 simulation with MuJoCo',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb4_ros2_node = turtlebot4_mujoco.tb4_ros2_node:main',
        ],
    },
)
