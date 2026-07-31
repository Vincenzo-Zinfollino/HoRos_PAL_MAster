import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pick_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools'
        'rclpy',
        'geometry_msgs',
        'shape_msgs',
        'moveit_msgs',
        'tf2_ros',
        'gazebo_msgs',
       ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_spawner = pick_task.scene_spawner:main',
            'main_task_node = pick_task.main_task_node:main',
            

        ],
    },
)
