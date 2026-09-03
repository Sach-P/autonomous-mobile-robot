from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sachin',
    maintainer_email='sachpatel1220@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_pose_publisher = robot_behavior.goal_pose_publisher:main',
            'goal_client = robot_behavior.goal_client:main',
            'stop_service = robot_behavior.stop_service:main',
            'keyboard_stop_client = robot_behavior.keyboard_stop_client:main'
        ],
    },
)
