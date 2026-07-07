from glob import glob
from setuptools import setup

package_name = 'object_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dev@todo.com',
    description='Lightweight ByteTrack-style tracker for ROS2 YOLO detections.',
    entry_points={
        'console_scripts': [
            'object_tracker_node = object_tracker.tracker_node:main',
        ],
    },
)
