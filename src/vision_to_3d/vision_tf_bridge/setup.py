from glob import glob
from setuptools import setup

package_name = 'vision_ros1_tf_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'websocket-client'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dev@todo.com',
    description='Bridge ROS2 vision PointStamped targets into ROS1 TF via rosbridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_tf_bridge_node = vision_ros1_tf_bridge.vision_tf_bridge_node:main',
            'selected_detection_bridge_node = vision_ros1_tf_bridge.selected_detection_bridge_node:main',
            'patrol_memory_bridge_node = vision_ros1_tf_bridge.patrol_memory_bridge_node:main',
            'ros1_tf_to_ros2_bridge_node = vision_ros1_tf_bridge.ros1_tf_to_ros2_bridge_node:main',
            'fetch_task_bridge_node = vision_ros1_tf_bridge.fetch_task_bridge_node:main',
        ],
    },
)
