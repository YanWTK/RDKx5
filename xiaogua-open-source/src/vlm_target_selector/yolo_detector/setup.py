from setuptools import setup

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dev@todo.com',
    description='YOLOv8 BPU 检测节点，支持类别过滤，发布 ai_msgs/PerceptionTargets',
    entry_points={
        'console_scripts': [
            'yolo_detector_node = yolo_detector.detector_node:main',
        ],
    },
)
