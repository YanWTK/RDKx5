from setuptools import setup
from glob import glob

package_name = 'vision_to_3d_local'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'STARTUP_FLOW.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dev@todo.com',
    description='将 2D 检测结果 + 深度图转换为相机局部 3D 坐标',
    entry_points={
        'console_scripts': [
            'vision_to_3d_local_node = vision_to_3d_local.vision_to_3d_local_node:main',
        ],
    },
)
