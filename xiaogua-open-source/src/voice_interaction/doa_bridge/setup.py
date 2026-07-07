from setuptools import setup

package_name = 'doa_ros1_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/doa_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'websocket-client'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Bridge DOA (sound source localization) from ROS2 to ROS1 via rosbridge',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = doa_ros1_bridge.bridge_node:main',
        ],
    },
)
