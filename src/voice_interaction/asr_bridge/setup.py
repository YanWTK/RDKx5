from setuptools import setup

package_name = 'asr_ros1_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websocket-client'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Bridge ASR results from ROS2 to ROS1 via rosbridge',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = asr_ros1_bridge.bridge_node:main',
            'tts_host_node = asr_ros1_bridge.tts_host_node:main',
            'persona_control_node = asr_ros1_bridge.persona_control_node:main',
        ],
    },
)
