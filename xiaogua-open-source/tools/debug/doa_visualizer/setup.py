from setuptools import setup

package_name = 'doa_visualizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/doa_visualizer.launch.py']),
        ('share/' + package_name + '/config', ['config/default.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Visualize reSpeaker XVF3800 DOA on TF tree and as RVIZ Marker arrow.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'doa_visualizer_node = doa_visualizer.visualizer_node:main',
        ],
    },
)