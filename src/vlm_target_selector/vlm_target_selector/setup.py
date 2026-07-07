from glob import glob
from setuptools import setup

package_name = 'vlm_target_selector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (
            'share/' + package_name,
            [
                'package.xml',
                'README.md',
                'BAILIAN_VLM_TEST_FLOW.md',
                'VLM_TRACKING_CHAIN.md',
                '记忆.md',
                '巡逻.md',
            ],
        ),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'openai>=1.35.0'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dev@todo.com',
    description='VLM target selector for numbered YOLO detections.',
    entry_points={
        'console_scripts': [
            'vlm_target_selector_node = vlm_target_selector.selector_node:main',
            'half_water_test_node = vlm_target_selector.half_water_test_node:main',
            'image_mjpeg_server_node = vlm_target_selector.image_mjpeg_server_node:main',
            'lost_reselector_node = vlm_target_selector.lost_reselector_node:main',
            'memory_target_select_adapter_node = vlm_target_selector.memory_target_select_adapter:main',
            'object_memory_query_node = vlm_target_selector.memory_query_node:main',
            'task_understanding_node = vlm_target_selector.task_understanding_node:main',
            'patrol_scan_adapter_node = vlm_target_selector.patrol_scan_adapter:main',
            'target_confirm_node = vlm_target_selector.target_confirm_node:main',
            'yolo_vlm_tracking_demo_node = vlm_target_selector.yolo_vlm_tracking_demo_node:main',
        ],
    },
)
