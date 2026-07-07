from setuptools import find_packages, setup


package_name = "robopilot_app_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/robopilot_app.launch.py"]),
        (
            "share/" + package_name + "/config",
            [
                "config/robopilot_app.yaml",
                "config/10-robopilot-xhci-noise.conf",
                "config/99-robopilot-journal.conf",
            ],
        ),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@localhost",
    description="Robopilot App-facing communication layer for ROS 2 hosts",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robopilot_app_bridge_node = robopilot_app_bridge.bridge_node:main",
            "robopilot_ros1_bridge_node = robopilot_app_bridge.ros1_bridge_node:main",
            "robopilot_mapping_service_node = robopilot_app_bridge.mapping_service_node:main",
            "robopilot_mock_camera_node = robopilot_app_bridge.mock_camera:main",
            "robopilot_mjpeg_server = robopilot_app_bridge.mjpeg_server:main",
            "cmd_vel_relay = robopilot_app_bridge.cmd_vel_relay:main",
        ],
    },
)
