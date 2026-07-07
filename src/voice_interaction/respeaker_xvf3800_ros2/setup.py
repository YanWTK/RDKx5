from setuptools import find_packages, setup


package_name = "respeaker_xvf3800_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/respeaker_xvf3800.launch.py"]),
        ("share/" + package_name + "/launch", ["launch/respeaker_xvf3800_asr.launch.py"]),
        ("share/" + package_name + "/launch", ["launch/respeaker_xvf3800_wake.launch.py"]),
        ("share/" + package_name + "/config", ["config/default.yaml"]),
        ("share/" + package_name + "/config", ["config/wake_sensitive.yaml"]),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools", "openai>=1.52.0"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@localhost",
    description="ROS 2 driver and control wrapper for Seeed reSpeaker XVF3800",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "respeaker_xvf3800_node = respeaker_xvf3800_ros2.node:main",
            "respeaker_xvf3800_asr_node = respeaker_xvf3800_ros2.asr_node:main",
            "respeaker_xvf3800_wake_node = respeaker_xvf3800_ros2.wake_word_node:main",
            "record_xvf3800_audio = respeaker_xvf3800_ros2.record_audio_topic:main",
        ],
    },
)
