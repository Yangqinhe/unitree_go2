from glob import glob

from setuptools import setup


package_name = "go2_front_video_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Convert Unitree Go2 front video stream messages into standard ROS image topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "front_video_bridge_node = go2_front_video_bridge.front_video_bridge_node:main",
        ],
    },
)
