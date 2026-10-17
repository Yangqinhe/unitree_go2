from glob import glob

from setuptools import setup


package_name = "go2_local_costmap"

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
    description="Local point cloud costmap and target tracking helpers for Unitree Go2 person following.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "local_costmap_node = go2_local_costmap.local_costmap_node:main",
            "target_tracker_node = go2_local_costmap.target_tracker_node:main",
            "obstacle_marker_node = go2_local_costmap.obstacle_marker_node:main",
        ],
    },
)
