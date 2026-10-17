from setuptools import setup

package_name = "go2_keyboard_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Keyboard teleoperation node for Unitree Go2 sport mode.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "keyboard_control = go2_keyboard_control.keyboard_control:main",
        ],
    },
)
