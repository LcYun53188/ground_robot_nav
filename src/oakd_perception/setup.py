from setuptools import find_packages, setup

package_name = "oakd_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/oakd_unified.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "oakd_depth_node = oakd_perception.oakd_depth_node:main",
            "oakd_imu_node = oakd_perception.oakd_imu_node:main",
            "oakd_unified_node = oakd_perception.oakd_unified_node:main",
            "traversable_depth_filter = oakd_perception.traversable_depth_filter:main",
        ],
    },
)
