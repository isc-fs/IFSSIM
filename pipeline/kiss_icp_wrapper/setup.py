from setuptools import setup

package_name = "kiss_icp_wrapper"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul Moran",
    maintainer_email="raul@isc-fs.com",
    description="ROS 2 LifecycleNode wrapper for the kiss-icp Python pipeline.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Lifecycle node. ROS name "kiss_icp_node" matches the entry in
            # AUTONOMY_LIFECYCLE_NODES (mode_manager). Subscribes /lidar/Lidar1
            # (remapped from /fsds/lidar/Lidar1 by pipeline.launch.py) only in
            # active state; publishes /odom_lidar.
            "kiss_icp_node = kiss_icp_wrapper.kiss_icp_node:main",
        ],
    },
)
