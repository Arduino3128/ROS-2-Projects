from setuptools import setup

package_name = "uav_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="atticus",
    maintainer_email="atticus@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = uav_controller.controller:main",
            "server_controller = uav_controller.UAV.server_controller:main",
            "server = uav_controller.UAV.server:main",
        ],
    },
)
