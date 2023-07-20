from setuptools import setup

package_name = "loopo_driver"

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
    maintainer="itsamewolf",
    maintainer_email="lupo961@gmail.com",
    description="TODO: Package description",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "loopo_driver_node = loopo_driver.loopo_driver_node:main",
            "loopo_action_servers_node = loopo_driver.loopo_action_servers:main",
        ]
    },
)
