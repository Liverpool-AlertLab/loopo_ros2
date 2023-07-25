from setuptools import setup

package_name = "openmanipulator_x_loopo"

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
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "experiment_control = openmanipulator_x_loopo.scripts.experiment_control:main"
        ],
    },
)
