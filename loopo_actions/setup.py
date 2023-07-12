import os
from glob import glob
from setuptools import setup

package_name = 'loopo_actions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsamewolf',
    maintainer_email='lupo961@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "extension_actions_server = loopo_actions.extension_actions:main",
            "twist_actions_server = loopo_actions.twist_actions:main",
            "loop_actions_server = loopo_actions.twist_actions:main",
        ],
    },
)
