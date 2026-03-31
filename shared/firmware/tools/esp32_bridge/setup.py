from setuptools import setup
import os
from glob import glob

package_name = 'esp32_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'esp32_bridge_node = esp32_bridge.esp32_bridge_node:main',
        ],
    },
)
