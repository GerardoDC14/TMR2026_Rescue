from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'esp32_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyaudio'],
    zip_safe=True,
    maintainer='arepo',
    maintainer_email='arepo90@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge_node = esp32_bridge.esp32_bridge_node:main',
            'audio_node = esp32_bridge.audio_node:main',
        ],
    },
)
