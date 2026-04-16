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
    description='ROS2 bridge between the ESP32 binary UART protocol and ROS2 topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_bridge = esp32_bridge.main_bridge:main',
            'servo_bridge = esp32_bridge.servo_bridge:main',
            'audio_node = esp32_bridge.audio_node:main',
            'rc_control = esp32_bridge.rc_control:main',
            'gst_sender = esp32_bridge.gst_sender:main'
        ],
    },
)
