from setuptools import find_packages, setup

package_name = 'jaguar_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gerardo',
    maintainer_email='gerardodelcid16@gmail.com',
    description='Joystick teleoperation, serial bridge, and launch files for the Dicerox 6-DOF arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_tk = jaguar_teleop.teleop_tk:main',
            'keyboard_servo = jaguar_teleop.keyboard_servo:main',
            'joystick_servo = jaguar_teleop.joystick_servo:main',
            'serial_bridge = jaguar_teleop.serial_bridge:main',
        ],
    },
)
