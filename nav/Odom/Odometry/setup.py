from setuptools import setup

package_name = 'Odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='susanaakemi',
    maintainer_email='susy.akemi.campos@gmail.com',
    description='EKF odometry node with IMU and encoder fusion',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odometry = Odometry.odometry:main',
        ],
    },
)