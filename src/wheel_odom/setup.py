import os
from glob import glob
from setuptools import setup

package_name = 'wheel_odom'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('wheel_odom', 'launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Wheel encoder odometry for Raspberry Pi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_publisher = wheel_odom.encoder_publisher:main',
            'encoder_odometry = wheel_odom.encoder_odometry:main',
        ],
    },
)