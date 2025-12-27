import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.xml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kunal-jazzy',
    maintainer_email='kunal-jazzy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_avoidance=rover_control.obstacle_avoidance:main',
            'cmd_vel_switch=rover_control.cmd_vel_switch:main',
        ],
    },
)
