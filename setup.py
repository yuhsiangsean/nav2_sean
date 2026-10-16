from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nav2_sean'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='a0929370607@gmail.com',
    description=(
        'Nav2 to PX4 Offboard bridge for indoor (no-GPS) minimal SITL '
        'navigation testing. Ubuntu 22.04 / ROS 2 Humble port of '
        'px4_nav2_bridge, with map.x=e/map.y=n/map.z=-d instead of '
        'map.x=n/map.y=-e/map.z=-d.'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = nav2_sean.cmd_vel_bridge:main',
            'px4_odom_tf = nav2_sean.px4_odom_tf:main',
        ],
    },
)
