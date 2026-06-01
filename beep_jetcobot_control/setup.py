from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'beep_jetcobot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config'), 
        glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='sebin5736@gmail.com',
    description='JetCobot control and vision nodes',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_control = beep_jetcobot_control.joint_control:main',
            'pick_place = beep_jetcobot_control.pick_place:main',
            'aruco_detector = beep_jetcobot_control.aruco_detector:main',
        ],
    },
)
