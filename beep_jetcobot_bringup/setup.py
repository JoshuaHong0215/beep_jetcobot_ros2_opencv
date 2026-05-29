from setuptools import find_packages, setup

package_name = 'beep_jetcobot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='sebin5736@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_control = beep_jetcobot_bringup.joint_control:main',
            'pick_place = beep_jetcobot_bringup.pick_place:main',
            'aruco_detector = beep_jetcobot_bringup.aruco_detector:main'
        ],
    },
)
