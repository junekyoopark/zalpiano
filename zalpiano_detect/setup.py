from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zalpiano_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='junekyoopark@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detect = zalpiano_detect.aruco_detect:main',
            'aruco_to_center = zalpiano_detect.aruco_to_center:main',
            'image_publisher = zalpiano_detect.image_publisher:main',
            'goal_pose_publisher = zalpiano_detect.goal_pose_publisher:main',
        ],
    },
)
