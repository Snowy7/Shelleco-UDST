from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'eco_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='snowydev7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detection = eco_perception.lane_detection_node:main',
            'stop_sign_detection = eco_perception.stop_sign_node:main',
            'obstacle_detection = eco_perception.obstacle_detection_node:main',
        ],
    },
)
