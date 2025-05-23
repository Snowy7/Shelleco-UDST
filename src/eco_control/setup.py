from setuptools import find_packages, setup

package_name = 'eco_control'

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
    maintainer='ubuntu',
    maintainer_email='snowydev7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steering = eco_control.steering_node:main',
            'motor = eco_control.motor_node:main',
            'lateral_controller = eco_control.lateral_controller_node:main',
            'longitudinal_controller = eco_control.longitudinal_controller_node:main',
        ],
    },
)
