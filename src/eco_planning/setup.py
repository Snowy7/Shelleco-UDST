from setuptools import find_packages, setup

package_name = 'eco_planning'

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
            'dashboard = eco_planning.dashboard_node:main',
            'state_machine = eco_planning.state_machine_node:main',
            'section1_planner = eco_planning.section1_planner_node:main',
            'section2_planner = eco_planning.section2_planner_node:main',
        ],
    },
)
