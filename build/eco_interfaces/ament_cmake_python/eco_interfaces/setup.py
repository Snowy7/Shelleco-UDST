from setuptools import find_packages
from setuptools import setup

setup(
    name='eco_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('eco_interfaces', 'eco_interfaces.*')),
)
