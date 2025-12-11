from setuptools import find_packages
from setuptools import setup

setup(
    name='nav_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('nav_interfaces', 'nav_interfaces.*')),
)
