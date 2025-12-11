from setuptools import find_packages
from setuptools import setup

setup(
    name='calculator_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('calculator_interfaces', 'calculator_interfaces.*')),
)
