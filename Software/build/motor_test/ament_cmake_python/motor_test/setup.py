from setuptools import find_packages
from setuptools import setup

setup(
    name='motor_test',
    version='0.0.0',
    packages=find_packages(
        include=('motor_test', 'motor_test.*')),
)
