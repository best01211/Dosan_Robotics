from setuptools import find_packages
from setuptools import setup

setup(
    name='dynamixel_sdk_custom_interfaces',
    version='3.8.4',
    packages=find_packages(
        include=('dynamixel_sdk_custom_interfaces', 'dynamixel_sdk_custom_interfaces.*')),
)
