from setuptools import find_packages
from setuptools import setup

setup(
    name='localization_package',
    version='0.0.0',
    packages=find_packages(
        include=('localization_package', 'localization_package.*')),
)
