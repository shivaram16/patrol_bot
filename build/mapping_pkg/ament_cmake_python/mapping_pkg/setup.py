from setuptools import find_packages
from setuptools import setup

setup(
    name='mapping_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('mapping_pkg', 'mapping_pkg.*')),
)
