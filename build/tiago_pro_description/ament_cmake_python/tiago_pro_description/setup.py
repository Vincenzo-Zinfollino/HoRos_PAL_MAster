from setuptools import find_packages
from setuptools import setup

setup(
    name='tiago_pro_description',
    version='2.4.1',
    packages=find_packages(
        include=('tiago_pro_description', 'tiago_pro_description.*')),
)
