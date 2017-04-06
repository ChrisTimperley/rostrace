#!/usr/bin/python2.7
from setuptools import setup

setup(
    name='rosbagtotrace',
    version='0.0.1',
    description='Converts ROS bag files into Daikon traces and declarations',
    long_description='TBA',
    # need to modify to have multiple authors!
    author='Afsoon Afsal',
    # TODO: add author email and repo URL
    #author_email='
    #url='https://github.com/ChrisTimperley/roshammer',
    license='mit',
    packages=['rosbagtotrace'],
    entry_points = {
        'console_scripts': [ 'rosbagtotrace = rosbagtotrace.rosbagtotrace:main' ]
    }
)
