#!/usr/bin/python2.7
from setuptools import setup

setup(
    name='rostrace',
    version='0.0.1',
    description='Converts ROS bag files into Daikon traces and declarations',
    long_description='TBA',
    # need to modify to have multiple authors!
    author='Afsoon Afsal, Chris Timperley',
    author_email='afsafzal@cs.cmu.edu, christimperley@gmail.com',
    # TODO: move to Squares organisation
    url='https://github.com/ChrisTimperley/rostrace',
    license='mit',
    packages=['rostrace'],
    entry_points = {
        'console_scripts': [ 'rostrace = rostrace.rostrace:main' ]
    }
)
