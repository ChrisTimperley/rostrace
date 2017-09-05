#!/usr/bin/env python
from setuptools import setup

setup(
    name='rostrace',
    version='2.0.1',
    description='Allows detailed logging and debugging of running ROS systems',
    long_description='TBA',
    # need to modify to have multiple authors!
    author='Chris Timperley, Afsoon Afzal',
    author_email='christimperley@gmail.com, afsoona@cs.cmu.edu',
    # TODO: move to Squares organisation
    url='https://github.com/ChrisTimperley/rostrace',
    license='mit',
    packages=['rostrace'],
    entry_points = {
        'console_scripts': [ 'rostrace = rostrace.rostrace:main' ]
    }
)
