#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='commonroad-crime',
      version='0.0.1',
      description='criticality measure of automated vehicles',
      keywords="criticality, autonomous driving",
      author='yuanfei Lin',
      author_email='yuanfei.lin@tum.de',
      license='GPLv3',
      packages=find_packages(),
      install_requires=[
          'commonroad-io>=2021.4,<2022.2',
          'commonroad-drivability-checker>=2021.4',
          'commonroad-vehicle-models>=1.0.0'
          'matplotlib>=3.5.2'
          'numpy>=1.19.5',
      ],
      extras_require={
          'tests': [
              'pytest>=7.1'
          ]
      })
