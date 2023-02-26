#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='commonroad-crime',
      version='0.0.3',
      description='criticality measures of automated vehicles',
      keywords="criticality, autonomous driving",
      author='yuanfei Lin',
      author_email='yuanfei.lin@tum.de',
      license='GPLv3',
      packages=find_packages(),
      install_requires=[
          'commonroad-io>=2021.4,<2022.2',
          'commonroad-drivability-checker==2021.4',
          'commonroad-vehicle-models>=1.0.0'
          'matplotlib>=3.5.2'
          'numpy>=1.23.0',
          'scipy>=1.7.3',
          'shapely==1.7.1',
          'omegaconf>=2.1.1',
          'casadi>=3.5.5',
      ],
      extras_require={
          'tests': [
              'pytest>=7.1'
          ]
      })

