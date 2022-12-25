#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='commonroad-crime',
      version='0.0.1',
      description='criticality measure of automated vehicles',
      keywords="criticality, autonomous driving",
      author='yuanfei Lin',
      author_email='yuanfei.lin@tum.de',
      license='BSD 3-Clause',
      packages=find_packages(),
      install_requires=[
          'commonroad-io>=2022.3',
          'commonroad-drivability-checker>=2022.2.1',
          'commonroad-vehicle-models>=2.0.0'
          'matplotlib>=3.5.2'
          'numpy>=1.19.5',
          'scipy==1.7.3',
          'shapely==1.7.1',
          'omegaconf>=2.1.1',
          'casadi>=3.5.5',
      ],
      extras_require={
          'tests': [
              'pytest>=7.1'
          ]
      })

