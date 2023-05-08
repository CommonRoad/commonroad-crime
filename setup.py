#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='commonroad-crime',
      version='0.2.3',
      description='criticality measures of automated vehicles',
      keywords="criticality, autonomous driving",
      author='Yuanfei Lin, Oliver Specht, Ivana Peneva',
      author_email='yuanfei.lin@tum.de',
      license='BSD 3-Clause',
      packages=find_packages(),
      install_requires=[
          'commonroad-io>=2022.3',
          'commonroad-vehicle-models>=3.0.0',
          'commonroad-route-planner>=2022.3',
          'commonroad-drivability-checker>=2022.2',
          'matplotlib>=3.5.2',
          'numpy>=1.19.0',
          'scipy>=1.7.3',
          'shapely<3.0.0,>=2.0.1',
          'omegaconf>=2.1.1',
          'casadi>=3.5.5, <3.6.0',
      ],
      extras_require={
          'tests': [
              'pytest>=7.1'
          ]
      })

