#!/usr/bin/env python

import os

from setuptools import setup, find_packages

from commonroad_crime.__version__ import __version__


this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, "README.md"), "r", encoding="utf-8") as f:
    readme = f.read()


setup(
    name="commonroad-crime",
    version=__version__,
    description="criticality measures of automated vehicles",
    keywords="criticality, autonomous driving",
    long_description_content_type="text/markdown",
    long_description=readme,
    author="Technical University of Munich",
    author_email="commonroad@lists.lrz.de",
    license="BSD 3-Clause",
    project_urls={
        "Documentation": "https://cps.pages.gitlab.lrz.de/commonroad-criticality-measures/",
        "Forum": "https://commonroad.in.tum.de/forum/c/commonroad-crime/20",
        "Source": "https://gitlab.lrz.de/tum-cps/commonroad-crime",
    },
    url="https://commonroad.in.tum.de/tools/commonroad-crime",
    packages=find_packages(),
    data_files=[(".", ["LICENSE"])],
    install_requires=[
        "commonroad-io>=2023.4",
        "commonroad-vehicle-models>=3.0.0",
        "commonroad-route-planner>=2022.3,<2024.1",
        "commonroad-drivability-checker>=2023.1",
        "commonroad-reach>=2023.1",
        "matplotlib>=3.5.2",
        "numpy>=1.19.0",
        "scipy>=1.7.3",
        "shapely<3.0.0,>=2.0.1",
        "omegaconf>=2.1.1",
        "casadi>=3.6.3",
        "tqdm>=4.65.0",
        "imageio>=2.9.0",
        "pytest>=7.4.0",
    ],
    classifiers=[
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: OSI Approved :: BSD License",
        "Operating System :: POSIX :: Linux",
    ],
    include_package_data=True,
    package_data={"": ["*.yaml"]},
)
