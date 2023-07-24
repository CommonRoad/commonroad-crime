.. CommonRoad-CriMe documentation master file, created by
   sphinx-quickstart on Wed Jun  7 19:18:19 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

CommonRoad-CriMe
================

The commonroad-crime package is a toolbox to compute **Cr**\iticality  **Me**\asures(e.g. time-to-collision, time-to-react,...). Such measures can be used to trigger warnings and emergency maneuvers in driver assistance systems or repair an infeasible trajectory. 

Installation
============

commonroad-crime can be installed with::

	pip install commonroad-crime

For adding new measures, we recommend using [Anaconda](https://www.anaconda.com/) to manage your environment so that even if you mess something up, you can always have a safe and clean restart. A guide for managing python environments with Anaconda can be found [here](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

After installing Anaconda, create a new environment with::

	conda create -n commonroad-py37 python=3.7 -y

Here the name of the environment is called commonroad-py37. You may also change this name as you wish. In such case, don't forget to change it in the following commands as well. Always activate this environment before you do anything related::

	conda activate commonroad-py37
	
or ::

	source activate commonroad-py37
	
Then, install the dependencies with::

	cd <path-to-this-repo>
	pip install -e .
	conda develop .

**(optional)** In order to run reachability analysis related measures 
(e.g., worst-time-to-react (WTTR), drivable area (DA)), you have to install the following repositories as well:
`commonroad-reach <https://gitlab.lrz.de/tum-cps/commonroad-reach>`__

To test the installition, run unittest::

	cd tests
	python -m unittest -v

This will install related dependencies specified in `requirements.txt`. Or simply install the dependencies listed in `requirements.txt` and add this repository to your python path.


Getting Started
===============

A tutorial on the main functionalities of the project is :ref:`available here<getting_started>`.

.. toctree::
   :maxdepth: 3
   :caption: Contents:
   
   user/index.rst
   subpackages/index.rst


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Contact information
===================

:Website: `http://commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
:Email: `commonroad@lists.lrz.de <commonroad@lists.lrz.de>`_
