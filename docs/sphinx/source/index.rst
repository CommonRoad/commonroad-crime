.. CommonRoad-CriMe documentation master file, created by
   sphinx-quickstart on Wed Jun 7 19:18:19 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

CommonRoad-CriMe
================
Automotive manufacturers must ensure that autonomous
vehicles can recognize and effectively handle unexpected
situations. To effortlessly measure and compare the criticality of an
autonomous vehicle, we present the novel CommonRoad **CriMe**
(Criticality Measures) toolbox, which:

- provides a framework in Python with unified notations, vehicle models, and coordinate systems for criticality measures;
- adopts and supplements the categorization of criticality measures defined in `this collection <criticality-metrics.readthedocs.io/>`_;
- is open-source and allows users to easily modify, add, and compare criticality measures;
- offers efficient and reliable computation by bridging to powerful scenario evaluation tools.

.. seealso::
   * `CommonRoad Input-Output <https://commonroad.in.tum.de/commonroad-io>`_
   * `CommonRoad Drivability Checker <https://commonroad.in.tum.de/drivability-checker>`_
   * `CommonRoad Reach <https://commonroad.in.tum.de/tools/commonroad-reach>`_
   * `CommonRoad Scenario Designer <https://commonroad.in.tum.de/tools/scenario-designer>`_

Installation
===============

commonroad-crime can be installed with::

    pip install commonroad-crime

For adding new measures, we recommend using `Anaconda <https://www.anaconda.com/>`_ to manage your environment so that even if you mess something up, you can always have a safe and clean restart. A guide for managing python environments with Anaconda can be found `here <https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html>`_.

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

To test the installation, run unittest::

    cd tests
    python -m unittest -v


Overview
===============

.. toctree::
   :maxdepth: 2

   user/index.rst
   subpackages/index

Citation
===================
.. code-block:: text

   @InProceedings{lin2023crime,
         title     = {{CommonRoad-CriMe}: {A} Toolbox for Criticality Measures of Autonomous Vehicles},
         author    = {Yuanfei Lin and Matthias Althoff},
         booktitle = {Proc. of the IEEE Intell. Veh. Symp.},
         year      = {2023},
   }


Contact information
===================

.. only:: html

    :Release: |release|
    :Date: |today|

:Website: `http://commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
:Forum: `CommonRoad forum <https://commonroad.in.tum.de/forum/c/commonroad-crime/20>`_
:Email: `commonroad@lists.lrz.de <commonroad@lists.lrz.de>`_
