.. _getting_started:

Getting Started
=====================


Single Criticality Measure
---------------------------------

This tutorial demonstrates how to use CommonRoad-CriMe with signal
criticality measure. Please make sure that you have correctly installed
the package and have gone through the tutorials for CommonRoad
Input-Output beforehand.

Import and Building Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Computing the criticality requires a configuration that hold essential
parameters of the measure. The configuration is often built from default
parameters (located in ``./config_files/defaults/``) and
scenario-specific parameters. If you wish to overwrite a parameter for a
specific scenario, simply create a .yaml file with the name of the
scenario under ``./config_files/`` and store the parameter in it.

.. code-block:: python 

    from commonroad_crime.data_structure.configuration import CriMeConfiguration    from commonroad_crime.data_structure.crime_interface import CriMeInterface
    from commonroad_crime.measure import TTCStar

    # Specify the scenario
    scenario_id = "ZAM_Urban-7_1_S-2" 

    # Build the configuration
config = CriMeConfiguration.load(f"../config_files/{scenario_id}.yaml", scenario_id)    config.update()
    config.print_configuration_summary()

Compute the criticality
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In this tutorial, we should the evaluation of the ``time-to-collision``
with given predication, which we call ``TTCStar``. The set-based
prediction of other traffic participants is given.

.. code-block:: python

    evaluator = TTCStar(config) 
    time_step = 0 
    other_veh_id = 99
    evaluator.compute(time_step, other_veh_id)

Visualization of the evaluation result
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

    config.debug.save_plots = False 
    evaluator.visualize()

Evasive maneuver to avoid the collision?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The TTC and its variants do not provide enough information for collision
avoidance, as they do not include possible evasive maneuvers. To address
this limitation, the time-to-react (TTR) is proposed as the latest
possible time before the TTC, at which an evasive maneuver still exists.

.. code-block:: python

    from commonroad_crime.measure import TTR

    evaluator = TTR(config) 
    evaluator.compute(time_step, other_veh_id)
    evaluator.visualize()

Multiple Criticality Measures
---------------------------------

This tutorial demonstrates how to use CommonRoad-CriMe to evaluate the
criticality of a considered ego vehicle given a CommonRoad
scenario/scene and multiple measures. Please make sure that you have
correctly gone through the tutorial Nr. 0.

Import and Building Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.crime_interface import CriMeInterface
    from commonroad_crime.measure import (HW, TTC, TTR, ALongReq, LongJ, BTN, P_MC, PF)

    # Specify the scenario

    scenario_id = "DEU_Gar-1_1_T-1"

    # Build the configuration

config = CriMeConfiguration.load(f"../config_files/{scenario_id}.yaml", scenario_id)    config.update()
    config.print_configuration_summary()

Compute the criticality with various measures
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Invoking criticality measurement is done via the ``CriMeInterface``
class. We should the evaluation process with the following exemplary
metrics: - ``HW``: Headway - ``TTC``: Time-To-Collision - ``TTR``:
Time-To-React - ``ALongReq``: Required Longitudinal Acceleration -
``LongJ``: Longitudinal Jerk - ``LatJ``: Lateral Jerk - ``BTN``: Brake
Threat Number - ``STN``: Steer Threat Number - ``P_MC``: Collision
Probability via Monte Carlo Simulation - ``PF``: Potential Functions as
Superposition of Scoring Functions

1. Evaluation on Scene (first time step)
""""""""""""""""""""""""""""""""""""""""

.. code-block:: python

    crime_interface = CriMeInterface(config)
    crime_interface.evaluate_scene([HW, TTC, TTR, ALongReq, LongJ, BTN, P_MC, PF],)

2. Evaluation on Scenario
""""""""""""""""""""""""""

.. code-block:: python

    ts_start = 0 # starting time step 
    ts_end = 10 # ending time step
    crime_interface.evaluate_scenario([HW, TTC, TTR, ALongReq, LongJ, BTN, P_MC, PF], ts_start, ts_end)

Visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Evaluation results for debugging and showcasting
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
.. code-block:: python

    select_ts = 0 
    config.debug.save_plots = False
    crime_interface.visualize(select_ts)

2. Criticality curves
""""""""""""""""""""""
.. code-block:: python

    import commonroad_crime.utility.visualization as utils_vis

    utils_vis.plot_criticality_curve(crime_interface)
