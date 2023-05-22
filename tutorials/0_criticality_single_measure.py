#!/usr/bin/env python
# coding: utf-8

# # Tutorial: Single Criticality Measure
# This tutorial demonstrates how to use CommonRoad-CriMe with signal criticality measure. Please make sure that you have correctly installed the package and have gone through the tutorials for CommonRoad Input-Output beforehand.

# ## Import and Building Configuration
# Computing the criticality requires a configuration that hold essential parameters of the measure. The configuration is often built from default parameters (located in `./config_files/defaults/`) and scenario-specific parameters. If you wish to overwrite a parameter for a specific scenario, simply create a .yaml file with the name of the scenario under `./config_files/` and store the parameter in it.

# In[1]:


from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface
from commonroad_crime.measure import TTCStar

# ==== specify scenario
scenario_id = "ZAM_Urban-7_1_S-2"

# ==== build configuration
config = ConfigurationBuilder.build_configuration(scenario_id)
config.update()
config.print_configuration_summary()


# ## Compute the criticality
# 
# In this tutorial, we should the evaluation of the `time-to-collision` with given predication, which we call `TTCStar`. The set-based prediction of other traffic participants is given.

# In[2]:


evaluator = TTCStar(config)
time_step = 0
other_veh_id = 99
evaluator.compute(time_step, other_veh_id)


# ## Visualization of the evaluation result

# In[3]:


config.debug.save_plots = False
evaluator.visualize()


# ## Evasive maneuver to avoid the collision?
# The TTC and its variants do not provide enough information for collision avoidance, as they do not include possible evasive maneuvers. To address this limitation, the time-to-react (TTR)
# is proposed as the latest possible time before the TTC, at which an evasive maneuver still exists.

# In[4]:


from commonroad_crime.measure import TTR

evaluator = TTR(config)
evaluator.compute(time_step, other_veh_id)
evaluator.visualize()


# In[ ]:





# In[ ]:




