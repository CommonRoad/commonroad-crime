from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface
from commonroad_crime.measure import TTCStar, ET

# ==== specify scenario
scenario_id = "ZAM_Tjunction-1_97_T-1"
#scenario_id = "DEU_Moabit-4_1_T-1"
#scenario_id = "DEU_Moelln-4_4_T-1"
#scenario_id = "HRV_Pula-27_3_T-1"

# ==== build configuration
config = ConfigurationBuilder.build_configuration(scenario_id)
config.update()
config.print_configuration_summary()
config.vehicle.ego_id = 1
evaluator = ET(config)
print(evaluator.compute(5))


from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface

# ==== specify scenario
#scenario_id = "ZAM_Tjunction-1_97_T-1"
#scenario_id = "DEU_Moabit-4_1_T-1"
#scenario_id = "DEU_Moelln-4_4_T-1"
scenario_id = "HRV_Pula-27_3_T-1"

# ==== build configuration
config = ConfigurationBuilder.build_configuration(scenario_id)
config.update()
config.print_configuration_summary()
config.vehicle.ego_id = 338
config.print_configuration_summary()
evaluator = PSD(config)

time_step = 0
obstacles = evaluator.sce.dynamic_obstacles
for i in obstacles:
    print(i.obstacle_id)
    other_veh_id = i.obstacle_id
    evaluator.compute(other_veh_id, time_step)