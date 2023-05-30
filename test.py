from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface
from commonroad_crime.measure import TTCStar, ET

# ==== specify scenario
#scenario_id = "ZAM_Tjunction-1_97_T-1"
#scenario_id = "DEU_Moabit-4_1_T-1"
#scenario_id = "DEU_Moelln-4_4_T-1"
#scenario_id = "HRV_Pula-27_3_T-1"
scenario_id = "DEU_Flensburg-10_1_T-1"

# ==== build configuration
config = ConfigurationBuilder.build_configuration(scenario_id)
config.update()
config.print_configuration_summary()
evaluator = ET(config)
ans = evaluator.compute(31)
print(ans)
config.debug.save_plots = False
evaluator.visualize()

