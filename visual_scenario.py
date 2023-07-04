import matplotlib.pyplot as plt
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams
from commonroad_crime.measure import TTCStar, PSD
import matplotlib.ticker as ticker
def only_ego_and_other(scenario, ego_id, other_id):
    to_be_deleted_obstacles = []
    for i in scenario.dynamic_obstacles:
        if (i.obstacle_id != ego_id  and i.obstacle_id != other_id):
            to_be_deleted_obstacles.append(i)
    scenario.remove_obstacle(to_be_deleted_obstacles)
# generate path of the file to be opened
scenario_name = 'BEL_Putte-8_2_T-1'
#scenario_name = 'HRV_Pula-27_3_T-1'
#scenario_name = 'HRV_Pula-27_3_T-1'
file_path = './scenarios/' + scenario_name + '.xml'
# read in the scenario and planning problem set


config = ConfigurationBuilder.build_configuration(scenario_name)
config.update()
config.print_configuration_summary()
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
print(len(scenario.dynamic_obstacles))
only_ego_and_other(scenario, 349, other_id=32)
print(len(scenario.dynamic_obstacles))
# plot the planning problem and the scenario for the fifth time step
plt.figure(figsize=(25, 20))
rnd = MPRenderer()
rnd.draw_params.dynamic_obstacle.show_label = True
rnd.draw_params.lanelet_network.draw_ids = [14121, 14377]
#rnd.draw_params.lanelet_network.lanelet.show_label = True
rnd.draw_params.time_begin = 0
scenario.draw(rnd)
#planning_problem_set.draw(rnd)
rnd.render()
plt.show()