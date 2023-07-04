import matplotlib.pyplot as plt

# import functions to read xml file and visualize commonroad objects
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface
# generate path of the file to be opened
#scenario_name = 'BEL_Putte-8_2_T-1'
#scenario_name = 'HRV_Pula-27_3_T-1'
#scenario_name = 'HRV_Pula-27_3_T-1'
scenario_name = 'ZAM_Tjunction-1_97_T-1'

file_path = './scenarios/' + scenario_name + '.xml'
# read in the scenario and planning problem set
def only_ego_and_other(scenario, ego_id, other_id):
    to_be_deleted_obstacles = []
    for i in scenario.dynamic_obstacles:
        if (i.obstacle_id != ego_id  and i.obstacle_id != other_id):
            to_be_deleted_obstacles.append(i)
    scenario.remove_obstacle(to_be_deleted_obstacles)

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
# plot the scenario for 40 time step, here each time step corresponds to 0.1 second
#only_ego_and_other(scenario, 349, 328)
for i in range(0, 100, 5):
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    rnd.draw_params.dynamic_obstacle.show_label = True

    # set time step in draw_params
    rnd.draw_params.time_begin = i
    # plot the scenario at different time step
    scenario.draw(rnd)
    # plot the planning problem set
    #planning_problem_set.draw(rnd)
    rnd.render()
    plt.show()