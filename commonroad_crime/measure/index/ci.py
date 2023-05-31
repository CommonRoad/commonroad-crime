__author__ = "Yuanfei Lin and Kun Qian"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndex, TypeMonotone
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol
import math
from commonroad.scenario.scenario import Tag

from commonroad_crime.measure.time.pet import PET

import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor
import numpy as np
import matplotlib.pyplot as plt
from typing import Union
from commonroad.visualization.mp_renderer import MPRenderer

logger = logging.getLogger(__name__)

class CI(CriMeBase):
    """
    W. K. Alhajyaseen, “The integration of conflict probability and severity for the safety assessment of intersections”,
    Arabian Journal for Science and Engineering, vol. 40, no. 2, pp. 421–430, 2015.
    """
    measure_name = TypeIndex.CI
    monotone = TypeMonotone.POS


    def __init__(self, config: CriMeConfiguration):
        super(CI, self).__init__(config)
        self._pet_object = PET(config)


    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step
        state_list_ego = self.ego_vehicle.prediction.trajectory.state_list
        state_list_other = self.other_vehicle.prediction.trajectory.state_list
        
        
        
        # Only for scenarios with intersection tag
        if Tag.INTERSECTION not in self.sce.tags:
            utils_log.print_and_log_info(logger, f"* Measure only for intersection. CI is set to 0.")
            self.value = 0
            return self.value
        


        # In paper, assumption that released collision energy affects vehicle occupants
        alpha = 1.0

        #Since the conflict index is only estimated for one type of conflict, β is assumed to be 1.0. (paper)
        # beta depends on conflict type and surrounding collisions
        beta = 1.0

        # 1500kg cars as in the paper
        m1 = 1500
        m2 = 1500

        # Use in case of existing PET
        # pet = self._pet_object.compute(vehicle_id, time_step)
        pet = 2.0

        if pet >= 5:
            self.value = 0
        else:
            coll_time_ego = -1
            coll_time_other = -1

            # Find whether a conflict area exists first: Check whether the two vehicles overlap at a certain point in time
            for i in range(time_step, len(state_list_ego)):
                ego_poly=self.ego_vehicle.occupancy_at_time(i).shape.shapely_object
                for j in range(time_step, len(state_list_other)):
                    if self.other_vehicle.occupancy_at_time(j) is not None:
                        other_poly = self.other_vehicle.occupancy_at_time(j).shape.shapely_object
                        if ego_poly.crosses(other_poly):
                            coll_time_ego = i
                            coll_time_other = j
                            break
                else:
                    continue
                break


            # Without trajectory collision, CI is zero
            if coll_time_ego == -1:
                utils_log.print_and_log_info(logger, f"* No trajectory intersection found between ego vehicle and vehicle id {vehicle_id}")
                self.value = 0
                return self.value


            # We use lanelet orientation for current vehicle orientation
            
            ego_lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([self.ego_vehicle.state_at_time(coll_time_ego).
                                                                       position])[0]
            other_lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([self.other_vehicle.state_at_time(coll_time_other).
                                                                       position])[0]

            
            ego_orientation = utils_sol.compute_lanelet_width_orientation(
                self.sce.lanelet_network.find_lanelet_by_id(ego_lanelet_id[0]),
                self.ego_vehicle.state_at_time(time_step).position
            )[1]
            
            other_orientation = utils_sol.compute_lanelet_width_orientation(
                self.sce.lanelet_network.find_lanelet_by_id(other_lanelet_id[0]),
                self.other_vehicle.state_at_time(time_step).position
            )[1]
            theta1 = ego_orientation
            theta2 = other_orientation
            
            state_ego = self.ego_vehicle.state_at_time(coll_time_ego)
            state_other = self.other_vehicle.state_at_time(coll_time_other)
            
            
            
            v_x = (m1 * state_ego.velocity + m2 * state_other.velocity)/(m1+m2)
            v_y = (m1 * state_ego.velocity_y + m2 * state_other.velocity_y)/(m1+m2)
            v = math.sqrt(v_x**2 +v_y**2)
            
            ###ORIGINAL
            """
            
            u1 = math.sqrt(state_ego.velocity ** 2 + state_ego.velocity_y ** 2)
            u2 = math.sqrt(state_other.velocity ** 2 + state_other.velocity_y ** 2)

            # Fix bug for theta1=theta2=0
            var1 = (m1 * u1 * math.sin(theta1) + m2 * u2 * math.sin(theta2))
            var2 = (m1 * u1 * math.cos(theta1) + m2 * u2 * math.cos(theta2))
            denom =    (m1+m2) * math.sin(math.atan2(var1, var2 ) )
            epsilon = float(denom == 0) * 1e-10
            
            v = var1    / (denom + epsilon  )
            """
            k_delta = 1/2*(m1 * u1**2) + 1/2*(m2 * u2**2) - 1/2*(m1+m2) * v**2


            self.value = alpha * k_delta / math.exp(beta * pet)
            return self.value

    def visualize(self): #, figsize: tuple = (25, 15)):
        self._initialize_vis()#figsize=figsize,
                             #plot_limit=utils_vis.
                             #plot_limits_from_state_list(self.time_step,
                             #self.ego_vehicle.prediction.trajectory.state_list,
                             #                            margin=10))
        self.rnd.draw_params.dynamic_obstacle.show_label = True
        self.rnd.render()
        
        utils_vis.draw_reference_path(self.rnd, np.array(self.clcs.reference_path()))
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)
        other_veh_ids = [veh.obstacle_id for veh in self.sce.obstacles
                         if veh.obstacle_id is not self.ego_vehicle.obstacle_id]
        for v_id in other_veh_ids:
            ci = self.compute(time_step=0, vehicle_id=v_id)
            if ci > 0.0:
                self.set_other_vehicles(v_id)
                utils_vis.draw_state_list(self.rnd, self.other_vehicle.prediction.trajectory.state_list[self.time_step:],
                             color=TUMcolor.TUMred,linewidth = 5)

        plt.title(f"{self.measure_name} of {self.value} m")



        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()
        












"""
    def visualize(self, figsize: tuple = (25, 15), plot_limit: Union[list, None] = None):
        if plot_limit is None:
            plot_limit = self.configuration.debug.plot_limits
        self.rnd = MPRenderer(figsize=figsize, plot_limits=plot_limit)
        self.rnd.draw_params.dynamic_obstacle.show_label = True
        utils_vis.draw_sce_at_time_step(self.rnd, self.configuration, self.sce, self.time_step)
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)
        other_veh_ids = [veh.obstacle_id for veh in self.sce.obstacles
                         if veh.obstacle_id is not self.ego_vehicle.obstacle_id]
        for v_id in other_veh_ids:
            ci = self.compute(time_step=0, vehicle_id=v_id)
            if ci > 0.0:
                self.set_other_vehicles(v_id)
                utils_vis.draw_state_list(self.rnd, self.other_vehicle.prediction.trajectory.state_list[self.time_step:],
                             color=TUMcolor.TUMred,linewidth = 5)
       
        self.rnd.render()

        plt.title(f"{self.measure_name} of {self.value} m")

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()

"""