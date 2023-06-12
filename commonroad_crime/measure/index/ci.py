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
        state_list_ego = self.ego_vehicle.prediction.trajectory.state_list
        state_list_other = self.other_vehicle.prediction.trajectory.state_list
        
        # Only for scenarios with intersection tag
        if Tag.INTERSECTION not in self.sce.tags:
            utils_log.print_and_log_info(logger, f"* Measure only for intersection. CI is set to 0.")
            self.value = 0
            return self.value
        
        # In paper, assumption that released collision energy affects vehicle occupants
        alpha = 1.0

        # Since the conflict index is only estimated for one type of conflict, β is assumed to be 1.0. (paper)
        # beta depends on conflict type and surrounding collisions
        beta = 1.0

        # 1500kg cars as in the paper
        m1 = 1500
        m2 = 1500

        # Use PET object
        pet = self._pet_object.compute(vehicle_id, time_step)
        print("PET")
        print(pet)
        if pet >= 5:
            value = 0
            return value
        else:

            # Find whether a conflict area exists first: Check whether the two vehicles overlap at a certain point in time
            

            intersection = None
            coll_time_ego = -1
            coll_time_other = -1
            for i in range(time_step, len(state_list_ego)):
                
                ego_poly=self.ego_vehicle.occupancy_at_time(i).shape.shapely_object
                for j in range(time_step, len(state_list_other)):
                    if self.other_vehicle.occupancy_at_time(j) is not None:
                        #print("OOOO")
                        other_poly = self.other_vehicle.occupancy_at_time(j).shape.shapely_object
                        #print(other_poly)
                        if ego_poly.intersects(other_poly):
                            self.intersection = ego_poly.intersection(other_poly)
                            coll_time_ego = i
                            coll_time_other = j
                            break
                else:
                    continue
                break


            # Without trajectory collision, CI is zero
            if coll_time_ego == -1:
                utils_log.print_and_log_info(logger, f"* No trajectory intersection found between ego vehicle and vehicle id {vehicle_id}")
                value = 0
                return value
            
            state_ego = self.ego_vehicle.state_at_time(coll_time_ego)
            state_other = self.other_vehicle.state_at_time(coll_time_other)
            
            
            
            self.v_x = (m1 * state_ego.velocity + m2 * state_other.velocity)/(m1+m2)
            self.v_y = (m1 * state_ego.velocity_y + m2 * state_other.velocity_y)/(m1+m2)
            v = math.sqrt(self.v_x**2 +self.v_y**2)
            
            u1 = math.sqrt(state_ego.velocity ** 2 + state_ego.velocity_y ** 2)
            u2 = math.sqrt(state_other.velocity ** 2 + state_other.velocity_y ** 2)

            k_delta = 1/2*(m1 * u1**2) + 1/2*(m2 * u2**2) - 1/2*(m1+m2) * v**2
            self.value = alpha * k_delta / math.exp(beta * pet)

            # Save parameters for visualization
            self.cte = coll_time_ego
            self.cto = coll_time_other

            return self.value

        
    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=utils_vis.
                             plot_limits_from_state_list(self.time_step,
                                                         self.ego_vehicle.prediction.trajectory.state_list,
                                                         margin=50))
        self.rnd.render()
        if self.intersection is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
            return 0

        plt.title(f"{self.measure_name} of {self.value} m")
        
        for i in range(self.cte+1):            
            x_i, y_i = self.ego_vehicle.occupancy_at_time(i).shape.shapely_object.exterior.xy
            plt.plot(x_i, y_i, color='blue')
            plt.fill(x_i, y_i, color='blue')
            
        for i in range(self.cto+1):            
            x_i, y_i = self.other_vehicle.occupancy_at_time(i).shape.shapely_object.exterior.xy
            plt.plot(x_i, y_i, color='orange')
            plt.fill(x_i, y_i, color='orange')
            
        self.other_vehicle.occupancy_at_time(i).shape.shapely_object
        if self.intersection.geom_type == "Polygon":
            x_i, y_i = self.intersection.exterior.xy
            plt.plot(x_i, y_i, color='red')
            plt.fill(x_i, y_i, color='red')
            centroid = self.intersection.centroid
            x = centroid.x
            y = centroid.y
            print(self.v_y)
            plt.arrow(x=x, y=y, dx=self.v_x, dy=self.v_y, head_width = 0.4, width = 0.1, ec ='red')


        
        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()
