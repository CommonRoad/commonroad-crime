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
import math
from commonroad.scenario.scenario import Tag

from commonroad_crime.measure.time.pet import PET
from commonroad.scenario.obstacle import DynamicObstacle

import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
from commonroad_crime.utility.visualization import TUMcolor
import matplotlib.pyplot as plt

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
        self.ci_config = config.index.ci
        self._pet_object = PET(config)
        self.v_x, self.v_y = None, None

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        
        # Only for scenarios with intersection tag
        if (Tag.INTERSECTION not in self.sce.tags) or (len(self.sce.lanelet_network.intersections) == 0):
            utils_log.print_and_log_info(logger, f"*\t\t Measure only for intersection. CI is set to 0.")
            self.value = 0.
            return self.value
        if not isinstance(self.other_vehicle, DynamicObstacle):
            utils_log.print_and_log_info(logger,
                                         f"*\t\t Vehicle {vehicle_id} "
                                         f"is not a dynamic obstacle, CI is set to 0")
            self.value = 0.
            return self.value
        
        # Alpha determines to which degree released collision energy affects vehicle occupants.
        alpha = self.ci_config.alpha

        # Beta depends on conflict type and surrounding collisions 
        beta = self.ci_config.beta

        # Standard value for mass of vehicles 1500kg
        if self.configuration.vehicle.dynamic.parameters.m is not None:
            m1 = self.configuration.vehicle.dynamic.parameters.m
        else:
            m1 = self.ci_config.m
        if self.configuration.velocity.m_b:
            # todo: need to be refactored together with the dataclass-based configuration
            m2 = self.configuration.velocity.m_b
        else:
            # assume that the vehicle b has the same mass as the ego vehicle
            m2 = m1

        # Use PET object
        pet = self._pet_object.compute(vehicle_id, time_step)
        
        # Threshold of 5.0 as standard in the paper
        threshold = self.ci_config.pet_threshold
        
        # As PET = math.inf in case of no conflict area for PET, no separate check is needed
        if pet >= threshold:
            utils_log.print_and_log_info(logger, f"*\t\t As the PET is beyond the threshold,"
                                                 f" the CI is 0 for the ego vehicle and vehicle {vehicle_id}")
            value = 0.
            return value
        
        # Find the collision time and the conflict area with the help of the PET object
        state_ego = self.ego_vehicle.state_at_time(self._pet_object.ego_vehicle_enter_time)
        state_other = self.other_vehicle.state_at_time(self._pet_object.other_vehicle_enter_time)

        self.v_x = (m1 * state_ego.velocity + m2 * state_other.velocity) / (m1 + m2)
        self.v_y = (m1 * state_ego.velocity_y + m2 * state_other.velocity_y) / (m1 + m2)
        v = math.sqrt(self.v_x ** 2 + self.v_y ** 2)
        
        u1 = math.sqrt(state_ego.velocity ** 2 + state_ego.velocity_y ** 2)
        u2 = math.sqrt(state_other.velocity ** 2 + state_other.velocity_y ** 2)

        k_delta = 1 / 2 * (m1 * u1 ** 2) + 1 / 2 * (m2 * u2 ** 2) - 1 / 2 * (m1 + m2) * v ** 2
        self.value = utils_gen.int_round(alpha * k_delta / math.exp(beta * pet), 2)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value
        
    def visualize(self, figsize: tuple = (25, 15)):
        if self._pet_object.ca is None:
            utils_log.print_and_log_info(logger, "* \t\tNo conflict area")
            return
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(self.time_step,
                                                                self.ego_vehicle.prediction.
                                                                trajectory.state_list,
                                                                margin=50)
        save_sce = self.sce
        self._pet_object.sce_without_ego_and_other()
        self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
        self.rnd.render()
        self.sce = save_sce
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step::5],
                                  color=TUMcolor.TUMlightgray, linewidth=1, start_time_step=0)
        utils_vis.draw_state_list(self.rnd, self.other_vehicle.prediction.trajectory.state_list[self.time_step::5],
                                  color=TUMcolor.TUMlightgray, linewidth=1, start_time_step=0)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMblack, alpha=1)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMgreen, alpha=1)

        plt.title(f"{self.measure_name} of {self.value} time steps")
        x_i, y_i = self.ca.exterior.xy
        plt.plot(x_i, y_i, color=TUMcolor.TUMblack, zorder=1001)
        plt.fill(x_i, y_i, color=TUMcolor.TUMred, zorder=1001)
        state_ego = self.ego_vehicle.state_at_time(self._pet_object.ego_vehicle_enter_time)
        state_other = self.other_vehicle.state_at_time(self._pet_object.other_vehicle_enter_time)
        centroid = self._pet_object.ca.centroid
        x = centroid.x
        y = centroid.y
        plt.arrow(x=x, y=y, dx=state_ego.velocity, dy=state_ego.velocity_y,
                  head_width=0.4, width=0.1, ec='black')
        plt.arrow(x=x, y=y, dx=state_other.velocity, dy=state_other.velocity_y,
                  head_width=0.4, width=0.1, ec='green')
        plt.arrow(x=x, y=y, dx=self.v_x, dy=self.v_y, head_width=0.4, width=0.1, ec='red')

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()
    """           
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
            plt.arrow(x=x, y=y, dx=self.v_x, dy=self.v_y, head_width = 0.4, width = 0.1, ec ='red')


        
        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()
        """