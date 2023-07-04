__author__ = "Yuanfei Lin, Ziqian Xu"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
from typing import Union
import shapely.ops
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle
from commonroad.scenario.lanelet import Lanelet
from shapely.geometry import Polygon
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeDistance, TypeMonotone
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.solver as utils_sol
import matplotlib.pyplot as plt
import logging
import numpy as np
import commonroad_crime.utility.general as utils_gen
from commonroad_crime.utility.visualization import TUMcolor
from commonroad_crime.measure.distance.msd import MSD
from commonroad_crime.measure.time.et import ET
from commonroad.geometry.shape import ShapeGroup
from commonroad.geometry.polyline_util import compute_polyline_lengths, compute_polyline_intersections, is_point_on_polyline, compute_total_polyline_length


logger = logging.getLogger(__name__)


class PSD(CriMeBase):
    """
    See https://criticality-metrics.readthedocs.io/
    """
    metric_name = TypeDistance.PSD
    measure_name = TypeDistance.PSD
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(PSD, self).__init__(config)
        self._msd_object = MSD(config)
        self._et_object = ET(config)
   

    def compute(self, vehicle_id: int = None, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} beginning at time step {time_step}")
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        other_vehicle = self.sce.obstacle_by_id(vehicle_id)
        self._et_object.other_vehicle = other_vehicle
        self._et_object.time_step = time_step
        self.value = None
        state_list = self.ego_vehicle.prediction.trajectory.state_list[self.time_step:]
        intersected_points = None
        #compute MSD
        msd = self._msd_object.compute(vehicle_id, time_step)
        if msd == 0:
            utils_log.print_and_log_info(logger, f"*\t\t msd is zero")
            return math.inf
        if msd == math.inf:
            utils_log.print_and_log_info(logger, f"*\t\t msd is infinity")
            return 0
        if isinstance(self.other_vehicle, DynamicObstacle):
            ca = self._et_object.get_ca()
            if ca is Polygon:
                xi, yi = ca.exterior.xy
                ca_exterior_pline_t = np.vstack((xi, yi))
                ca_exterior_pline = ca_exterior_pline_t.transpose()
                pos = np.asarray([state.position for state in state_list])
                intersected_points = compute_polyline_intersections(ca_exterior_pline, pos)
                if intersected_points is not None and intersected_points.shape[0] == 1:
                    self.ca = ca
                    self.enter_position = intersected_points
                elif intersected_points is not None and intersected_points.shape[0] > 1:
                    self.ca = ca
                    self.enter_position = intersected_points[0,:]
                    for ts in range(time_step, self.ego_vehicle.prediction.final_time_step):
                        if ca.contains_point(self.ego_vehicle.state_at_time(ts).position):
                            enter_time = ts - 1
                            break
                    end_position = self.ego_vehicle.state_at_time(enter_time).position
                    state_list_new = self.ego_vehicle.prediction.trajectory.state_list[self.time_step:enter_time+1]
                    pos_new = np.asarray([state.position for state in state_list_new])
                    distance = compute_total_polyline_length(pos_new)
                    distance += \
                    math.sqrt((self.enter_position[0]-end_position[0])**2+(self.enter_position[1]-end_position[1])**2)
                    psd = utils_gen.int_round(distance / msd, 2)
                    self.value = psd
                    utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {psd}")
                    return psd
                else:
                    utils_log.print_and_log_info(logger, f"*\t\t valid ca does not exist in this scenario")
                    return math.inf
            else :
                utils_log.print_and_log_info(logger, f"*\t\t valid ca does not exist in this scenario")
                return math.inf
        else :
            utils_log.print_and_log_info(logger, f"*\t\t {other_vehicle} Not a dynamic obstacle, ca does not exist")
            return math.inf
    
                
    def visualize(self, figsize: tuple = (25,15)):
        
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(self.time_step,
                                                                self.ego_vehicle.prediction.
                                                                trajectory.state_list,
                                                                margin=10)
        
        if self.value is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
          
        elif self.ca is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
      
        else:
            self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
            self.rnd.render()
            x_i, y_i = self.ca.exterior.xy
            plt.plot(x_i, y_i, color=TUMcolor.TUMred)
            plt.fill(x_i, y_i, color=TUMcolor.TUMred)
            utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                      color=TUMcolor.TUMblue, linewidth=1)
            utils_vis.draw_state_list(self.rnd, self.other_vehicle.prediction.trajectory.state_list[self.time_step:],
                                      color=TUMcolor.TUMlightgray, linewidth=1)
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, time_step=self.time_step,
                                             color=TUMcolor.TUMgreen)
            utils_vis.draw_circle(self.rnd, self.enter_position, 1, 0.5, color=TUMcolor.TUMred)
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.time_step, color=TUMcolor.TUMdarkred)
            plt.title(f"{self.metric_name} of {self.time_step} time steps")
        
            
            if self.configuration.debug.draw_visualization:
                if self.configuration.debug.save_plots:
                    utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
                else:
                    plt.show()
            