__author__ = "Oliver Specht, Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import matplotlib.pyplot as plt
import logging
import numpy as np

from commonroad.geometry.shape import ShapeGroup

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeDistance, TypeMonotone
from commonroad_crime.data_structure.base import CriMeBase
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class DCE(CriMeBase):
    """
    J. Eggert, “Predictive risk estimation for intelligent ADAS functions,”
    in 17th International Conference on Intelligent Transportation Systems (ITSC), pp. 711–718, IEEE, 2014.
    """

    measure_name = TypeDistance.DCE
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(DCE, self).__init__(config)
        self.time_dce = None

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        """
        Computed the shortest distance of ego-vehicle and obstacle by calculating the distance between both polygons
        (boundaries of both objects) for each time step and returning the minimum.
        """
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step

        state_list = self.ego_vehicle.prediction.trajectory.state_list
        dce = math.inf
        for i in range(time_step, len(state_list)):
            if self.other_vehicle.occupancy_at_time(i) is not None:
                ego_poly = self.ego_vehicle.occupancy_at_time(i).shape.shapely_object
                other_shape = self.other_vehicle.occupancy_at_time(i).shape
                if isinstance(other_shape, ShapeGroup):
                    distance = min(
                        [ego_poly.distance(shape_element.shapely_object) for shape_element in other_shape.shapes])
                else:
                    other_poly = self.other_vehicle.occupancy_at_time(i).shape.shapely_object
                    distance = ego_poly.distance(other_poly)
                if distance < dce:
                    self.time_dce = i
                    dce = distance
                if dce == 0.:
                    break
            else:
                break
        if dce is not math.inf:
            self.value = utils_gen.int_round(dce, 2)
        else:
            self.value = dce
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} with vehicle id {vehicle_id} = {self.value}")
        return self.value

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=utils_vis.
                             plot_limits_from_state_list(self.time_step,
                                                         self.ego_vehicle.prediction.trajectory.state_list,
                                                         margin=10))
        self.rnd.render()
        utils_vis.draw_reference_path(self.rnd, np.array(self.clcs.reference_path()))
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)

        utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(self.time_dce),
                             color=TUMcolor.TUMblue)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, self.time_dce,
                                         color=TUMcolor.TUMred)

        plt.title(f"{self.measure_name} of {self.value} m")

        x, y = self.ego_vehicle.occupancy_at_time(self.time_dce).shape.shapely_object.exterior.xy
        plt.plot(x, y)

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()
