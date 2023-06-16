__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import logging
import math

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
from commonroad_crime.measure.time.et import ET
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor

from commonroad.scenario.scenario import Tag
from commonroad.scenario.obstacle import DynamicObstacle

logger = logging.getLogger(__name__)


class PET(CriMeBase):
    """
    See https://criticality-metrics.readthedocs.io/
    """
    measure_name = TypeTime.PET
    monotone = TypeMonotone.NEG
    metric_name = TypeTime.PET

    def __init__(self, config: CriMeConfiguration):
        super(PET, self).__init__(config)
        self.ca = None
        self.exit = None
        self.enter = None

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} beginning at time step {time_step}")
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        if Tag.INTERSECTION not in self.sce.tags:
            utils_log.print_and_log_info(logger, f"* Measure only for intersection. PET is set to inf.")
            self.value = math.inf
            return self.value
        proxy_et = ET(self.configuration)
        proxy_et.compute(vehicle_id)
        ca = proxy_et.ca
        pet = self.get_pet(self.time_step, ca)
        if ca is not None and not math.isinf(pet):
            self.ca = ca
            self.value = pet
            return pet
        else:
            utils_log.print_and_log_info(logger, f"*\t\t pet does not exist in this scenario")
            return math.inf

    def visualize(self, figsize: tuple = (25, 15)):
        if self.ca is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
            return 0
        if self.exit is None and self.enter is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
            return 0
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(self.time_step,
                                                                self.ego_vehicle.prediction.
                                                                trajectory.state_list,
                                                                margin=150)
        save_sce = self.sce
        self.sce_without_ego_and_other()
        self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
        self.rnd.render()
        self.sce = save_sce
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMlightgray, linewidth=1, start_time_step=0)
        utils_vis.draw_state_list(self.rnd, self.other_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMlightgray, linewidth=1, start_time_step=0)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMdarkred)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMgreen)
        if self.exit is not None:
            utils_vis.draw_state(self.rnd, self.other_vehicle.state_at_time(self.exit),
                                 color=TUMcolor.TUMorange)
        if self.enter is not None:
            utils_vis.draw_state(self.rnd, self.other_vehicle.state_at_time(self.enter),
                                 color=TUMcolor.TUMblack)

        plt.title(f"{self.metric_name} of {self.value} time steps")
        if self.ca is not None:
            x_i, y_i = self.ca.exterior.xy
            plt.plot(x_i, y_i, color=TUMcolor.TUMblack, zorder=1001)
            plt.fill(x_i, y_i, color=TUMcolor.TUMred, zorder=1001)

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()

    def get_pet(self, time_step, CA):
        if CA is None:
            return math.inf
        already_in = None
        enter_time = None
        for i in range(time_step, len(self.other_vehicle.prediction.trajectory.state_list)):
            other_v_poly = self.create_polygon(self.other_vehicle, i)
            if other_v_poly.intersects(CA) and already_in is None:
                enter_time = i
                self.enter = enter_time - 1
                already_in = True
            if not other_v_poly.intersects(CA) and already_in is True:
                exit_time = i
                self.exit = exit_time
                time = exit_time - enter_time
                # time steps to seconds
                time = time * self.dt
                return time
        return math.inf

    def create_polygon(self, obstacle: DynamicObstacle, time_step: int, w: float = 0, l_front: float = 0,
                       l_back: float = 0) -> Polygon:
        """
        Computes the shapely-polygon of an obstacle/vehicle. Will keep minimum shape of the object, but can be extended by
        providing different values, if they are bigger than the original values.

        :param obstacle: obstacle of which the polygon should be calculated
        :param time_step: point in time in scenario
        :param w: new width
        :param l_front: extended length to the front, measured from the center
        :param l_back: extended length to the back, measured from the center
        :return: shapely-polygon of the obstacle
        """
        pos = obstacle.state_at_time(time_step).position
        angle = obstacle.state_at_time(time_step).orientation
        angle_cos = math.cos(angle)
        angle_sin = math.sin(angle)
        width = max(obstacle.obstacle_shape.width * 0.5, w)
        length_front = max(obstacle.obstacle_shape.length * 0.5, l_front)
        length_back = max(obstacle.obstacle_shape.length * 0.5, l_back)
        coords = [(pos[0] + length_front * angle_cos - width * angle_sin,
                   pos[1] + length_front * angle_sin + width * angle_cos),
                  (pos[0] - length_back * angle_cos - width * angle_sin,
                   pos[1] - length_back * angle_sin + width * angle_cos),
                  (pos[0] - length_back * angle_cos + width * angle_sin,
                   pos[1] - length_back * angle_sin - width * angle_cos),
                  (pos[0] + length_front * angle_cos + width * angle_sin,
                   pos[1] + length_front * angle_sin - width * angle_cos),
                  (pos[0] + length_front * angle_cos - width * angle_sin,
                   pos[1] + length_front * angle_sin + width * angle_cos)]
        return Polygon(coords)

    def sce_without_ego_and_other(self):
        self.sce.remove_obstacle(self.ego_vehicle)
        self.sce.remove_obstacle(self.other_vehicle)
