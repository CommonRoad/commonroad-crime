__author__ = "Ivana Peneva"
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
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.solver as utils_sol
import matplotlib.pyplot as plt
import logging
import commonroad_crime.utility.general as utils_gen
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class ET(CriMeBase):
    metric_name = TypeTimeScale.ET
    """
        B. L. Allen, B. T. Shin, and P. J. Cooper, “Analysis of 
        Traffic Conflicts and Collisions,” Transportation Research
        Record, vol. 667, pp. 67–74, 1978.
    """

    def __init__(self, config: CriMeConfiguration):
        super(ET, self).__init__(config)
        self.intersection = None
        self.exit = None
        self.enter = None

    def compute(self, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} beginning at time step {time_step}")
        self.time_step = time_step
        state_list = self.ego_vehicle.prediction.trajectory.state_list

        obstacle_list: list[Union[StaticObstacle, DynamicObstacle]] = self.sce.obstacles
        intersection_list: list[Polygon] = []
        for obstacle in obstacle_list:

            inter: Polygon = self.calculate_ca_two_vehicles(obstacle, time_step)
            if inter is not None and inter.area > 1:
                # 1 as munimum area of a polygon
                intersection_list.append(inter)

        self.intersection = shapely.ops.cascaded_union(intersection_list)

        already_in = False

        for i in range(time_step, len(state_list)):
            ego_v_poly = utils_sol.create_polygon(self.ego_vehicle, i)
            if ego_v_poly.intersects(self.intersection) and self.enter is None:
                self.enter = i
                already_in = True
            if not ego_v_poly.intersects(self.intersection) and already_in is True:
                self.exit = i
                self.value = self.exit - self.enter
                # time steps to seconds
                self.value = self.value * self.dt
                return self.value

        if self.enter == math.inf:
            utils_log.print_and_log_info(logger, "* The ego vehicle never encroaches the CA")
            self.value = math.inf
            return self.value
        elif self.exit is None:
            utils_log.print_and_log_info(logger, "* The ego vehicle encroaches the CA, but never leaves it")
            self.value = math.inf
            return self.value

    def visualize(self, figsize: tuple = (25, 15)):
        if self.value == math.inf:
            return 0
        if self.intersection is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
            return 0

        if self.exit is None and self.enter is None:
            return 0
        self._initialize_vis(figsize=figsize)
        self.rnd.render()
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)
        if self.exit is not None:
            utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(self.exit),
                                 color=TUMcolor.TUMorange)
        if self.enter is not None and self.enter != math.inf:
            utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(self.enter),
                                 color=TUMcolor.TUMblue)

        plt.title(f"{self.metric_name} of {self.value} m")
        if self.intersection.type == "Polygon":
            x_i, y_i = self.intersection.exterior.xy
            plt.plot(x_i, y_i, color='red')
            plt.fill(x_i, y_i)

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()

    def calculate_ca_two_vehicles(self, obstacle, time_step: int = 0):
        if utils_gen.check_in_same_lanelet(self.sce.lanelet_network, self.ego_vehicle,
                                           obstacle, self.time_step):
            utils_log.print_and_log_info(logger, f"*\t\t vehicle {obstacle} is  "
                                                 f"in the same lanelet as the "
                                                 f"ego vehicle {self.ego_vehicle.obstacle_id}")
        else:
            # ca stands for conflict area
            self.time_step = time_step
            self.set_other_vehicles(obstacle.obstacle_id)
            if isinstance(self.other_vehicle, DynamicObstacle):

                merged_ego = self.merged_area_vehicle(time_step, self.ego_vehicle)
                merged_other = self.merged_area_vehicle(time_step, self.other_vehicle)

                intersection = merged_ego.intersection(merged_other)
                if intersection.is_empty:
                    utils_log.print_and_log_info(logger,
                                                 f"No critical area with ego vehicle and obstacle {obstacle.obstacle_id} ")
                    # plt.show()
                    return None
                geom_list = []
                if intersection.type == "GeometryCollection":
                    for geom in intersection.geoms:
                        if geom.type == "Polygon":
                            geom_list.append(geom)
                    merged_geoms = geom_list[0]
                    for g in range(1, len(geom_list)):
                        merged_geoms.union(geom_list[g])
                    intersection = merged_geoms

                if intersection.type == "Polygon":
                    x_i, y_i = intersection.exterior.xy
                    plt.plot(x_i, y_i, color='red')
                    # plt.show()
                    return intersection
                return None
        return None

    def merged_area_vehicle(self, time_step, vehicle):
        state_list = vehicle.prediction.trajectory.state_list
        vehicle_polygons = []
        last_lanelet_id = None

        for i in range(time_step, len(state_list)):
            lanelet_id: int = \
                self.sce.lanelet_network.find_lanelet_by_position([vehicle.state_at_time(i).position])[0][
                    0]

            if last_lanelet_id != lanelet_id:
                last_lanelet_id = lanelet_id
                lanelet: Lanelet = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id)
                lanelet_polygon: Polygon = lanelet.polygon.shapely_object
                vehicle_polygons.append(lanelet_polygon)

        merged = vehicle_polygons[0]

        for i in range(1, len(vehicle_polygons)):
            a = vehicle_polygons[i].union(merged)
            merged = a

        x_e, y_e = merged.exterior.xy
        plt.plot(x_e, y_e, color='blue')
        return merged
