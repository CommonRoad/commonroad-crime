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
from commonroad.geometry.shape import ShapeGroup

logger = logging.getLogger(__name__)


class PSD(CriMeBase):
    """
    See https://criticality-metrics.readthedocs.io/
    """
    measure_name = TypeDistance.PSD
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(PSD, self).__init__(config)
        self._msd_object = MSD(config)
        self.ca = None

    def compute(self, vehicle_id: int = None, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} beginning at time step {time_step}")
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        other_vehicle = self.sce.obstacle_by_id(vehicle_id)
        self.time_step = time_step
        self.value = None
        # compute MSD
        msd = self._msd_object.compute(vehicle_id, time_step)
        if msd == 0:
            utils_log.print_and_log_info(logger, f"*\t\t msd is zero")
            return math.inf
        if msd == math.inf:
            utils_log.print_and_log_info(logger, f"*\t\t msd is infinity")
            return 0
        if isinstance(self.other_vehicle, DynamicObstacle):
            ca = self.get_ca()
            if ca is not None:
                et = self.get_et(self.time_step, ca)
                if (et != math.inf):
                    self.ca = ca
                    ego_poly = self.ego_vehicle.occupancy_at_time(self.time_step).shape.shapely_object
                    distance = ego_poly.distance(ca)
                    psd = utils_gen.int_round(distance / msd, 2)
                    self.value = psd
                    utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {psd}")
                    return psd
                else:
                    utils_log.print_and_log_info(logger, f"*\t\t valid ca does not exist in this scenario")
                    return math.inf
            else:
                utils_log.print_and_log_info(logger, f"*\t\t valid ca does not exist in this scenario")
                return math.inf
        else:
            utils_log.print_and_log_info(logger, f"*\t\t {other_vehicle} Not a dynamic obstacle, ca does not exist")
            return math.inf

    def get_ca(self):
        """
        Determine the existence of a conflict area based on the definition, and return it if it exists.
        1.Determine if there is any intersection between the lanelets traversed by the ego vehicle and other vehicles.
        2.Determine if the intersected lanelets located at an intersection.
        3.Determine if the intersecting lanelets are not in the direction of trajectory of the other vehicle.
        4.Determine if the ego vehicle and the other vehicle originate from different incomings.
        """
        time_step = self.time_step
        other_vehicle = self.other_vehicle
        ref_path_lanelets_ego = self.get_ref_path_lanelets_ID(time_step, self.ego_vehicle)
        ca = None
        for i in range(time_step, len(other_vehicle.prediction.trajectory.state_list)):
            other_vehicle_state = other_vehicle.state_at_time(i)
            other_vehicle_lanelet_id = \
                self.sce.lanelet_network.find_lanelet_by_position([other_vehicle_state.position])[0]
            intersected_ids = set(ref_path_lanelets_ego).intersection(set(other_vehicle_lanelet_id))
            for intersected_lanelet_id in intersected_ids:  # 1.
                intersected_lanelet = self.sce.lanelet_network.find_lanelet_by_id(intersected_lanelet_id)
                if (self.is_at_intersection(intersected_lanelet)):  # 2.
                    other_vehicle_dir_lanelet_id = \
                        self.sce.lanelet_network.find_most_likely_lanelet_by_state([other_vehicle_state])[0]  # 3.
                    if (other_vehicle_dir_lanelet_id != intersected_lanelet.lanelet_id and not self.same_income(
                            other_vehicle_dir_lanelet_id, intersected_lanelet_id)):  # 4.
                        ca = self.get_ca_from_lanelets(other_vehicle_dir_lanelet_id, intersected_lanelet_id)
        return ca

    def get_ca_from_lanelets(self, lanelet_id_a, lanelet_id_b):
        if ((lanelet_id_a is None) or (lanelet_id_b is None)):
            return None
        lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_a)
        lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_b)
        lanelet_a_polygon: Polygon = lanelet_a.polygon.shapely_object
        lanelet_b_polygon: Polygon = lanelet_b.polygon.shapely_object
        ca = lanelet_a_polygon.intersection(lanelet_b_polygon)
        return ca

    def same_income(self, lanelet_id_a, lanelet_id_b):
        """
        "Determine if the two lanelets originate from the same incoming at an intersection."
        """
        lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_a)
        lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_b)
        first_incoming_lanelet_a = lanelet_a
        first_incoming_lanelet_b = lanelet_b
        while (self.is_at_intersection(first_incoming_lanelet_a)):
            predecessor = first_incoming_lanelet_a.predecessor
            if (len(predecessor) >= 1):
                first_incoming_lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(predecessor[0])
            else:
                return False

        while (self.is_at_intersection(first_incoming_lanelet_b)):
            predecessor = first_incoming_lanelet_b.predecessor
            if (len(predecessor) >= 1):
                first_incoming_lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(predecessor[0])
            else:
                return False
        intersection = self.sce.lanelet_network.map_inc_lanelets_to_intersections[first_incoming_lanelet_a.lanelet_id]
        incoming_a = intersection.map_incoming_lanelets[first_incoming_lanelet_a.lanelet_id]
        incoming_b = intersection.map_incoming_lanelets[first_incoming_lanelet_b.lanelet_id]
        return incoming_a.incoming_id == incoming_b.incoming_id

    def is_at_intersection(self, lanelet_x):
        if ('intersection' in lanelet_x.lanelet_type):
            return True
        intersections = self.sce.lanelet_network.intersections
        at_intersection = False
        for intersection in intersections:
            for incoming in intersection.incomings:
                successors_left = [successor for successor in incoming.successors_left]
                successors_right = [successor for successor in incoming.successors_right]
                successors_straight = [successor for successor in incoming.successors_straight]
                if lanelet_x.lanelet_id in set(successors_straight + successors_left + successors_right):
                    at_intersection = True
        return at_intersection

    def get_ref_path_lanelets_ID(self, time_step, vehicle):
        """
        Obtain all the lanes passed by the predicted trajectory of the vehicle.
        """
        state_list = vehicle.prediction.trajectory.state_list
        ref_path_lanelets_ID = set()
        for i in range(time_step, len(state_list)):
            lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([vehicle.state_at_time(i).position])[0]
            ref_path_lanelets_ID.update(lanelet_id)
        return list(ref_path_lanelets_ID)

    def get_et(self, time_step, CA):
        already_in = None
        enter_time = None
        exit_time = None
        for i in range(time_step, len(self.ego_vehicle.prediction.trajectory.state_list)):
            ego_v_poly = self.create_polygon(self.ego_vehicle, i)
            if ego_v_poly.intersects(CA) and already_in is None:
                enter_time = i
                self.enter = enter_time - 1
                already_in = True
            if not ego_v_poly.intersects(CA) and already_in is True:
                exit_time = i
                self.exit = exit_time
                time = exit_time - enter_time
                # time steps to seconds
                time = time * self.dt
                return time
        return math.inf
        if enter_time is None:
            utils_log.print_and_log_info(logger, "* The ego vehicle never encroaches the CA")
            return math.NINF
        elif exit_time is None:
            utils_log.print_and_log_info(logger,
                                         "* The ego vehicle encroaches the CA, but never leaves it")
            return math.NINF

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

    def visualize(self, figsize: tuple = (25, 15)):

        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(self.time_step,
                                                                self.ego_vehicle.prediction.
                                                                trajectory.state_list,
                                                                margin=10)

        if self.value is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
            return 0
        if self.ca is None:
            utils_log.print_and_log_info(logger, "* No conflict area")
            return 0

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
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMdarkred)
        plt.title(f"{self.metric_name} of {self.time_step} time steps")

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()
