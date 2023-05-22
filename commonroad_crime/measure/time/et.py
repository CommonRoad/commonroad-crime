__author__ = "Yuanfei Lin"
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
from commonroad_crime.data_structure.base import CriMeBase, TypeMonotone
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.solver as utils_sol
import matplotlib.pyplot as plt
import logging
import numpy as np
import commonroad_crime.utility.general as utils_gen
from commonroad_crime.utility.visualization import TUMcolor



logger = logging.getLogger(__name__)


class ET(CriMeBase):
    """
    See https://criticality-metrics.readthedocs.io/
    """
    measure_name = TypeTime.ET
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(ET, self).__init__(config)

    def compute(self, obstacle_id, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} beginning at time step {time_step}")
        self.time_step = time_step
        obstacle = self.sce.lanelet_network.find_lanelet_by_id(obstacle_id)
        ca = None
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
                ref_path_lanelets_ego = self.get_ref_path_lanelets_ID(time_step, self.ego_vehicle)
                for i in range(time_step, len(obstacle.prediction.trajectory.state_list)):
                    obstacle_state = obstacle.state_at_time(i)
                    obstacle_lanelet_id: int = \
                        self.sce.lanelet_network.find_lanelet_by_position([obstacle_state.position])[0][0]
                    obstacle_lanelet = self.sce.lanelet_network.find_lanelet_by_id(obstacle_lanelet_id)
                    intersected = set(ref_path_lanelets_ego).intersection(obstacle_lanelet_id)
                    for intersected_lanelet in intersected:
                        if ('intersection' in obstacle_lanelet.lanelet_type):
                            obstacle_dir_lanelet_id = \
                            self.sce.lanelet_network.find_most_likely_lanelet_by_state([obstacle_state])[0]
                            if (obstacle_dir_lanelet_id != intersected_lanelet and self.same_income(
                                    obstacle_dir_lanelet_id, intersected_lanelet)):
                                ca = self.get_ca_from_lanelets(obstacle_dir_lanelet_id, intersected_lanelet)
                if ca is not None:
                    time_in_ca = self.time_in_CA(ca, self.time_step, ca)
                return ca, time_in_ca
        return ca, None
    def get_ca_from_lanelets(self, lanelet_id_a, lanelet_id_b):
        lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_a)
        lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_b)
        lanelet_a_polygon: Polygon = lanelet_a.polygon.shapely_object
        lanelet_b_polygon: Polygon = lanelet_b.polygon.shapely_object
        ca = lanelet_a_polygon.intersection(lanelet_b_polygon)
        return ca
    def same_income(self, lanelet_id_a, lanelet_id_b):
        lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_a)
        lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_b)
        first_incoming_lanelet_a = lanelet_a
        first_incoming_lanelet_b = lanelet_b
        while("intersection" in first_incoming_lanelet_a.lanelet_type):
            first_incoming_lanelet_a = first_incoming_lanelet_a.predecessor[0]
        while ("intersection" in first_incoming_lanelet_b.lanelet_type):
            first_incoming_lanelet_b = first_incoming_lanelet_b.predecessor[0]
        intersection = self.sce.lanelet_network.map_inc_lanelets_to_intersections[lanelet_id_a]
        incoming_a = intersection.map_incoming_lanelets[first_incoming_lanelet_a]
        incoming_b = intersection.map_incoming_lanelets[first_incoming_lanelet_a]
        return incoming_a == incoming_b


    def get_ref_path_lanelets_ID(self, time_step, vehicle):
        state_list = vehicle.prediction.trajectory.state_list
        last_lanelet_id = None
        ref_path_lanelets_ID = []
        for i in range(time_step, len(state_list)):
            lanelet_id: int = \
                self.sce.lanelet_network.find_lanelet_by_position([vehicle.state_at_time(i).position])[0][0]
            if last_lanelet_id != lanelet_id:
                last_lanelet_id = lanelet_id
                ref_path_lanelets_ID.append(last_lanelet_id)
        return ref_path_lanelets_ID
    def time_in_CA(self, time_step, CA):
        already_in = None
        enter_time = None
        exit_time = None
        for i in range(time_step, len(self.ego_vehicle.prediction.trajectory.state_list)):
            ego_v_poly = utils_sol.create_polygon(self.ego_vehicle, i)
            if ego_v_poly.intersects(CA) and already_in is None:
                enter_time = i
                already_in = True
            if not ego_v_poly.intersects(CA) and already_in is True:
                exit_time = i
                time = exit_time - enter_time
                # time steps to seconds
                time = time * self.dt
                return time

        if enter_time == None:
            utils_log.print_and_log_info(logger, "* The ego vehicle never encroaches the CA")
            return np.NINF
        elif exit_time is None:
            utils_log.print_and_log_info(logger,
                                         "* The ego vehicle encroaches the CA, but never leaves it")
            return np.NINF