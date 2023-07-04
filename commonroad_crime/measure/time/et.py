__author__ = "Yuanfei Lin, Liguo Chen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import matplotlib.pyplot as plt
import logging
from shapely.geometry import Polygon

from commonroad.scenario.scenario import Tag
from commonroad.scenario.obstacle import DynamicObstacle

from commonroad_crime.data_structure.base import CriMeBase, TypeMonotone
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
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
        self.ca = None  # ca stands for conflict area
        self.enter_time = None  # The time points when the ego vehicle enters and leaves the conflict area
        self.exit_time = None

    def compute(self, vehicle_id, time_step: int = 0, call_from_pet: bool = False):
        # If is called by PET to calculate the conflict area, log information of ET will not be output.
        if not call_from_pet:
            utils_log.print_and_log_info(logger,
                                         f"* Computing the {self.measure_name} beginning at time step {time_step}")
        if (Tag.INTERSECTION not in self.sce.tags) or (len(self.sce.lanelet_network.intersections) == 0):
            if not call_from_pet:
                utils_log.print_and_log_info(logger, f"* \t\tMeasure only for intersection. ET is set to inf.")
            self.value = math.inf
            return self.value
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        if isinstance(self.other_vehicle, DynamicObstacle):
            self.ca = self.get_ca()
            self.value = self.get_et(self.time_step, self.ca)
            # The conflict area may not exist, indicated by self.ca being None.
            # Even if the conflict area exists, there are two scenarios where the ET remains undefined,
            # and we set it to infinity.
            if self.ca is None:
                if not call_from_pet:
                    utils_log.print_and_log_info(logger, f"* \t\tconflict area does not exist, ET is set to inf.")
            elif math.isinf(self.value):
                if not call_from_pet:
                    if self.enter_time is None:
                        utils_log.print_and_log_info(logger,
                                                     "* \t\tThe ego vehicle never encroaches the CA, ET is set to inf.")
                    else:
                        utils_log.print_and_log_info(logger,
                                                     "* \t\tThe ego vehicle encroaches the CA, "
                                                     "but never leaves it, ET is set to inf.")
            return self.value
        else:
            if not call_from_pet:
                utils_log.print_and_log_info(logger,
                                             f"*\t\t {self.other_vehicle} Not a dynamic obstacle, ET is set to inf")
            self.value = math.inf
            return self.value

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
        ref_path_lanelets_ego = self.get_ref_path_lanelets_id(time_step, self.ego_vehicle)
        ca = None
        for i in range(time_step, len(other_vehicle.prediction.trajectory.state_list)):
            other_vehicle_state = other_vehicle.state_at_time(i)
            other_vehicle_lanelet_id = \
                self.sce.lanelet_network.find_lanelet_by_position([other_vehicle_state.position])[0]
            intersected_ids = set(ref_path_lanelets_ego).intersection(set(other_vehicle_lanelet_id))
            for intersected_lanelet_id in intersected_ids:  # 1.
                intersected_lanelet = self.sce.lanelet_network.find_lanelet_by_id(intersected_lanelet_id)
                if self.is_at_intersection(intersected_lanelet):  # 2.
                    other_vehicle_dir_lanelet_id = self.get_dir_lanelet_id(self.other_vehicle, i)
                    if (other_vehicle_dir_lanelet_id != intersected_lanelet.lanelet_id and not self.same_income(
                            other_vehicle_dir_lanelet_id, intersected_lanelet_id)):  # 3, 4.
                        ca = self.get_ca_from_lanelets(other_vehicle_dir_lanelet_id, intersected_lanelet_id)
        return ca

    def visualize(self, figsize: tuple = (25, 15)):
        if self.ca is None:
            utils_log.print_and_log_info(logger, "* \t\tNo conflict area")
            return 0
        if self.exit_time is None and self.enter_time is None:
            utils_log.print_and_log_info(logger, "* \t\tNo conflict area")
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
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step::3],
                                  color=TUMcolor.TUMlightgray, linewidth=1, start_time_step=0)
        utils_vis.draw_state_list(self.rnd, self.other_vehicle.prediction.trajectory.state_list[self.time_step::3],
                                  color=TUMcolor.TUMgray, linewidth=1, start_time_step=0)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMblack)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMblue)
        if self.exit_time is not None:
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, time_step=self.exit_time,
                                             color=TUMcolor.TUMblack)
        if self.enter_time is not None:
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, time_step=self.enter_time,
                                             color=TUMcolor.TUMblack)

        plt.title(f"{self.measure_name} of {self.value} time steps")
        if self.ca is not None:
            x_i, y_i = self.ca.exterior.xy
            plt.plot(x_i, y_i, color=TUMcolor.TUMblack, zorder=1001)
            plt.fill(x_i, y_i, color=TUMcolor.TUMred, zorder=1001)

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()

    def get_dir_lanelet_id(self, vehicle: DynamicObstacle, time_step: int):
        """
        By querying the trajectory of the vehicle, the occupied lanelets based on the driving direction of the vehicle(
        dir_lanlet) can be obtained.
        """
        init_lanelets = self.sce.lanelet_network.find_lanelet_by_position(
            [vehicle.state_at_time(time_step).position])[0]
        init_lanelets_set = set(init_lanelets)
        for i in range(time_step, len(vehicle.prediction.trajectory.state_list)):
            current_lanelets_set = set(
                self.sce.lanelet_network.find_lanelet_by_position([vehicle.state_at_time(i).position])[0])
            lanelets_not_in_init = current_lanelets_set - init_lanelets_set
            # Find the moment when the vehicle just occupies new lanelets,
            # the predecessor of the newly occupied lanelets is the desired dir_lanelet.
            if len(lanelets_not_in_init) > 0:
                for successor_id in lanelets_not_in_init:
                    predecessor_id__of_successor = set(
                        self.sce.lanelet_network.find_lanelet_by_id(successor_id).predecessor)
                    if len(predecessor_id__of_successor - init_lanelets_set) > 0:
                        return (predecessor_id__of_successor - init_lanelets_set).pop()
        # In case of the vehicle does not occupy new lanes within the timeframe of the trajectory,
        # represent 'dir_lanelet' based on the most likely lanelet inferred from the state.
        return self.sce.lanelet_network.find_most_likely_lanelet_by_state([vehicle.state_at_time(time_step)])[0]

    def get_ca_from_lanelets(self, lanelet_id_a, lanelet_id_b):
        if (lanelet_id_a is None) or (lanelet_id_b is None):
            return None
        lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_a)
        lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_b)
        lanelet_a_polygon: Polygon = lanelet_a.polygon.shapely_object
        lanelet_b_polygon: Polygon = lanelet_b.polygon.shapely_object
        ca = lanelet_a_polygon.intersection(lanelet_b_polygon)
        return ca

    def same_income(self, lanelet_id_a, lanelet_id_b):
        """
        Determine if the two lanelets originate from the same incoming at an intersection.
        """
        lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_a)
        lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(lanelet_id_b)
        first_incoming_lanelet_a = lanelet_a
        first_incoming_lanelet_b = lanelet_b
        while self.is_at_intersection(first_incoming_lanelet_a):
            predecessor = first_incoming_lanelet_a.predecessor
            if len(predecessor) >= 1:
                first_incoming_lanelet_a = self.sce.lanelet_network.find_lanelet_by_id(predecessor[0])
            else:
                return False

        while self.is_at_intersection(first_incoming_lanelet_b):
            predecessor = first_incoming_lanelet_b.predecessor
            if len(predecessor) >= 1:
                first_incoming_lanelet_b = self.sce.lanelet_network.find_lanelet_by_id(predecessor[0])
            else:
                return False
        intersection = self.sce.lanelet_network.map_inc_lanelets_to_intersections[first_incoming_lanelet_a.lanelet_id]
        incoming_a = intersection.map_incoming_lanelets[first_incoming_lanelet_a.lanelet_id]
        incoming_b = intersection.map_incoming_lanelets[first_incoming_lanelet_b.lanelet_id]
        return incoming_a.incoming_id == incoming_b.incoming_id

    def is_at_intersection(self, lanelet_x):
        if 'intersection' in lanelet_x.lanelet_type:
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

    def get_ref_path_lanelets_id(self, time_step, vehicle):
        """
        Obtain all the lanes passed by the predicted trajectory of the vehicle.
        """
        state_list = vehicle.prediction.trajectory.state_list
        ref_path_lanelets_id = set()
        for i in range(time_step, len(state_list)):
            lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([vehicle.state_at_time(i).position])[0]
            ref_path_lanelets_id.update(lanelet_id)
        return list(ref_path_lanelets_id)

    def get_et(self, time_step, ca):
        # In case conflict area does not exist, ET will be set to inf.
        if ca is None:
            return math.inf
        already_in = None
        enter_time = None
        for i in range(time_step, len(self.ego_vehicle.prediction.trajectory.state_list)):
            ego_v_poly = create_polygon(self.ego_vehicle, i)
            if ego_v_poly.intersects(ca) and already_in is None:
                enter_time = i
                self.enter_time = enter_time - 1
                already_in = True
            if not ego_v_poly.intersects(ca) and already_in is True:
                exit_time = i
                self.exit_time = exit_time
                time = exit_time - enter_time
                # time steps to seconds
                time = time * self.dt
                return time
        return math.inf

    def sce_without_ego_and_other(self):
        self.sce.remove_obstacle(self.ego_vehicle)
        self.sce.remove_obstacle(self.other_vehicle)
def create_polygon(obstacle: DynamicObstacle, time_step: int, w: float = 0,
                   l_front: float = 0,
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