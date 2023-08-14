__author__ = "Yuanfei Lin, Oliver Specht"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import logging
import math

from commonroad.scenario.obstacle import DynamicObstacle
from matplotlib import pyplot as plt

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndex, TypeMonotone
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.visualization as utils_vis


logger = logging.getLogger(__name__)


class SOI(CriMeBase):
    """
    The SOI defines a personal space for each actor and counts violations by other participants while
    setting them in relation to the analyzed period of time.

    -- H. Tsukaguchi and M. Mori, “Occupancy indices and its application to planning of residential streets,”
    Doboku Gakkai Ronbunshu, vol. 1987, no. 383, pp. 141–144, 1987.
    """

    measure_name = TypeIndex.SOI
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(SOI, self).__init__(config)
        self.value_list = []

    def create_sp_polygon(self, obstacle: DynamicObstacle, time_step: int):
        """
        Creates a polygon matching the personal space of the vehicle/bicycle/pedestrian.
        Uses predefined minimal margin, adds calculated area. Lateral boundaries depend on lanelet boundaries.
        """
        margin_front = self.configuration.index.soi.margin_front
        margin_back = self.configuration.index.soi.margin_back
        margin_side = self.configuration.index.soi.margin_side

        state = obstacle.state_at_time(time_step)

        # create minimum required area for vehicle
        minimum_space = utils_sol.create_polygon(
            obstacle,
            time_step,
            obstacle.obstacle_shape.width * 0.5 + margin_side,
            obstacle.obstacle_shape.length * 0.5 + margin_front,
            obstacle.obstacle_shape.length * 0.5 + margin_back,
        )

        # get all lanelets intersected by this area
        unique_lanelet_list = list(
            dict.fromkeys(
                [
                    lanelet
                    for sublist in self.sce.lanelet_network.find_lanelet_by_position(
                        list(minimum_space.exterior.coords) + [minimum_space.centroid]
                    )
                    for lanelet in sublist
                ]
            )
        )

        """
        calculating the minimum braking distance assuming optimal weather, no slope and no reaction time
        according to the formula s = (0.278 × t × v) + v² / (254 × (f + G)) by AASHTO Formula
        s = stopping distance in meters
        t = reaction time in seconds. Assumed to be 0 for now
        v = speed of the car in km/h
        G = grade/slope of the road, expressed as a decimal. Positive for an uphill grade and negative for 
            a downhill road. Assumed to be 0 for now.
        f = Coefficient of friction between the tires and the road. 
            It is assumed to be 0.7 on a dry road and between 0.3 and 0.4 on a wet road.
        """
        if isinstance(obstacle, DynamicObstacle):
            v = (
                math.sqrt(state.velocity**2 + state.velocity_y**2)
                * math.cos(state.orientation)
                * 3.6
            )  # in km/h
        else:
            v = 0.0
        minimum_breaking_distance = (0.278 * 0 * v) + (v**2) / (254 * (0.7 + 0))

        # create vehicle with correct front and back margin as well as exaggerated width to definitely match the whole
        # width of the lanelets
        exaggerated_vehicle = utils_sol.create_polygon(
            obstacle,
            time_step,
            obstacle.obstacle_shape.width * 0.5 + margin_side + 5,
            obstacle.obstacle_shape.length * 0.5
            + minimum_breaking_distance
            + margin_front,
            obstacle.obstacle_shape.length * 0.5 + margin_back,
        )

        # union of all intersections of exaggerated_vehicle with all lanelets,
        # that intersected with the minimum_space before
        personal_space = minimum_space
        for l_id in unique_lanelet_list:
            lanelet = self.sce.lanelet_network.find_lanelet_by_id(
                l_id
            ).polygon.shapely_object
            personal_space = personal_space.union(
                lanelet.intersection(exaggerated_vehicle)
            )
        return personal_space

    def compute(self, time_step: int = 0, vehicle_id: int = None, verbose: bool = True):
        """
        Calculates how often the personal space of the ego-vehicle is violated by obstacles in the observed time
        """
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} at time step {time_step}",
            verbose,
        )
        self.value = 0
        self.value_list.clear()
        self.time_step = time_step

        for ts in range(
            self.time_step, len(self.ego_vehicle.prediction.trajectory.state_list)
        ):
            ego_poly = self.create_sp_polygon(self.ego_vehicle, ts)

            for obstacle in self.sce.obstacles:
                # Skip ego-vehicle and obstacles out of scope (e.g. timeline ended for this obstacle)
                if (
                    isinstance(obstacle, DynamicObstacle)
                    and len(obstacle.prediction.trajectory.state_list) < ts
                ):
                    continue
                if obstacle.obstacle_id == self.ego_vehicle.obstacle_id:
                    continue

                other_poly = utils_sol.create_polygon(obstacle, ts)
                if ego_poly.intersects(other_poly):
                    self.value += 1

            self.value_list.append(self.value)
        self.value = utils_gen.int_round(self.value, 2)
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}", verbose
        )
        return self.value

    def bounds(self, margin):
        """
        Calculates the boundary of the relevant scene based on the start and end position of the ego-vehicle
        and a margin around.
        """
        pos1 = self.ego_vehicle.state_at_time(self.time_step).position
        pos2 = self.ego_vehicle.prediction.trajectory.state_list[-1].position
        if pos1[0] < pos2[0]:
            if pos1[1] < pos2[1]:
                return [
                    pos1[0] - margin,
                    pos2[0] + margin,
                    pos1[1] - margin,
                    pos2[1] + margin,
                ]
            else:
                return [
                    pos1[0] - margin,
                    pos2[0] + margin,
                    pos2[1] - margin,
                    pos1[1] + margin,
                ]
        else:
            if pos1[1] < pos2[1]:
                return [
                    pos2[0] - margin,
                    pos1[0] + margin,
                    pos1[1] - margin,
                    pos2[1] + margin,
                ]
            else:
                return [
                    pos2[0] - margin,
                    pos1[0] + margin,
                    pos2[1] - margin,
                    pos1[1] + margin,
                ]

    def visualize(self, figsize: tuple = (25, 15)):
        """
        Visualizes each time step with the current soi-value. Adds them to a -gif afterwards.
        """
        for time_step in range(
            self.time_step, len(self.ego_vehicle.prediction.trajectory.state_list)
        ):
            plt.cla()
            plt.axis("equal")
            plt.axis(self.bounds(10))
            plt.title(
                f"{self.measure_name} of {self.value_list[time_step - self.time_step]}"
            )

            for lanelet in self.sce.lanelet_network.lanelet_polygons:
                x, y = lanelet.shapely_object.exterior.xy
                plt.plot(x, y, color="black")

            # draw personal space of ego-vehicle
            obs = self.sce.obstacle_by_id(self.ego_vehicle.obstacle_id)
            x, y = self.create_sp_polygon(obs, time_step).exterior.xy
            plt.fill(x, y, facecolor="lightblue")

            # draw ego-vehicle itself
            x, y = utils_sol.create_polygon(obs, time_step).exterior.xy
            plt.fill(x, y, facecolor="blue")

            # draw al other obstacles
            for obstacle in self.sce.obstacles:
                # Skip ego-vehicle and obstacles out of scope (e.g. timeline ended for this obstacle)
                if (
                    isinstance(obstacle, DynamicObstacle)
                    and len(obstacle.prediction.trajectory.state_list) < time_step
                ):
                    continue
                if obstacle.obstacle_id == self.ego_vehicle.obstacle_id:
                    continue

                else:
                    x, y = utils_sol.create_polygon(obstacle, time_step).exterior.xy
                    plt.fill(x, y, facecolor="gray")

            if self.configuration.debug.draw_visualization:
                if self.configuration.debug.save_plots:
                    utils_vis.save_fig(
                        self.measure_name,
                        self.configuration.general.path_output,
                        time_step,
                        suffix="png",
                    )
                else:
                    plt.show()

        # combine each frame to a .gif
        if (
            self.configuration.debug.draw_visualization
            and self.configuration.debug.save_plots
        ):
            # not working for pipeline. Doesn't know commonroad_reach
            utils_vis.make_gif(
                self.configuration.general.path_output,
                self.measure_name + "_",
                range(
                    self.time_step,
                    len(self.ego_vehicle.prediction.trajectory.state_list),
                ),
                f"{self.measure_name} of {self.time_step}",
                duration=self.dt,
            )
