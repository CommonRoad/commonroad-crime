__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.2"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import time
from enum import Enum
from abc import abstractmethod
import copy
import logging
from typing import Union

import numpy as np

# CommonRoad packages
from commonroad.scenario.obstacle import Obstacle, DynamicObstacle, StaticObstacle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.prediction.prediction import SetBasedPrediction

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import (
    TypeTime,
    TypeNone,
    TypeReachableSet,
    TypeMonotone,
    TypePotential,
    TypeProbability,
)
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

logger = logging.getLogger(__name__)


class CriMeBase:
    """Base class for CRIticality MEasures"""

    measure_name: Enum = TypeNone.NONE
    monotone: Enum = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        """
        :param config: predefined configuration
        """
        if not isinstance(config, CriMeConfiguration):
            message = "Provided configuration is not valid"
            utils_log.print_and_log_error(logger, message)
            assert TypeError(message)
        # ==========  configuration  =========
        self.configuration = config
        self.value = None
        self.time_step = 0
        # =======  Scenario or scene  ========
        if config.scenario:
            self.sce = copy.deepcopy(config.scenario)
        else:
            self.sce = copy.deepcopy(config.scene)
        if self.sce is None:
            utils_log.print_and_log_warning(
                logger, "Scenario/scene in the configuration needs to be first updated"
            )
        self.dt = self.sce.dt

        assert self.sce.obstacle_by_id(self.configuration.vehicle.ego_id), (
            "<Criticality: the provided ego vehicle "
            f"{self.configuration.vehicle.ego_id} is "
            "not contained in the scenario>"
        )
        # =======       Vehicles      ========
        self.ego_vehicle: DynamicObstacle = self.sce.obstacle_by_id(
            self.configuration.vehicle.ego_id
        )
        if not isinstance(self.ego_vehicle, StaticObstacle) and not isinstance(
            self.ego_vehicle.prediction, SetBasedPrediction
        ):
            utils_gen.check_elements_state_list(
                [self.ego_vehicle.initial_state]
                + self.ego_vehicle.prediction.trajectory.states_in_time_interval(
                    time_begin=self.ego_vehicle.initial_state.time_step + 1,
                    time_end=self.ego_vehicle.prediction.final_time_step,
                ),
                self.dt,
            )
            self._update_clcs()
        self.other_vehicle: Union[Obstacle, DynamicObstacle, StaticObstacle, None] = (
            None  # optional
        )
        self.rnd: Union[MPRenderer, None] = None

    @property
    def clcs(self):
        return self.configuration.vehicle.curvilinear.clcs

    @clcs.setter
    def clcs(self, clcs: CurvilinearCoordinateSystem):
        raise AttributeError(
            "Please set up the `clcs` via the `update` function in the configuration."
        )

    def __repr__(self):
        return f"{self.measure_name}"

    def __eq__(self, other: object) -> bool:
        if isinstance(other, CriMeBase):
            return self.measure_name == other.measure_name
        else:
            return False

    def __key(self):
        return self.measure_name

    def __hash__(self):
        return hash(self.__key())

    def _update_clcs(self):
        """
        Updates the curvilinear coordinate system in the configuration setting using the reference path from the lanelet
        where the ego vehicle is currently located to the end of the lanelet.
        """
        # default setting of ego vehicle's curvilinear coordinate system
        ego_initial_lanelet_id = list(
            self.ego_vehicle.prediction.center_lanelet_assignment[
                self.ego_vehicle.initial_state.time_step
            ]
        )[0]
        reference_path = utils_gen.generate_reference_path(
            ego_initial_lanelet_id, self.sce.lanelet_network
        )
        clcs = CurvilinearCoordinateSystem(reference_path)
        self.configuration.update(CLCS=clcs)

    def _initialize_vis(
        self, figsize: tuple = (25, 15), plot_limit: Union[list, None] = None
    ):
        """
        Initializes the visualization with the scenario at the evaluated time step.

        :param figsize: size of the figure
        :param plot_limit: [xmin, xmax, ymin, ymax]
        """
        if plot_limit is None:
            plot_limit = self.configuration.debug.plot_limits
        self.rnd = MPRenderer(figsize=figsize, plot_limits=plot_limit)
        utils_vis.draw_sce_at_time_step(
            self.rnd, self.configuration, self.sce, self.time_step
        )

    def validate_update_states_log(
        self, vehicle_id: int = None, time_step: int = 0, verbose: bool = True
    ) -> bool:
        """
        Validates the presence of vehicle states at a given time step and updates internal states accordingly.

        This function checks if both the ego vehicle and another specified vehicle have valid states at the specified
        time step. It updates the internal time step and other vehicle states if valid. It logs the process and any
        warnings encountered.
        """
        if time_step is not None:
            if not self.ego_vehicle.state_at_time(time_step):
                utils_log.print_and_log_warning(
                    logger,
                    f"<{self.measure_name}>:"
                    f" The ego vehicle does NOT have the state at time step {time_step}",
                    verbose,
                )
                return False
            self.time_step = time_step

        if vehicle_id is not None:
            self.set_other_vehicles(vehicle_id)
            if not self.other_vehicle.state_at_time(self.time_step):
                utils_log.print_and_log_warning(
                    logger,
                    f"* <{self.measure_name}>:"
                    f" The vehicle {self.other_vehicle.obstacle_id} does NOT have the state at time step {time_step}",
                    verbose,
                )
                return False
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} at time step {time_step}",
            verbose,
        )
        return True

    def set_other_vehicles(self, vehicle_id: int):
        """
        Sets up the id for other measure-related vehicles.
        """
        # if already being set, do not have to reset again
        if self.other_vehicle:
            if vehicle_id == self.other_vehicle.obstacle_id:
                return

        if not self.sce.obstacle_by_id(vehicle_id):
            raise ValueError(
                f"<Criticality>: Vehicle (id: {vehicle_id}) is not contained in the scenario!"
            )
        self.other_vehicle = copy.deepcopy(self.sce.obstacle_by_id(vehicle_id))
        if isinstance(self.other_vehicle, DynamicObstacle):
            if isinstance(self.other_vehicle.prediction, TrajectoryPrediction):
                utils_gen.check_elements_state_list(
                    [self.other_vehicle.initial_state]
                    + self.other_vehicle.prediction.trajectory.state_list,
                    self.dt,
                )
            else:
                utils_gen.check_elements_state(
                    self.other_vehicle.initial_state, dt=self.dt
                )
        else:
            utils_gen.check_elements_state(self.other_vehicle.initial_state, dt=self.dt)

    def _except_obstacle_in_same_lanelet(self, expected_value: float, verbose: bool):
        if not utils_gen.check_in_same_lanelet(
            self.sce.lanelet_network,
            self.ego_vehicle,
            self.other_vehicle,
            self.time_step,
        ):
            utils_log.print_and_log_info(
                logger,
                f"*\t\t vehicle {self.other_vehicle.obstacle_id} is not "
                f"in the same lanelet as the "
                f"ego vehicle {self.ego_vehicle.obstacle_id}",
                verbose,
            )
            self.value = expected_value
            utils_log.print_and_log_info(
                logger, f"*\t\t {self.measure_name} = {self.value}", verbose
            )
            return True
        return False

    @abstractmethod
    def compute(self, time_step: int, vehicle_id: Union[int, None], verbose: bool):
        """
        Specific computing function for each measure
        """
        pass

    def compute_criticality(
        self, time_step: int, vehicle_id: Union[int, None] = None, verbose: bool = True
    ):
        """
        Wrapper for computing the criticality, i.e., the value of the measure.
        """
        if self.ego_vehicle.state_at_time(time_step) is None:
            utils_log.print_and_log_warning(
                logger,
                f"* The ego vehicle doesn't have state at time step {time_step}",
            )
            return np.nan

        utils_log.print_and_log_info(
            logger, "*********************************", verbose
        )

        self.time_step = time_step

        if vehicle_id:
            other_veh_ids = [vehicle_id]
        else:
            other_veh_ids = [
                veh.obstacle_id
                for veh in self.sce.obstacles
                if veh.obstacle_id is not self.ego_vehicle.obstacle_id
                and veh.state_at_time(self.time_step) is not None
            ]

        time_start = time.time()
        criti_list = []
        if self.measure_name in [
            TypeTime.TTR,
            TypeTime.TTM,
            TypeTime.TTB,
            TypeTime.TTK,
            TypeTime.TTS,
            TypeReachableSet.DA,
            TypePotential.PF,
            TypeProbability.P_MC,
        ]:
            criti = self.compute(time_step=time_step, vehicle_id=None, verbose=verbose)
        else:
            for v_id in other_veh_ids:
                criti_list.append(
                    self.compute(time_step=time_step, vehicle_id=v_id, verbose=verbose)
                )
            if len([c for c in criti_list if c is not None]) > 0:
                if np.all(np.isnan(criti_list)):
                    utils_log.print_and_log_warning(
                        logger,
                        f"* Due to the missing entries, all elements are NaN, "
                        f"the result for time step {time_step} is NaN",
                    )
                    return np.nan
                # Not all elements are NaN, return the max/min of the non-NaN values
                if self.monotone == TypeMonotone.POS:
                    criti = np.nanmax(criti_list)
                else:
                    criti = np.nanmin(criti_list)
            else:
                return None
        time_computation = time.time() - time_start
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} of the scenario: {criti}", verbose
        )
        utils_log.print_and_log_info(
            logger, f"\tTook: \t{time_computation:.3f}s", verbose
        )
        return criti

    @abstractmethod
    def visualize(self):
        """
        Visualize the result, which will be measure-dependent.
        """
        pass
