__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import time
from enum import Enum
from abc import abstractmethod
import copy
import logging
from typing import Union

# CommonRoad packages
from commonroad.scenario.obstacle import Obstacle, DynamicObstacle, StaticObstacle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.prediction.prediction import SetBasedPrediction

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import (TypeTime, TypeNone, TypeReachableSet, TypeMonotone,
                                                  TypePotential, TypeProbability)
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
            utils_log.print_and_log_warning(logger,
                                            "Scenario/scene in the configuration needs to be first updated")
        self.dt = self.sce.dt

        assert self.sce.obstacle_by_id(self.configuration.vehicle.ego_id), '<Criticality: the provided ego vehicle ' \
                                                                           f'{self.configuration.vehicle.ego_id} is ' \
                                                                           'not contained in the scenario>'
        # =======       Vehicles      ========
        self.ego_vehicle: DynamicObstacle = self.sce.obstacle_by_id(self.configuration.vehicle.ego_id)
        if not isinstance(self.ego_vehicle, StaticObstacle) and \
                not isinstance(self.ego_vehicle.prediction, SetBasedPrediction):
            utils_gen.check_elements_state_list([self.ego_vehicle.initial_state] + self.ego_vehicle.
                                                prediction.trajectory.states_in_time_interval(time_begin=1,
                                                                                              time_end=self.ego_vehicle.
                                                                                              prediction.final_time_step),
                                                self.dt)
            self.clcs: CurvilinearCoordinateSystem = self._update_clcs()
        self.other_vehicle: Union[Obstacle, DynamicObstacle, StaticObstacle, None] = None  # optional
        self.rnd: Union[MPRenderer, None] = None

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
        ego_initial_lanelet_id = list(self.ego_vehicle.prediction.center_lanelet_assignment[0])[0]
        reference_path = utils_gen.generate_reference_path(ego_initial_lanelet_id, self.sce.lanelet_network)
        clcs = CurvilinearCoordinateSystem(reference_path)
        self.configuration.update(CLCS=clcs)
        return clcs

    def _initialize_vis(self,
                        figsize: tuple = (25, 15), plot_limit: Union[list, None] = None):
        """
        Initializes the visualization with the scenario at the evaluated time step.

        :param figsize: size of the figure
        :param plot_limit: [xmin, xmax, ymin, ymax]
        """
        if plot_limit is None:
            plot_limit = self.configuration.debug.plot_limits
        self.rnd = MPRenderer(figsize=figsize, plot_limits=plot_limit)
        utils_vis.draw_sce_at_time_step(self.rnd, self.configuration, self.sce, self.time_step)

    def set_other_vehicles(self, vehicle_id: int):
        """
        Sets up the id for other measure-related vehicle.
        """
        if not self.sce.obstacle_by_id(vehicle_id):
            raise ValueError(f"<Criticality>: Vehicle (id: {vehicle_id}) is not contained in the scenario!")
        self.other_vehicle = self.sce.obstacle_by_id(vehicle_id)
        if isinstance(self.other_vehicle, DynamicObstacle):
            if isinstance(self.other_vehicle.prediction, TrajectoryPrediction):
                utils_gen.check_elements_state_list([self.other_vehicle.initial_state] +
                                                    self.other_vehicle.prediction.trajectory.state_list, self.dt)
            else:
                utils_gen.check_elements_state(self.other_vehicle.initial_state, dt=self.dt)
        else:
            utils_gen.check_elements_state(self.other_vehicle.initial_state, dt=self.dt)

    def _except_obstacle_in_same_lanelet(self, expected_value: float):
        if not utils_gen.check_in_same_lanelet(self.sce.lanelet_network, self.ego_vehicle,
                                               self.other_vehicle, self.time_step):
            utils_log.print_and_log_info(logger, f"*\t\t vehicle {self.other_vehicle.obstacle_id} is not "
                                                 f"in the same lanelet as the "
                                                 f"ego vehicle {self.ego_vehicle.obstacle_id}")
            self.value = expected_value
            utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
            return True
        return False

    @abstractmethod
    def compute(self, time_step: int, vehicle_id: Union[int, None]):
        """
        Specific computing function for each measure
        """
        pass

    def compute_criticality(self, time_step: int, vehicle_id: Union[int, None] = None, verbose=True):
        """
        Wrapper for computing the criticality, i.e., the value of the measure.
        """
        utils_log.print_and_log_info(logger, "*********************************", verbose)

        self.time_step = time_step
        if vehicle_id:
            other_veh_ids = [vehicle_id]
        else:
            other_veh_ids = [veh.obstacle_id for veh in self.sce.obstacles
                             if veh.obstacle_id is not self.ego_vehicle.obstacle_id]

        time_start = time.time()
        criti_list = []
        if self.measure_name in [TypeTime.TTR, TypeTime.TTM, TypeTime.TTB,
                                 TypeTime.TTK, TypeTime.TTS, TypeReachableSet.DA,
                                 TypePotential.PF, TypeProbability.P_MC]:
            criti = self.compute(time_step=time_step, vehicle_id=None)
        else:
            for v_id in other_veh_ids:
                criti_list.append(self.compute(time_step=time_step, vehicle_id=v_id))
            if self.monotone == TypeMonotone.POS:
                criti = max(criti_list)
            else:
                criti = min(criti_list)
        time_computation = time.time() - time_start
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} of the scenario: {criti}")
        utils_log.print_and_log_info(logger, f"\tTook: \t{time_computation:.3f}s", verbose)
        return criti

    @abstractmethod
    def visualize(self):
        """
        Visualize the result, which will be measure-dependent.
        """
        pass
