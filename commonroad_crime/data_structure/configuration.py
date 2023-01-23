__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
from typing import Union, Optional
from omegaconf import ListConfig, DictConfig

from commonroad.scenario.scenario import Scenario
from commonroad.common.solution import VehicleType
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.feasibility.vehicle_dynamics import PointMassDynamics
from commonroad_crime.data_structure.scene import Scene
import commonroad_crime.utility.general as utils_general

from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3

logger = logging.getLogger(__name__)


class CriMeConfiguration:
    """Class to hold criticality-measures-related configurations"""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.scenario: Optional[Scenario] = None
        self.scene: Optional[Scene] = None
        self.general: GeneralConfiguration = GeneralConfiguration(config)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config)
        self.debug: DebugConfiguration = DebugConfiguration(config)

        self.time: TimeDomainConfiguration = TimeDomainConfiguration(config)
        self.velocity: VelocityDomainConfiguration = VelocityDomainConfiguration(config)
        self.acceleration: AccelerationDomainConfiguration = AccelerationDomainConfiguration(config)
        self.potential: PotentialDomainConfiguration = PotentialDomainConfiguration(config)
        self.probability: ProbabilityDomainConfiguration = ProbabilityDomainConfiguration(config)
        self.reachable_set: ReachableSetDomainConfiguration = ReachableSetDomainConfiguration(config)
        self.index: IndexDomainConfiguration = IndexDomainConfiguration(config)

    def update(self,
               ego_id: int = None,
               sce: Union[Scene, Scenario] = None,
               CLCS: Optional[CurvilinearCoordinateSystem] = None,
               ):
        """
        Updates criticality configuration based on the given attributes.

        Possible ways of updating the configuration:
        1. No attribute is given: load the scenario and evaluate the criticality of all obstacles.
        2. scenario
        3. scene
        4. scenario + clcs
        5. scene + clcs
        """
        if isinstance(sce, Scene):
            self.scene = sce
        elif isinstance(sce, Scenario):
            self.scenario = sce
        elif not sce and (self.scene or self.scenario):  # prevent reloading the existing scenario
            pass
        else:
            self.scenario = utils_general.load_scenario(self)  # if none is provided, scenario is at default
        if CLCS:
            self.vehicle.curvilinear.clcs = CLCS
        if ego_id:
            if self.scenario:
                if self.scenario.obstacle_by_id(ego_id) is None:
                    assert f'<Criticality>: Vehicle (id: {ego_id}) is not contained in the scenario!'
            if self.scene:
                if self.scene.obstacle_by_id(ego_id) is None:
                    assert f'<Criticality>: Vehicle (id: {ego_id}) is not contained in the scenario!'
            self.vehicle.ego_id = ego_id

    def print_configuration_summary(self):
        string = "# ===== Configuration Summary ===== #\n"
        string += f"# Scene/Scenario: {self.general.name_scenario}\n"
        string += f"# ego vehicle: id {self.vehicle.ego_id}\n"
        string += "# ================================= #"

        print(string)
        for line in string.split("\n"):
            logger.info(line)


class GeneralConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.general
        name_scenario = config_relevant.name_scenario

        self.name_scenario = name_scenario
        self.path_scenarios = config_relevant.path_scenarios
        self.path_scenarios_batch = config_relevant.path_scenarios_batch
        self.path_output_abs = config_relevant.path_output
        self.path_logs = config_relevant.path_logs
        self.path_icons = config_relevant.path_icons

    @property
    def path_scenario(self):
        return self.path_scenarios + self.name_scenario + ".xml"

    @property
    def path_output(self):
        return self.path_output_abs + self.name_scenario + "/"


class TimeDomainConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.time_scale
        self.activated = config_relevant.activated
        self.metric = config_relevant.metric
        self.steer_width = config_relevant.steer_width
        self.tau = config_relevant.tau


class ReachableSetDomainConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.reachable_set_scale
        self.time_horizon = config_relevant.time_horizon
        self.cosy = config_relevant.coordinate_system


class VelocityDomainConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.velocity_scale
        self.m_b = config_relevant.m_b


class AccelerationDomainConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.acceleration_scale
        self.safety_time = config_relevant.safety_time
        self.acceleration_mode = config_relevant.acceleration_mode


class PotentialDomainConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.potential_scale
        self.A_lane = config_relevant.A_lane
        self.A_car = config_relevant.A_car

        self.sigma_factor = config_relevant.sigma_factor
        self.scale_factor = config_relevant.scale_factor
        self.slope_scale = config_relevant.slope_scale

        self.alpha = config_relevant.alpha
        self.beta = config_relevant.beta
        self.d_0 = config_relevant.d_0

        self.follow_time = config_relevant.follow_time
        self.wedge_vertex = config_relevant.wedge_vertex
        self.desired_speed = config_relevant.desired_speed

        self.u_max = config_relevant.u_max


class ProbabilityDomainConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.probability_scale
        self.monte_carlo = ProbabilityDomainConfiguration.MonteCarlo(config_relevant)
        
    class MonteCarlo:
        def __init__(self, dict_config: Union[ListConfig, DictConfig]):
            dict_mc = dict_config.monte_carlo
            self.prediction_horizon = dict_mc.prediction_horizon
            self.nr_samples = dict_mc.nr_samples
            self.mvr_weights = dict_mc.weights


class IndexDomainConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.index_scale
        self.tci = IndexDomainConfiguration.TCI(config_relevant)

    class TCI:
        def __init__(self, dict_config: Union[ListConfig, DictConfig]):
            dict_tci = dict_config.TCI
            self.w_x = dict_tci.w_x
            self.w_y = dict_tci.w_y
            self.w_ax = dict_tci.w_ax
            self.w_ay = dict_tci.w_ay
            self.N = dict_tci.prediction_horizon


class VehicleConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.vehicle
        self.ego_id = config_relevant.ego_id

        # vehicle configuration in the curvilinear coordinate system
        self.curvilinear = VehicleConfiguration.Curvilinear(config_relevant)

        # vehicle configuration in the cartesian frame
        id_type_vehicle = config_relevant.id_type_vehicle
        self.cartesian = self.to_vehicle_parameter(id_type_vehicle)
        self.complete_cartesian_constraints(config_relevant)
        self.dynamic = PointMassDynamics(VehicleType(id_type_vehicle))

    def complete_cartesian_constraints(self, dict_config: Union[ListConfig, DictConfig]):
        dict_cartesian = dict_config.cartesian
        self.cartesian.j_x_min = dict_cartesian.j_x_min
        self.cartesian.j_x_max = dict_cartesian.j_x_max
        self.cartesian.j_y_min = dict_cartesian.j_y_min
        self.cartesian.j_y_max = dict_cartesian.j_y_max
        self.cartesian.a_x_min = dict_cartesian.a_x_min
        self.cartesian.a_x_max = dict_cartesian.a_x_max
        self.cartesian.a_y_min = dict_cartesian.a_y_min
        self.cartesian.a_y_max = dict_cartesian.a_y_max

    class Curvilinear:
        def __init__(self, dict_config: Union[ListConfig, DictConfig]):
            dict_curvilinear = dict_config.curvilinear
            self.clcs = None

            self.v_lon_min = dict_curvilinear.v_lon_min
            self.v_lon_max = dict_curvilinear.v_lon_max
            self.v_lat_min = dict_curvilinear.v_lat_min
            self.v_lat_max = dict_curvilinear.v_lat_max

            self.a_lon_max = dict_curvilinear.a_lon_max
            self.a_lon_min = dict_curvilinear.a_lon_min
            self.a_lat_max = dict_curvilinear.a_lat_max
            self.a_lat_min = dict_curvilinear.a_lat_min
            self.a_max = dict_curvilinear.a_max

            self.j_lon_min = dict_curvilinear.j_lon_min
            self.j_lon_max = dict_curvilinear.j_lon_max
            self.j_lat_min = dict_curvilinear.j_lat_min
            self.j_lat_max = dict_curvilinear.j_lat_max

            self.reference_point = dict_curvilinear.reference_point

    @staticmethod
    def to_vehicle_parameter(vehicle_type: str):
        if vehicle_type == 1:

            return parameters_vehicle1()
        elif vehicle_type == 2:
            return parameters_vehicle2()
        elif vehicle_type == 3:
            return parameters_vehicle3()
        else:
            raise TypeError(f"Vehicle type {vehicle_type} not supported!")

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, length: float):
        self._length = length

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, width: float):
        self._width = width


class DebugConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.debug

        self.save_plots = config_relevant.save_plots
        if config_relevant.plot_limits:
            self.plot_limits = list(config_relevant.plot_limits)
        else:
            self.plot_limits = None
        self.draw_visualization = config_relevant.draw_visualization
        self.draw_icons = config_relevant.draw_icons
