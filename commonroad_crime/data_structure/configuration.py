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
    """Class to hold criticality-measure-related configurations"""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.scenario: Optional[Scenario] = None
        self.scene: Optional[Scene] = None
        self.general: GeneralConfiguration = GeneralConfiguration(config)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config)
        self.debug: DebugConfiguration = DebugConfiguration(config)

        self.time_metrics: TimeBasedConfiguration = TimeBasedConfiguration(config)
        self.space_metrics: SpaceBasedConfiguration = SpaceBasedConfiguration(config)
        self.reachable_set_scale: ReachableSetScaleConfiguration = ReachableSetScaleConfiguration(config)

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
        else:
            self.scenario = utils_general.load_scenario(self)  # if none is provided, scenario is at default
        self.vehicle.curvilinear.clcs = CLCS
        if ego_id:
            if self.scenario.obstacle_by_id(ego_id) is None or self.scene.obstacle_by_id(ego_id):
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
        self.path_scenario = config_relevant.path_scenarios + name_scenario + ".xml"
        self.path_output = config_relevant.path_output + name_scenario + "/"
        self.path_logs = config_relevant.path_logs


class SpaceBasedConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.space_metrics
        self.activated = config_relevant.activated
        self.approach = config_relevant.approach


class TimeBasedConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.time_metrics
        self.activated = config_relevant.activated
        self.metric = config_relevant.metric
        self.steer_width = config_relevant.steer_width


class ReachableSetScaleConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.reachable_set_scale
        self.time_horizon = config_relevant.time_horizon


class VehicleConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.vehicle
        self.ego_id = config_relevant.ego_id

        # vehicle configuration in the curvilinear coordinate system
        # fixme: check whether this is actually runable
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
            self.clcs = None  # fixme: other assignment?

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
        self.draw_visualization = config_relevant.draw_visualization
        self.draw_icons = config_relevant.draw_icons

        # self.save_config = config_relevant.save_config
        # self.verbose_debug = config_relevant.verbose_debug
        # self.verbose_info = config_relevant.verbose_info
        # self.draw_ref_path = config_relevant.draw_ref_path
        # self.draw_planning_problem = config_relevant.draw_planning_problem
        # self.draw_icons = config_relevant.draw_icons
        # self.draw_lanelet_labels = config_relevant.draw_lanelet_labels
        # self.plot_limits = config_relevant.plot_limits
        # self.plot_azimuth = config_relevant.plot_azimuth
        # self.plot_elevation = config_relevant.plot_elevation
        # self.ax_distance = config_relevant.ax_distance
