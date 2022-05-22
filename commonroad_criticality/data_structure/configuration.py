import logging
from typing import Union
from omegaconf import ListConfig, DictConfig

from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3

logger = logging.getLogger(__name__)


class CriticalityConfiguration:
    """Class to hold criticality-related configurations"""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.general: GeneralConfiguration = GeneralConfiguration(config)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config)
        self.time_metrics: TimeBasedConfiguration = TimeBasedConfiguration(config)
        self.space_metrics: SpaceBasedConfiguration = SpaceBasedConfiguration(config)

    def print_configuration_summary(self):
        string = "# ===== Configuration Summary ===== #\n"
        string += f"# {self.general.name_scenario}\n"
        string += "# Time metrics:\n"
        if self.time_metrics.activated == 1:
            time_metric = " "
            if "1" in self.time_metrics.metric:
                time_metric += "time-to-collision\t"
            if "2" in self.time_metrics.metric:
                time_metric += "time-to-react\t"
            if "3" in self.time_metrics.metric:
                time_metric += "time-to-violation\t"
            if "4" in self.time_metrics.metric:
                time_metric += "time-to-comply\t"
            string += f"# \tmetric: {time_metric}\n"
        else:
            string += "# \tnot activated\n"
        string += "# Space Metrics:\n"
        if self.space_metrics.activated == 1:
            if self.space_metrics.approach == 1:
                space_approach = "Area of drivable area"
            else:
                space_approach = "not defined"
            string += f"# \tapproach: {space_approach}\n"
        else:
            string += "# \tnot activated\n"
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


class VehicleConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.vehicle
        self.id_vehicle = None

        # vehicle configuration in the curvilinear coordinate system
        # todo: check whether this is actually runable
        self.curvilinear = VehicleConfiguration.Curvilinear(config_relevant)

        # vehicle configuration in the cartesian frame
        id_type_vehicle = config_relevant.id_type_vehicle
        self.cartesian = self.to_vehicle_parameter(id_type_vehicle)

        # vehicle size todo: need to be updated with the scenario
        self.width = self.cartesian.w
        self.length = self.cartesian.l

    class Curvilinear:
        def __init__(self, dict_config: Union[ListConfig, DictConfig]):
            dict_curvilinear = dict_config.curvilinear

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
