import glob
import logging
import os.path
from typing import Dict

import yaml

from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3

LOGGER = logging.getLogger(__name__)


class CriticalityConfiguration:
    """Class to hold criticality-related configurations"""

    @classmethod
    def load_default_configuration(cls, default_configuration_files_path):
        assert os.path.exists(default_configuration_files_path)

        dict_config_default = dict()
        for path_file in glob.glob(os.path.join(default_configuration_files_path, "*.yaml")):
            with open(path_file, "r") as file_config:
                try:
                    dict_config = yaml.load(file_config, Loader=yaml.Loader)
                except yaml.YAMLError as e:
                    LOGGER.debug(e)
                else:
                    dict_config_default = {**dict_config_default, **dict_config}

        return cls(dict_config_default)

    def __init__(self, dict_config_criticality: Dict):
        self.config_general = GeneralConfiguration(dict_config_criticality)
        self.config_vehicle = VehicleConfiguration(dict_config_criticality)
        self.config_time_metrics = TimeBasedConfiguration(dict_config_criticality)
        self.config_space_metrics = SpaceBasedConfiguration(dict_config_criticality)


class GeneralConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_general"]
        name_scenario = dict_relevant["name_scenario"]

        self.path_root = dict_relevant["path_root"]
        self.path_scenarios = dict_relevant["path_scenarios"]
        self.path_plots = dict_relevant["path_plots"]

        self.path_scenario = self.path_scenarios + name_scenario + ".xml"


class SpaceBasedConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_space_metrics"]


class TimeBasedConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_time_metrics"]


class VehicleConfiguration:
    def __init__(self, dict_config: Dict):
        self.id_vehicle = None
        dict_relevant = dict_config["config_vehicle"]

        # vehicle configuration in the curvilinear coordinate system
        self.curvilinear = VehicleConfiguration.Curvilinear(dict_config)

        # vehicle configuration in the cartesian frame
        id_type_vehicle = dict_relevant["id_type_vehicle"]
        self.cartesian = self.to_vehicle_parameter(id_type_vehicle)

        # vehicle size todo: need to be updated with the scenario
        self.width = self.cartesian.w
        self.length = self.cartesian.l

    class Curvilinear:
        def __init__(self, dict_config: Dict):
            dict_curvilinear = dict_config["config_vehicle"]["curvilinear"]

            self.length = dict_curvilinear["length"]
            self.width = dict_curvilinear["width"]

            self.v_lon_min = dict_curvilinear["v_lon_min"]
            self.v_lon_max = dict_curvilinear["v_lon_max"]
            self.v_lat_min = dict_curvilinear["v_lat_min"]
            self.v_lat_max = dict_curvilinear["v_lat_max"]

            self.a_lon_max = dict_curvilinear["a_lon_max"]
            self.a_lon_min = dict_curvilinear["a_lon_min"]
            self.a_lat_max = dict_curvilinear["a_lat_max"]
            self.a_lat_min = dict_curvilinear["a_lat_min"]
            self.a_max = dict_curvilinear["a_max"]

            self.j_lon_min = dict_curvilinear["j_lon_min"]
            self.j_lon_max = dict_curvilinear["j_lon_max"]
            self.j_lat_min = dict_curvilinear["j_lat_min"]
            self.j_lat_max = dict_curvilinear["j_lat_max"]

            self.reference_point = dict_curvilinear["reference_point"]

    @staticmethod
    def to_vehicle_parameter(vehicle_type: str):
        if vehicle_type == "1":
            return parameters_vehicle1()
        elif vehicle_type == "2":
            return parameters_vehicle2()
        elif vehicle_type == "3":
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
