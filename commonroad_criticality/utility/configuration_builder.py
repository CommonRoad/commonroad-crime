import glob
import logging
import os
import copy
from collections import defaultdict
from typing import Dict

import yaml

from commonroad_criticality.data_structure.configuration import CriticalityConfiguration

LOGGER = logging.getLogger(__name__)


class ConfigurationBuilder:
    path_root: str = None
    path_config: str = None
    path_config_default: str = None
    dict_config_overridden: dict = None

    @classmethod
    def set_root_path(cls, criticality_root: str,
                      path_to_config: str = "config_files",
                      dir_config_default: str = "default"):
        """Sets the path to the root directory
            :param criticality_root: root directory
            :param path_to_config: relative path of configurations to root path
            :param dir_config_default: directory under root folder containing default config files
        """
        assert os.path.exists(criticality_root)

        cls.path_root = criticality_root
        cls.path_config = os.path.join(cls.path_root, path_to_config)
        assert os.path.exists(cls.path_config)

        cls.path_config_default = os.path.join(cls.path_config, dir_config_default)
        assert os.path.exists(cls.path_config_default)

    @classmethod
    def build_configuration(cls, scenario_id: str):
        """Builds configuration from the default and scenario-specific config files"""
        # load default config files
        dict_config_default = cls.construct_default_config_dict()
        # Override if scenario-specific config file exists
        cls.dict_config_overridden = cls.override_with_scenario_config(dict_config_default, scenario_id)
        # convert to default dict
        cls.dict_config_overridden = defaultdict(lambda: None, cls.dict_config_overridden)
        # build the configuration object
        config = CriticalityConfiguration(cls.dict_config_overridden)
        return config

    @classmethod
    def construct_default_config_dict(cls) -> Dict:
        """Construct default config dictionary with partial config files.

        Collects all config files ending with 'yaml* under path_config_default.
        """

        dict_config_default = dict()
        for path_file in glob.glob(cls.path_config_default + "/*.yaml"):
            with open(path_file, "r") as file_config:
                try:
                    dict_config = yaml.load(file_config, Loader=yaml.Loader)
                except yaml.YAMLError as e:
                    LOGGER.debug(e)
                else:
                    dict_config_default = {**dict_config_default, **dict_config}

        cls.rectify_directories(dict_config_default)

        return dict_config_default

    @classmethod
    def rectify_directories(cls, dict_config):
        """Rectifies directories (converts from relative path to absolute path). """
        for key, path in dict_config["config_general"].items():
            if "path" not in key:
                continue

            path_relative = os.path.join(cls.path_root, path)
            if os.path.exists(path_relative):
                dict_config["config_general"][key] = path_relative
            else:
                continue

        dict_config["config_general"]["path_root"] = cls.path_root

    @classmethod
    def override_with_scenario_config(cls, dict_config_default: Dict, name_scenario: str) -> Dict:
        """Overrides default config file with scenario-specific config files.
            :param dict_config_default: dictionary created from default config files
            :param name_scenario: considered scenario
            return: dictionary containing overridden entries
        """
        dict_config_overridden = copy.deepcopy(dict_config_default)

        path_config_scenario = os.path.join(cls.path_config, f"{name_scenario}.yaml")
        if os.path.exists(path_config_scenario):
            with open(path_config_scenario, "r") as file_config:
                try:
                    dict_config_scenario = yaml.load(file_config, Loader=yaml.Loader)
                except yaml.YAMLError as e:
                    LOGGER.debug(e)
                else:
                    cls.override_nested_dicts(dict_config_overridden, dict_config_scenario)

        # add scenario name to the config file
        dict_config_overridden["config_general"]["name_scenario"] = name_scenario

        return dict_config_overridden

    @classmethod
    def override_nested_dicts(cls, dict1: Dict, dict2: Dict) -> Dict:
        """Recursively overrides a dictionary with another dictionary.

        Args:
            dict1 (Dict): dictionary to be overridden
            dict2 (Dict): dictionary to provide new values

        Returns:
            Dict: Overridden dictionary
        """
        for key, val in dict2.items():
            if isinstance(val, Dict):
                dict1[key] = cls.override_nested_dicts(dict1.get(key, {}), val)
            else:
                dict1[key] = val

        return dict1
