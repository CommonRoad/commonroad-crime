__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import multiprocessing
import os
from typing import List
import logging
import fnmatch
from enum import Enum, unique
from typing import Dict

from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.logger as utils_log
from commonroad.common.file_reader import CommonRoadFileReader

logger = logging.getLogger(__name__)


def initialize_process(config: CriMeConfiguration, flag_multi_processing: bool = True,):
    scenario_loader = ScenarioLoader(config.general.path_scenarios_batch)
    if flag_multi_processing:
        manager = multiprocessing.Manager()
        result_dict = manager.dict()
    else:
        result_dict = dict()
    result_dict["Success"] = list()
    result_dict["Failure"] = list()
    result_dict["Criticality"] = list()
    result_dict["scenarios_to_process"] = scenario_loader.scenario_ids
    result_dict["num_scenarios"] = len(scenario_loader.scenario_ids)
    result_dict["started_processing"] = 0
    return result_dict


def run_parallel(config: CriMeConfiguration):
    """
    Parallel batch evaluation of metrics, where the computation of criticality is carried out on multiple threads
    simultaneously. This reduces the runtime required to test your metric on more scenarios. One drawback is that it is
    not very easy to debug your code with parallel batch evaluation.
    """
    pass


def run_sequential():
    """
    Sequential batch evaluation of metrics, where the computation of criticality is carried out on a single thread.
    This is more user-friendly choice for test your metric on more scenarios since you can easily debug your code in
    your IDEs by creating breakpoints.
    """
    pass


class ScenarioLoader:
    def __init__(self, scenario_folder: str,):
        self.scenario_folder = scenario_folder
        self.scenario_ids = self._obtain_scenarios_with_path()

    def _obtain_scenarios_with_path(self) -> List:
        scenario_ids = list()
        extension_length = len('.xml')
        for path, directories, files in os.walk(self.scenario_folder):
            for scenario in fnmatch.filter(files, "*.xml"):
                scenario_ids.append(scenario[:-extension_length])
        if len(scenario_ids) == 0:
            utils_log.print_and_log_warning(logger, f"No Scenario found in directory: {self.scenario_folder}")
        utils_log.print_and_log_info(logger, f"Number of scenarios:{len(scenario_ids)}")
        return scenario_ids
