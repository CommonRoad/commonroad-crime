__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import multiprocessing
import os
from typing import List, Type, Dict
import logging
import math
import csv
import fnmatch
from commonroad.scenario.obstacle import StaticObstacle

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.logger as utils_log
from commonroad.common.file_reader import CommonRoadFileReader

logger = logging.getLogger(__name__)


def initialize_process(batch_path: str, flag_multi_processing: bool = True,):
    scenario_loader = ScenarioLoader(batch_path)
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
    return scenario_loader, result_dict


def run_parallel(config: CriMeConfiguration):
    """
    Parallel batch evaluation of metrics, where the computation of criticality is carried out on multiple threads
    simultaneously. This reduces the runtime required to test your metric on more scenarios. One drawback is that it is
    not very easy to debug your code with parallel batch evaluation.
    """
    pass


def run_sequential(batch_path: str, measures: List[Type[CriMeBase]]):
    """
    Sequential batch evaluation of metrics, where the computation of criticality is carried out on a single thread.
    This is more user-friendly choice for test your metric on more scenarios since you can easily debug your code in
    your IDEs by creating breakpoints.
    """
    scenario_loader, result_dict = initialize_process(batch_path, flag_multi_processing=False)

    for scenario_id in scenario_loader.scenario_ids[0:1]:
        sce_res = dict()
        sce_conf = ConfigurationBuilder.build_configuration(scenario_id)
        sce_conf.update()
        for measure in measures:
            sce_res[measure.metric_name] = dict()
            for obs in sce_conf.scenario.obstacles:
                if isinstance(obs, StaticObstacle):
                    continue
                sce_conf.vehicle.ego_id = obs.obstacle_id
                # construct the measure evaluator
                measure_object = measure(sce_conf)

                sce_res[measure.metric_name][obs.obstacle_id] = dict()
                for ts in range(obs.prediction.initial_time_step, obs.prediction.final_time_step):
                    measure_value = math.inf
                    try:
                        for other_obs in sce_conf.scenario.obstacles:
                            if other_obs is not obs:
                                measure_value = min(measure_value,
                                                    measure_object.compute(time_step=ts,
                                                                           vehicle_id=other_obs.obstacle_id))
                    except Exception as err:
                        utils_log.print_and_log_error(logger, f"Evaluation failed, see {err}")
                    sce_res[measure.metric_name][obs.obstacle_id][ts] = measure_value
        result_dict[scenario_id] = sce_res


def write_result_to_csv(result_dict: Dict, batch_path: str):
    with open('outputFile.csv', 'a',newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(rowData)

class ScenarioLoader:
    def __init__(self, scenario_folder: str,):
        self.scenario_folder = scenario_folder
        self.scenario_ids = self._obtain_scenarios_with_path()

    def _obtain_scenarios_with_path(self) -> List:
        utils_log.print_and_log_info(logger, "Loading scenarios...")
        scenario_ids = list()
        extension_length = len('.xml')
        for path, directories, files in os.walk(self.scenario_folder):
            for scenario in fnmatch.filter(files, "*.xml"):
                scenario_ids.append(scenario[:-extension_length])
        if len(scenario_ids) == 0:
            utils_log.print_and_log_warning(logger, f"No Scenario found in directory: {self.scenario_folder}")
        utils_log.print_and_log_info(logger, f"Number of scenarios: {len(scenario_ids)}")
        return scenario_ids
