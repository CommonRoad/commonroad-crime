__author__ = "Yuanfei Lin, Sicheng Wang"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

from multiprocessing import Manager, Pool

import os
from typing import List, Type, Dict
import logging
import math
import time
import csv
from tqdm import tqdm
from commonroad.scenario.obstacle import StaticObstacle

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


def initialize_process(
    batch_path: str,
    flag_multi_processing: bool = True,
):
    """
    Initialize the scenario process
    :param batch_path: the given scenario path
    :param flag_multi_processing: flag for whether multiprocessing is needed
    :return: scenario loader, dict for saving the evaluation result
    """
    scenario_loader = ScenarioLoader(batch_path)
    if flag_multi_processing:
        manager = Manager()
        result_dict = manager.dict()
        # result_dict["started_processing"] = 0
    else:
        result_dict = dict()
    # result_dict["Success"] = list()
    # result_dict["Failure"] = list()
    # result_dict["Criticality"] = list()
    # result_dict["scenarios_to_process"] = scenario_loader.scenario_ids
    # result_dict["num_scenarios"] = len(scenario_loader.scenario_ids)
    # result_dict["started_processing"] = 0
    return scenario_loader, result_dict


def process_scenario(
    sce_conf: CriMeConfiguration,
    measures: List[Type[CriMeBase]],
    result_dict: Dict,
    verbose: bool,
):
    scenario_id = sce_conf.general.name_scenario
    for measure in measures:
        sce_res = dict()
        sce_res[measure.measure_name] = dict()
        for obs in sce_conf.scenario.obstacles:
            if isinstance(obs, StaticObstacle):
                continue
            sce_conf.vehicle.ego_id = obs.obstacle_id
            # construct the measures evaluator
            try:
                measure_object = measure(sce_conf)
            except Exception as err:
                utils_log.print_and_log_error(
                    logger, f"Initialization failed {scenario_id}, see {err}", verbose
                )
                continue
            sce_res[measure.measure_name][obs.obstacle_id] = dict()
            if not isinstance(obs.prediction.initial_time_step, int) or not isinstance(
                obs.prediction.final_time_step, int
            ):
                continue
            for ts in range(
                obs.prediction.initial_time_step, obs.prediction.final_time_step
            ):
                measure_value = math.inf
                try:
                    time_start = time.time()
                    measure_value = measure_object.compute_criticality(
                        ts, verbose=verbose
                    )
                    calc_time = time.time() - time_start
                except Exception as err:
                    calc_time = math.nan
                    utils_log.print_and_log_error(
                        logger,
                        f"Evaluation failed {scenario_id}:{obs.obstacle_id}, see {err}",
                        verbose,
                    )
                sce_res[measure.measure_name][obs.obstacle_id][ts] = [
                    measure_value,
                    calc_time,
                ]
        result_dict[scenario_id] = sce_res


def load_config(config_root: str, scenario_id: str):
    """Loads configuration file, if it does not exist, use the default one"""
    if not config_root or not os.path.exists(f"{config_root}/{scenario_id}.yaml"):
        utils_log.print_and_log_info(
            logger,
            f"The configuration root is not provided or config file doesn't exist, "
            f"thus the default configuration is invoked",
        )
        configuration = CriMeConfiguration()
        configuration.general.set_scenario_name(scenario_id)
    else:
        configuration = CriMeConfiguration.load(
            f"{config_root}/{scenario_id}.yaml", scenario_id
        )
    return configuration


def run_parallel(
    scenario_path: str,
    measures: List[Type[CriMeBase]],
    config_root: str = None,
    num_worker: int = 16,
    verbose: bool = False,
):
    """
    Parallel batch evaluation of measures, where the computation of criticality is carried out on multiple threads
    simultaneously. This reduces the runtime required to test your metric on more scenarios. One drawback is that it is
    not very easy to debug your code with parallel batch evaluation.
    """
    config = CriMeConfiguration()
    utils_log.initialize_logger(config)

    scenario_loader, result_dict = initialize_process(
        scenario_path, flag_multi_processing=True
    )

    pool = Pool(num_worker)

    pbar = tqdm(
        desc="Scenarios Finished: ",
        total=len(scenario_loader.scenario_ids),
        colour="green",
    )

    def update(*a):
        pbar.update()

    for i in range(pbar.total):
        scenario_id, file_path = scenario_loader.scenario_ids[i]
        utils_log.print_and_log_error(
            logger, f"Evaluation of scenario {scenario_id}", verbose
        )
        sce_conf = load_config(config_root, scenario_id)
        sce_conf.general.path_scenarios = file_path
        sce_conf.update()
        pool.apply_async(
            process_scenario,
            args=(sce_conf, measures, result_dict, verbose),
            callback=update,
        )

    pool.close()
    pool.join()

    utils_log.print_and_log_info(logger, f"All Processes Done.")
    write_result_to_csv(result_dict, scenario_path)


def run_sequential(
    scenario_path: str,
    measures: List[Type[CriMeBase]],
    config_root: str = None,
    verbose: bool = False,
):
    """
    Sequential batch evaluation of measures, where the computation of criticality is carried out on a single thread.
    This is more user-friendly choice for test your metric on more scenarios since you can easily debug your code in
    your IDEs by creating breakpoints.
    """
    scenario_loader, result_dict = initialize_process(
        scenario_path, flag_multi_processing=False
    )
    config = CriMeConfiguration()
    utils_log.initialize_logger(config)

    for scenario_id, file_path in tqdm(
        scenario_loader.scenario_ids, desc="Scenarios Finished: ", colour="red"
    ):
        utils_log.print_and_log_error(
            logger, f"Evaluation of scenario {scenario_id}", verbose
        )
        sce_res = dict()
        sce_conf = load_config(config_root, scenario_id)
        sce_conf.general.path_scenarios = file_path
        sce_conf.update()
        for measure in measures:
            sce_res[measure.measure_name] = dict()
            for obs in sce_conf.scenario.obstacles:
                if isinstance(obs, StaticObstacle):
                    continue
                sce_conf.vehicle.ego_id = obs.obstacle_id
                # construct the measures evaluator
                try:
                    measure_object = measure(sce_conf)
                except Exception as err:
                    utils_log.print_and_log_error(
                        logger,
                        f"Initialization failed {scenario_id}, see {err}",
                        verbose,
                    )
                    continue
                sce_res[measure.measure_name][obs.obstacle_id] = dict()
                if not isinstance(
                    obs.prediction.initial_time_step, int
                ) or not isinstance(obs.prediction.final_time_step, int):
                    continue
                for ts in range(
                    obs.prediction.initial_time_step, obs.prediction.final_time_step
                ):
                    measure_value = math.inf
                    try:
                        time_start = time.time()
                        measure_value = measure_object.compute_criticality(
                            ts, verbose=verbose
                        )
                        calc_time = time.time() - time_start
                    except Exception as err:
                        utils_log.print_and_log_error(
                            logger,
                            f"Evaluation failed {scenario_id}:{obs.obstacle_id}, see {err}",
                            verbose,
                        )
                        calc_time = math.nan
                        pass
                    sce_res[measure.measure_name][obs.obstacle_id][ts] = [
                        measure_value,
                        calc_time,
                    ]
        result_dict[scenario_id] = sce_res
    write_result_to_csv(result_dict, scenario_path)


def write_result_to_csv(result_dict: Dict, batch_path: str):
    with open(batch_path + "evaluation_result.csv", "a", newline="") as csv_file:
        writer = csv.writer(csv_file)
        for sce_id, sce_res in result_dict.items():
            for measure, evaluation in sce_res.items():
                for veh_id, eva_detail in evaluation.items():
                    for ts, result in eva_detail.items():
                        writer.writerow(
                            [sce_id, measure, veh_id, ts, result[0], result[1]]
                        )


class ScenarioLoader:
    def __init__(
        self,
        scenario_folder: str,
    ):
        self.scenario_folder = scenario_folder
        self.scenario_ids = self._obtain_scenarios_with_path()

    def _obtain_scenarios_with_path(self) -> List:
        utils_log.print_and_log_info(logger, "Loading scenarios...")
        scenario_ids = list()
        for root, dirs, files in os.walk(self.scenario_folder):
            for file in files:
                if file.endswith(".xml"):
                    scenario_id = os.path.splitext(file)[0]
                    scenario_ids.append((scenario_id, root + "/"))
        if len(scenario_ids) == 0:
            utils_log.print_and_log_warning(
                logger, f"No Scenario found in directory: {self.scenario_folder}"
            )
        utils_log.print_and_log_info(
            logger, f"Number of scenarios: {len(scenario_ids)}"
        )
        return scenario_ids
