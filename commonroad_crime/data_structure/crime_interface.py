import logging
from typing import List, Type

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class CriMeInterface:
    """
    Interface for Criticality Measures
    """

    def __init__(self, config: CriMeConfiguration):
        self.config = config
        self.criticality_dict = dict()

    def evaluate_scene(self, metrics: List[Type[CriMeBase]],
                       time_step: int = 0,
                       vehicle_id: int = None,
                       verbose: bool = True):
        """
        Evaluate the criticality of given metrics
        """
        utils_log.print_and_log_info(logger, f"* Given metrics for time step {time_step}: "
                                             f"{', '.join([metric.metric_name.value for metric in metrics])}...",
                                     verbose)
        self.criticality_dict[time_step] = {}
        for metric in metrics:
            m_evaluator = metric(self.config)
            self.criticality_dict[time_step][metric.metric_name] = m_evaluator.compute_criticality(time_step,
                                                                                                   vehicle_id, verbose)
        # printing out the summary of the evaluations
        utils_log.print_and_log_info(logger, "*********************************", verbose)
        utils_log.print_and_log_info(logger, "\t Summary:", verbose)
        utils_log.print_and_log_info(logger,
                                     '\n'.join(
                                         '* {}: {}'.format(m, value) for m, value in
                                         self.criticality_dict[time_step].items()),
                                     verbose)

    def evaluate_scenario(self, metrics: List[Type[CriMeBase]],
                          time_start: int = 0,
                          time_end: int = 1,
                          vehicle_id: int = None,
                          verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Given metrics for the whole scenario: "
                                             f"{', '.join([metric.metric_name.value for metric in metrics])}...",
                                     verbose)
        for time_step in range(time_start, time_end):
            self.evaluate_scene(metrics, time_step, vehicle_id, verbose=False)
        # printing out the summary of the evaluations
        utils_log.print_and_log_info(logger, "*********************************", verbose)
        utils_log.print_and_log_info(logger, "\t Summary:", verbose)
        for time_step in range(time_start, time_end):
            utils_log.print_and_log_info(logger, '* At time step {}: '.format(time_step) +
                                         ', '.join(
                                             '{} = {}'.format(m, value) for m, value in
                                             self.criticality_dict[time_step].items()),
                                         verbose)
