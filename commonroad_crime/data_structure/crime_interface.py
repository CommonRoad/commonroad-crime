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
        self.time_start = 0
        self.time_end = 0
        self.measures = []
        self.measure_evaluators = []

    def evaluate_scene(self, measures: List[Type[CriMeBase]],
                       time_step: int = 0,
                       vehicle_id: int = None,
                       verbose: bool = True):
        """
        Evaluate the criticality of given measures
        """
        utils_log.print_and_log_info(logger, f"* Given measures for time step {time_step}: "
                                             f"{', '.join([measure.measure_name.value for measure in measures])}...",
                                     verbose)
        if time_step not in self.criticality_dict:
            self.criticality_dict[time_step] = {}
        for measure in measures:
            if measure not in self.measures:
                self.measures.append(measure)
            m_evaluator = measure(self.config)
            if measure.measure_name.value not in self.criticality_dict[time_step]:
                self.criticality_dict[time_step][measure.measure_name.value] = \
                    m_evaluator.compute_criticality(time_step,
                                                    vehicle_id,
                                                    verbose)
                self.measure_evaluators.append(m_evaluator)
        # printing out the summary of the evaluations
        utils_log.print_and_log_info(logger, "*********************************", verbose)
        utils_log.print_and_log_info(logger, "\t Summary:", verbose)
        utils_log.print_and_log_info(logger,
                                     '\n'.join(
                                         '* {}: {}'.format(m, value) for m, value in
                                         self.criticality_dict[time_step].items()),
                                     verbose)

    def evaluate_scenario(self, measures: List[Type[CriMeBase]],
                          time_start: int = 0,
                          time_end: int = 1,
                          vehicle_id: int = None,
                          verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Given measures for the whole scenario: "
                                             f"{', '.join([measure.measure_name.value for measure in measures])}...",
                                     verbose)
        self.time_start, self.time_end, = time_start, time_end
        for time_step in range(time_start, time_end + 1):
            self.evaluate_scene(measures, time_step, vehicle_id, verbose=False)
        # printing out the summary of the evaluations
        utils_log.print_and_log_info(logger, "*********************************", verbose)
        utils_log.print_and_log_info(logger, "\t Summary:", verbose)
        for time_step in range(time_start, time_end + 1):
            utils_log.print_and_log_info(logger, '* At time step {}: '.format(time_step) +
                                         ', '.join(
                                             '{} = {}'.format(m, value) for m, value in
                                             self.criticality_dict[time_step].items()),
                                         verbose)

    def visualize(self, time_step: int = None):
        for m_evaluator in self.measure_evaluators:
            if time_step is None:
                m_evaluator.visualize()
            else:
                if m_evaluator.time_step == time_step:
                    m_evaluator.visualize()



