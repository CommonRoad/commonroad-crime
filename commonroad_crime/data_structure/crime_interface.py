__author__ = "Yuanfei Lin, Marius Erath"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import os
import logging
from typing import List, Type
from lxml import etree

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

    def evaluate_scene(
        self,
        measures: List[Type[CriMeBase]],
        time_step: int = 0,
        vehicle_id: int = None,
        verbose: bool = True,
    ):
        """
        Evaluate the criticality of given measures
        """
        utils_log.print_and_log_info(
            logger,
            f"* Given measures for time step {time_step}: "
            f"{', '.join([measure.measure_name.value for measure in measures])}...",
            verbose,
        )
        if time_step not in self.criticality_dict:
            self.criticality_dict[time_step] = {}
        for measure in measures:
            if measure not in self.measures:
                self.measures.append(measure)
            m_evaluator = measure(self.config)
            if measure.measure_name.value not in self.criticality_dict[time_step]:
                self.criticality_dict[time_step][measure.measure_name.value] = (
                    m_evaluator.compute_criticality(
                        time_step, vehicle_id, verbose=verbose
                    )
                )
                self.measure_evaluators.append(m_evaluator)
        # printing out the summary of the evaluations
        utils_log.print_and_log_info(
            logger, "*********************************", verbose
        )
        utils_log.print_and_log_info(logger, "\t Summary:", verbose)
        utils_log.print_and_log_info(
            logger,
            "\n".join(
                "* {}: {}".format(m, value)
                for m, value in self.criticality_dict[time_step].items()
            ),
            verbose,
        )

    def evaluate_scenario(
        self,
        measures: List[Type[CriMeBase]],
        time_start: int = 0,
        time_end: int = 1,
        vehicle_id: int = None,
        verbose: bool = True,
    ):
        # Check if time_start is larger than time_end
        if time_start > time_end:
            utils_log.print_and_log_error(
                logger,
                f"* Time start must not be larger than time end, but {time_start} > {time_end}",
            )
            return
        utils_log.print_and_log_info(
            logger,
            f"* Given measures for the whole scenario: "
            f"{', '.join([measure.measure_name.value for measure in measures])}...",
            verbose,
        )
        (
            self.time_start,
            self.time_end,
        ) = (
            time_start,
            time_end,
        )
        for time_step in range(time_start, time_end + 1):
            self.evaluate_scene(measures, time_step, vehicle_id, verbose=verbose)
        # printing out the summary of the evaluations
        utils_log.print_and_log_info(
            logger, "*********************************", verbose
        )
        utils_log.print_and_log_info(logger, "\t Summary:", verbose)
        for time_step in range(time_start, time_end + 1):
            utils_log.print_and_log_info(
                logger,
                "* At time step {}: ".format(time_step)
                + ", ".join(
                    "{} = {}".format(m, value)
                    for m, value in self.criticality_dict[time_step].items()
                ),
                verbose,
            )

    def visualize(self, time_step: int = None):
        self.config.debug.draw_visualization = True
        for m_evaluator in self.measure_evaluators:
            if time_step is None:
                m_evaluator.visualize()
            else:
                if m_evaluator.time_step == time_step:
                    m_evaluator.visualize()

    def save_to_file(self, output_dir: str):
        """
        Saves the criticality measures of a scenario to an XML file.

        This method serializes the scenario information along with the related
        criticality measures into an XML structure and saves it to a file. The
        XML file is structured with a root element and child elements that include
        the scenario details, parameters, and measures listed over all timesteps.

        Parameters:
        output_dir (str): The directory where the XML file will be saved.

        The XML file is named using the scenario ID with the prefix 'CriMe-'
        and the suffix '_veh_123' where 123 is the ID of the vehicle with respect
        to which the criticality was computed (ego vehicle).
        The file will include pretty-printed XML for better readability.
        """
        # Ensure the output directory exists
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)  # Create the directory if it does not exist

        scenario = self.config.scenario
        root = etree.Element("commonroad-criticality-measures")

        # Add scenario info
        scenario_node = etree.SubElement(root, "scenario")
        scenario_node.set("timeStepSize", str(scenario.dt))
        scenario_node.set("commonRoadVersion", scenario.scenario_id.scenario_version)
        scenario_node.set("author", scenario.author)
        scenario_node.set("affiliation", scenario.affiliation)
        scenario_node.set("source", scenario.source)
        scenario_node.set("benchmarkID", str(scenario.scenario_id))

        # Add parameter info
        parameter_node = etree.SubElement(root, "parameters")
        parameter_node.set("egoID", str(self.config.vehicle.ego_id))

        # Add list of measures
        measure_list = etree.SubElement(root, "measure_list")
        for measure in self.measures:
            etree.SubElement(
                measure_list, "measure", {"name": measure.measure_name.value}
            )

        # Add data node
        data_node = etree.SubElement(root, "data")
        # Add all timesteps
        for timestep, measure_dict in self.criticality_dict.items():
            timestep_node = etree.SubElement(
                data_node, "timestep", {"timestep": str(timestep)}
            )
            # Add measure results in each time-step
            for measure_name, value in self.criticality_dict[timestep].items():
                etree.SubElement(
                    timestep_node,
                    "measure_value",
                    {"name": measure_name, "value": str(value)},
                )

        # Save to file
        tree = etree.ElementTree(root)
        tree.write(
            os.path.join(output_dir, f"CriMe-{scenario.scenario_id}_veh_{self.config.vehicle.ego_id}.xml"),
            pretty_print=True,
            xml_declaration=True,
            encoding="utf-8",
        )
