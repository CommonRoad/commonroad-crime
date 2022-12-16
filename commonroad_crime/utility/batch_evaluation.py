__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import os
import fnmatch
from typing import Dict

from commonroad_crime.data_structure.configuration import CriMeConfiguration

from commonroad.common.file_reader import CommonRoadFileReader


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

    def load_scenarios(self, scenario_id: str, verbose: bool = False):
        pass

    def _obtain_scenarios_with_path(self):
        scenarios: Dict[str, str] = dict()
        for path, directories, files in os.walk(self.scenario_folder):
            for scenario in fnmatch.filter(files, "*.xml"):
                pass
