__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
import matplotlib.pyplot as plt

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndexScale
from commonroad_crime.metric.acceleration_scale.a_lat_req import ALatReq
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.optimization as utils_opt
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.visualization as utils_vis

logger = logging.getLogger(__name__)


class TCI(CriMeBase):
    """
    the Trajectory Criticality Index (TCI) models criticality using an optimization problem. The task is to find a
    minimum difficulty value, i.e. how demanding even the easiest option for the vehicle will be under a set of physical
    and regulatory constraints.

    -- from P. Junietz, F. Bonakdar, B. Klamann, and H. Winner, “Criticality metric for the safety validation of
    automated driving using model predictive trajectory optimization,” in 21st International Conference on Intelligent
    Transportation Systems (ITSC), pp. 60–65, IEEE, 2018.
    """
    metric_name = TypeIndexScale.TCI

    def __init__(self, config: CriMeConfiguration):
        super(TCI, self).__init__(config)
        self._optimizer = utils_opt.TCIOptimizer(config, self.sce)

    def compute(self, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        self.time_step = time_step
        self._optimizer.optimize(self.ego_vehicle, time_step)


    def visualize(self):
        pass