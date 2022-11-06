__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import logging

import numpy as np

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeAccelerationScale
from commonroad_crime.metric.time_scale.thw import THW
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class DST(CriMeBase):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    the deceleration that has to be applied to a vehicle to maintain a certain safety time
    """
    metric_name = TypeAccelerationScale.DST

    def __init__(self, config: CriMeConfiguration):
        super(DST, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        # under the assumption that the velocity of the other object remains constant
        pass

    def visualize(self):
        pass

