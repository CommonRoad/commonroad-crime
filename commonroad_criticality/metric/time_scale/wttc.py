__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import matplotlib.pyplot as plt
from typing import Union, List
import logging

from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration


class WTTC(CriticalityBase):
    """
    The worst-time-to-collision metric extends the usual TTC by considering multiple traces of actors as predicted by an
    over-approximating dynamic motion model. From: Wachenfeld, Walther, et al. "The worst-time-to-collision metric
    for situation identification." 2016 IEEE Intelligent Vehicles Symposium (IV). IEEE, 2016.
    """
    metric_name = TimeScaleMetricType.WTTC
    
    def __init__(self, config: CriticalityConfiguration):
        super(WTTC, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        self.time_step = time_step

    def _wttc_st_obs(self):

    def _wttc_dy_obs(self):

    def visualize(self):
        pass



