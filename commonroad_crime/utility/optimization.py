__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import casadi as ca
from typing import List
from abc import abstractmethod

from commonroad.scenario.scenario import State
from commonroad_crime.data_structure.configuration import CriMeConfiguration


class OptimizerBase:
    def __init__(self, config: CriMeConfiguration):
        self.config = config

    @abstractmethod
    def cost_function(self,  *args, **kwargs):
        pass

    @abstractmethod
    def constraints(self, *args, **kwargs):
        pass

    @abstractmethod
    def optimize(self,  *args, **kwargs):
        pass


class TCIOptimizer(OptimizerBase):
    def __init__(self, config: CriMeConfiguration):
        super(TCIOptimizer, self).__init__(config)
        self.tci_config = self.config.index_scale.tci
        self.veh_config = self.config.vehicle

    def constraints(self, *args, **kwargs):
        pass

    def cost_function(self,  *args, **kwargs):
        pass

    def optimize(self, x_initial: State,
                 ref_state_list: List[State]):
        pass
