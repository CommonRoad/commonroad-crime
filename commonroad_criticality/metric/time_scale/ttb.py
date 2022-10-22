__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
from commonroad_criticality.utility.simulation import SimulationLong, Maneuver


class TTB(CriticalityBase):
    metric_name = TimeScaleMetricType.TTB
    
    def __init__(self, config: CriticalityConfiguration):
        super(TTB, self).__init__(config)
        self._sim_long = SimulationLong(Maneuver.BRAKE, self.ego_vehicle, config)

    def compute(self,  *args, **kwargs):
        pass

