__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
from decimal import Decimal
import matplotlib.pyplot as plt

from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
from commonroad_criticality.metric.time_scale.ttm import TTM
from commonroad_criticality.utility.simulation import Maneuver
import commonroad_criticality.utility.visualization as Utils_vis


class TTB(TTM):
    metric_name = TimeScaleMetricType.TTB
    
    def __init__(self, config: CriticalityConfiguration):
        super(TTB, self).__init__(config, Maneuver.BRAKE)

    def compute(self) -> Decimal:
        if self.ttc == 0:
            ttb = -math.inf
        elif self.ttc == Decimal(math.inf):
            ttb = Decimal(math.inf)
        else:
            ttb = Decimal(self.binary_search())

        if self.configuration.debug.draw_visualization:
            plt.title(f"{self.metric_name} at time step {int(ttb/Decimal(self.dt))}")
            Utils_vis.draw_cut_off_state(self.rnd, self.ego_vehicle.state_at_time(int(ttb/Decimal(self.dt))))
            if self.configuration.debug.save_plots:
                Utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, int(ttb/Decimal(self.dt)))
            else:
                plt.show()
        return ttb


