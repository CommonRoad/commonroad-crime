__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from typing import Union
from decimal import Decimal

from commonroad.scenario.obstacle import DynamicObstacle

from commonroad_criticality.metric.time_scale.ttc import TTC
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType

from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object


class TTR(TTC):
    metric_name = TimeScaleMetricType.TTR

    def __init__(self, config: CriticalityConfiguration):
        super(TTR, self).__init__(config)
        ttc_object = TTC(config)
        self._ttc = ttc_object.compute()

    def _detect_collision(self, vehicle: DynamicObstacle) -> bool:
        """
        detect whether the vehicle collides.
        """
        vehicle_co = create_collision_object(vehicle)
        return self.collision_checker.collide(vehicle_co)

    def compute(self) -> Union[Decimal]:
        # todo: time to steer, brake, kick-off all together
        pass
