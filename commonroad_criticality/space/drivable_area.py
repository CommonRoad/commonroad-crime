
from commonroad.scenario.scenario import Scenario
from commonroad_criticality.base import CriticalityBase
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration


class DrivableAreaCriticality(CriticalityBase):
    def __init__(self, scenario: Scenario, id_vehicle: int = None, config: CriticalityConfiguration = None):
        super().__init__(scenario, id_vehicle, config)