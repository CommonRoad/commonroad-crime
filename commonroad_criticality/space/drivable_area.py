
from commonroad.scenario.scenario import Scenario

from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface


from commonroad_criticality.base import CriticalityBase
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration


class DrivableAreaCriticality(CriticalityBase):
    def __init__(self, scenario: Scenario, id_vehicle: int = None, config: CriticalityConfiguration = None):
        super().__init__(scenario, id_vehicle, config)
        self.reachset_config = ConfigurationBuilder.build_configuration(str(scenario.scenario_id))
        self.reach_interface = self.init_reachability_analysis()

    def init_reachability_analysis(self):
        reach_interface = ReachableSetInterface(self.reachset_config)
        return reach_interface

    def compute(self):
        self.reach_interface.compute_reachable_sets()
        drivable_area = self.reach_interface.drivable_area
        pass

