from typing import List, Union, Dict

import numpy as np

from commonroad.scenario.scenario import Scenario

from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.pycrreach import ReachPolygon, ReachNode
from commonroad_reach.utility import geometry as util_geometry

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
        """Computes critically using size of drivable area constrained by obstacles compared to
        unconstrained drivable area."""
        # Drivable area
        self.reach_interface.compute_reachable_sets()
        drivable_area = self.reach_interface.drivable_area
        area_profile = self.compute_drivable_area_profile(drivable_area)

        # reference (without obstacles) todo: find a more efficient way to do so
        dyn_obs = self.reach_interface.config.scenario.dynamic_obstacles
        sta_obs = self.reach_interface.config.scenario.static_obstacles
        self.reachset_config.scenario.remove_obstacle(sta_obs)
        self.reachset_config.scenario.remove_obstacle(dyn_obs)
        self.reach_interface = self.init_reachability_analysis()
        self.reach_interface.compute_reachable_sets()
        drivable_area = self.reach_interface.drivable_area
        area_profile_reference = self.compute_drivable_area_profile(drivable_area)
        return self._criticality_metric(area_profile, area_profile_reference)

    def compute_drivable_area_profile(self, drivable_area: Dict[int, List[Union[ReachPolygon, ReachNode]]]):
        """Computes area profile for given reachability analysis."""
        area_profile = []
        for t, drivable_t in drivable_area.items():
            area_profile.append(util_geometry.area_of_reachable_set(drivable_t))
        return np.array(area_profile)

    def _criticality_metric(self, area_profile, reference_area_profile):
        assert np.size(area_profile) <= np.size(reference_area_profile)
        if any(np.isnan(area_profile)):
            return np.inf

        return np.mean(np.divide(area_profile, reference_area_profile[:np.size(area_profile)]))
