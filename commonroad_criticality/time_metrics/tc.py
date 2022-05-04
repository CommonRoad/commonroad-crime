from typing import Union, List, Any, Tuple
from collections import defaultdict
import math
import functools
from abc import ABC

import numpy as np
from commonroad.scenario.obstacle import State

from commonroad_criticality.time_metrics.base import CutOffBase
from commonroad_criticality.time_metrics.monitor_wrapper import STLRuleMonitor
from commonroad_criticality.time_metrics.utils import update_ego_vehicle, visualize_state_list, int_round
from commonroad_criticality.time_metrics.simulation import (CutOffAction,
                                                            SimulationLateral,
                                                            SimulationLong,
                                                            check_elements_state_list)


class TC(CutOffBase, ABC):
    """
    Time-To-Compliance.
    """

    def __init__(self,
                 rule_monitor: STLRuleMonitor):
        super().__init__(rule_monitor._world_state)
        self.rule_monitor = rule_monitor
        self._tv_time_step = rule_monitor.tv_time_step
        self._other_id = rule_monitor.other_id
        self._visualize = False
        self._compliant_maneuver = None
        self._tc = -math.inf
        self._tc_dict = defaultdict(float)

        self._sim_lon = SimulationLong(None,
                                       self.ego_vehicle,
                                       None,
                                       dt=rule_monitor.world_state.dt)
        self._sim_lat = SimulationLateral(None,
                                          self.ego_vehicle,
                                          None,
                                          rule_monitor.world_state,
                                          dt=rule_monitor.world_state.dt)

    @property
    def simulation_lateral(self) -> Union[SimulationLateral]:
        return self._sim_lat

    @property
    def simulation_longitudinal(self) -> Union[SimulationLong]:
        return self._sim_lon

    @property
    def tv(self):
        return int_round(self._tv_time_step * self.dT, 1)

    @property
    def tc(self):
        if self._tc == -math.inf:
            return self._tc
        return int_round(self._tc, 1)

    @property
    def tc_time_step(self) -> Union[int, float]:
        if self._tc == -math.inf:
            return self._tc
        return int(self._tc / self.dT)

    @property
    def tv_time_step(self) -> Union[int, float]:
        return self._tv_time_step

    @property
    def compliant_maneuver(self) -> CutOffAction:
        return self._compliant_maneuver

    def calc_tv_updated(self, updated_states: List[State] = None) -> Tuple[float, Any]:
        # detect violation time using STL monitor
        # self.rule_monitor.evaluate_initially()
        self.rule_monitor._world_state.time_step = 0
        update_ego_vehicle(self.world_state.road_network,
                           self.world_state.ego_vehicle,
                           updated_states,
                           0,
                           self.dT)
        self.rule_monitor.evaluate_consecutively()
        evaluated_robustness, evaluated_ids = self.rule_monitor.query_rule_rob_all()
        if evaluated_robustness[0] < 0:
            return -math.inf, evaluated_ids[0][0]  # all violated
        tv = np.argmax(evaluated_robustness < 0)
        if tv == 0:
            return math.inf, None  # no violation
        if evaluated_ids[tv] is ():
            return tv * self.dT, self.ego_vehicle.obstacle_id
        return tv * self.dT, evaluated_ids[tv][0]

    def generate(self, cut_off_maneuvers: List[CutOffAction]):
        """
        Computes the Time-to-Compliance (with traffic rules).
        :param cut_off_maneuvers: the given maneuvers of ego vehicle
        :return: TC, corresponding maneuver
        """
        if not cut_off_maneuvers:
            return -math.inf
        if self.tv == -math.inf:
            raise ValueError("<TC>: the trajectory is not repairable since it already disobeys the rules")
        elif self.tv == math.inf:
            self._tc = math.inf
        else:
            ttm = dict()
            for maneuver in cut_off_maneuvers:
                if maneuver not in self._tc_dict:
                    ttm[maneuver] = self.search_ttm(maneuver)
                    self._tc_dict[maneuver] = ttm[maneuver]
                else:
                    ttm[maneuver] = self._tc_dict[maneuver]

            self._tc = max(ttm.values())
            self._compliant_maneuver = max(ttm, key=ttm.get)
        return self._tc

    @functools.lru_cache(128)
    def search_ttm(self, maneuver: CutOffAction):
        ttm = - math.inf
        low = 0
        high = int(int_round(self.tv / self.dT))
        while low < high:
            mid = int(int_round(low + high) / 2)
            if maneuver in [CutOffAction.BRAKE, CutOffAction.KICKDOWN, CutOffAction.STEADYSPEED]:
                self._sim_lon.update_action(maneuver, mid)
                state_list = self._sim_lon.simulate_state_list()
            elif maneuver in [CutOffAction.LANECHANGELEFT, CutOffAction.LANECHANGERIGHT]:
                self._sim_lat.update_action(maneuver, mid)
                state_list = self._sim_lat.simulate_state_list()
            else:
                raise ValueError("<TTCC>: given compliant maneuver {} is not supported".format(maneuver))
            if state_list is None:
                flag_collision = True
                tv = -math.inf
            else:
                if self._visualize:
                    visualize_state_list(self._collision_checker, state_list, self.scenario,
                                         self._sim_lat.vehicle_dynamics.shape)
                # flag_collision = self._detect_collision(state_list)  # bool value
                check_elements_state_list(state_list, self.dT)
                tv, _ = self.calc_tv_updated(state_list)  # which should be tv instead of ttm
            # if violation-free and collision-free
            if tv == math.inf:  # and not flag_collision:
                low = mid + 1
            else:
                high = mid

        if low != 0:
            ttm = (low - 1) * self.dT
        return ttm

