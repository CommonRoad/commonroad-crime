import functools
import math
from typing import Iterable, Union, Tuple, Any, List
from enum import Enum
import numpy as np

from stl_crmonitor.crmonitor.evaluation.evaluation import RuleSetEvaluator
from stl_crmonitor.crmonitor.common.world_state import WorldState
from stl_crmonitor.crmonitor.predicates.rule import PropositionNode

# CommonRoad Toolbox
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem


class MonitorType(Enum):
    """
    Type of temporal logic used in the traffic rule monitor
    """
    MTL = "metric temporal logic"
    STL = "signal temporal logic"


class STLRuleMonitor:
    def __init__(self,
                 scenario: Scenario,
                 planning_problem: PlanningProblem,
                 vehicle_id: int,
                 rules: Union[str, Iterable[str]], ):
        self._world_state: WorldState = self.construct_world_state(scenario,
                                                                   planning_problem,
                                                                   vehicle_id)
        self._vehicle_id = vehicle_id
        self._rules = rules
        self._rule_eval = RuleSetEvaluator.create_from_config(rules,
                                                              dt=self._world_state.dt)
        self.rob_rule, self.rob_predicate, self.rob_abstraction = self.evaluate_initially()
        # obtain the time-to-violation
        self._tv, self._other_id = self._cal_tv_initial()
        self._prop_nodes = self._initialize_prop_rob()
        print("# =========== Traffic Rule Monitor ========== #")
        print("\tthe ego vehicle (id: {})'s initial\n\ttrajectory violates traffic rule {}".
              format(self._vehicle_id, self._rules))
        print('\tw.r.t vehicle {} at time step {}.'.format(self.other_id, self.tv_time_step))
        print("# =========================================== #")

    @property
    def tv_time_step(self) -> Union[int, float]:
        return self._tv

    @property
    def other_id(self) -> int:
        if self._other_id is None or not isinstance(self._other_id, int):
            return self._vehicle_id
        else:
            return self._other_id

    @property
    def vehicle_id(self) -> int:
        return self._vehicle_id

    @property
    def world_state(self) -> WorldState:
        return self._world_state

    @property
    def type(self):
        return MonitorType.STL

    @property
    def rule_eval(self):
        return self._rule_eval

    @property
    def proposition_nodes(self) -> List[PropositionNode]:
        return self._prop_nodes

    @property
    def sat_formula(self):
        return self._rule_eval.sat_formula

    @property
    @functools.lru_cache(128)
    def prop_robust_all(self):
        return self.rob_abstraction.query('other_id == @self._other_id')

    @property
    @functools.lru_cache(128)
    def prop_robust_ttv(self):
        return self.prop_robust_all.query('time_step == @self._tv')

    @staticmethod
    def construct_world_state(scenario: Scenario,
                              planning_problem: PlanningProblem,
                              ego_id: int) -> WorldState:
        """
        Constructs world state
        """
        world_state = WorldState.create_from_scenario(scenario,
                                                      ego_id,
                                                      planning_problem=planning_problem)
        return world_state

    def _initialize_prop_rob(self):
        # obtain the id of violation-relevant vehicle
        prop_nodes = self._rule_eval.proposition_nodes
        # assign the robustness at ttv
        if self._tv in (math.inf, -math.inf):
            return None
        for node in prop_nodes:
            node.ttv_value = self.prop_robust_ttv.query('alphabet == @node.alphabet')["robustness"].values[0]
        return prop_nodes

    def evaluate_initially(self):
        """
        Evaluate whether the ego vehicle disobeys traffic rules
        """
        return self._rule_eval. \
            evaluate_incremental(self._world_state,
                                 to_pandas=True)

    def evaluate_consecutively(self):
        """
        Evaluate the updated vehicle states (boolean assignments) in order to speed up the evaluation progress
        """
        self._rule_eval.switch_to_boolean()
        self.rob_rule, self.rob_predicate = self._rule_eval. \
            evaluate_consecutively(self._world_state,
                                   )

    def query_rule_rob_all(self):
        """
        Queries the robustness value and the other vehicle id with the minimum robustness
        """
        if self.rob_rule is None:
            raise ValueError("the evaluation procedure is not executed yet")
        return self.rob_rule['robustness'].values, self.rob_rule["other_ids"].values

    def _cal_tv_initial(self) -> Tuple[Union[int, float], Any]:
        # calculate the time-to-violation: detect violation time using STL monitor
        evaluated_robustness, evaluated_ids = self.query_rule_rob_all()
        if evaluated_robustness[0] < 0:
            if evaluated_ids[0] is ():
                return -math.inf, None
            return -math.inf, evaluated_ids[0][0]  # all violated
        tv = np.argmax(evaluated_robustness < 0)
        if tv == 0:
            return math.inf, None  # no violation
        if evaluated_ids[tv] is () or self._rules == 'R_G2':  # R_G2: we focus on the ego vehicle
            return int(tv), self._world_state.ego_vehicle.id
        return int(tv), evaluated_ids[tv][0]

# Currently, MTL monitor is not supported
# class MTLRuleMonitor:
#     def __init__(self,
#                  scenario: Scenario,
#                  ego_id: int,
#                  rule_set: Union[str, Iterable[str]]):
#         self.rule_eval = CommonRoadObstacleEvaluation(os.path.dirname(__file__) + "/../../config/")
#         self.rule_eval.activated_traffic_rule_sets = rule_set
#         assert self.rule_eval.simulation_param["evaluation_mode"] == "test", "<MTLRuleMonitor>: the given evaluation " \
#                                                                              "mode {} is invalid".\
#             format(self.rule_eval.simulation_param["evaluation_mode"])
#         self.rule_eval.update_eval_dict()
#         self._scenario = scenario
#         self._ego_id = ego_id
#
#     def evaluate_initially(self):
#         """
#         Evaluate the rule violation initially - if violated, return the corresponding rule-relevant vehicle (if existed)
#         """
#         eval_result = self.rule_eval.evaluate_scenario(self._scenario)
#         ego_result = None
#         for veh_id, evaluation in eval_result:
#             if veh_id == self._ego_id:
#                 ego_result = evaluation
#                 break
#         violation_boolean = False
#         violation_veh = list()
#         for rule_str, result in ego_result.items():
#             if not result:
#                 violation_veh.append(int(rule_str[-4:]))
#                 violation_boolean = True
#         return violation_boolean, violation_veh
