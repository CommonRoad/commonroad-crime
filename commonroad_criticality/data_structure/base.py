from abc import abstractmethod
from typing import List, Union

# CommonRoad packages
from commonroad.scenario.scenario import Scenario
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration


class CriticalityBase:
    """Base class for criticality measures"""
    metric_name = "base"

    def __init__(self, config: CriticalityConfiguration):
        assert isinstance(config, CriticalityConfiguration), '<Criticality>: Provided configuration is not valid!'
        # assert isinstance(id_vehicles, list), '<Criticality>: Provided vehicle ids are not in a list!'

        # ==========     Scenario or scene   =========
        self.scenario = config.scenario
        self.scene = config.scene
        # if id_vehicles is None:
        #     self.id_vehicles = [veh.obstacle_id for veh in self.scenario.obstacles if
        #                         veh.state_at_time(time_step) is not None]
        # else:
        #     for v_id in id_vehicles:
        #         if self.scenario.obstacle_by_id(v_id) is None:
        #             assert f'<Criticality>: Vehicle (id: {v_id}) is not contained in the scenario!'
        #     self.id_vehicles = id_vehicles

        # ==========  configuration  =========
        self.configuration = config

    @abstractmethod
    def compute(self,  *args, **kwargs):
        pass
