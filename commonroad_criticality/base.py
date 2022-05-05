# CommonRoad packages
from commonroad.scenario.scenario import Scenario
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration


class CriticalityBase:
    """Base class for criticality measures"""
    def __init__(self, scenario: Scenario, id_vehicle: int = None, config: CriticalityConfiguration = None,):
        assert isinstance(scenario, Scenario), '<Criticality>: Provided scenario is not valid!'
        assert isinstance(config, CriticalityConfiguration), '<Criticality>: Provided configuration is not valid!'
        # ==========     Scenario    =========
        self.scenario = scenario
        self.id_vehicle = id_vehicle

        # ==========  configuration  =========
        self.configuration = config


