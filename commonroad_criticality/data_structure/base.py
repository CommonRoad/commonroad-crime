import math
from abc import abstractmethod
import copy
from typing import Union

# CommonRoad packages
from commonroad.scenario.obstacle import Obstacle, DynamicObstacle
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
import commonroad_criticality.utility.visualization as utils_vis
import commonroad_criticality.utility.general as utils_gen

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem


class CriticalityBase:
    """Base class for criticality measures"""
    metric_name = "base"

    def __init__(self, config: CriticalityConfiguration):
        assert isinstance(config, CriticalityConfiguration), '<Criticality>: Provided configuration is not valid!'

        self.value = None
        self.time_step = 0
        # ==========  configuration  =========
        self.configuration = config
        # =======  Scenario or scene  ========
        if config.scenario:
            self.sce = copy.deepcopy(config.scenario)
        else:
            self.sce = copy.deepcopy(config.scene)
        if self.sce is None:
            assert "<Criticality>: the scenario/scene in the configuration needs to be first updated"
        self.dt = self.sce.dt
        # =======       Vehicles      ========
        self.ego_vehicle: DynamicObstacle = self.sce.obstacle_by_id(self.configuration.vehicle.ego_id)
        self.other_vehicle: Union[Obstacle, None] = None  # optional
        self.clcs: CurvilinearCoordinateSystem = self.update_clcs()
        self.rnd: Union[MPRenderer, None] = None

    def update_clcs(self):
        # default setting of ego vehicle's curvilinear coordinate system
        ego_initial_lanelet_id = list(self.ego_vehicle.prediction.center_lanelet_assignment[0])[0]
        reference_path = utils_gen.generate_reference_path(ego_initial_lanelet_id, self.sce.lanelet_network)
        clcs = CurvilinearCoordinateSystem(reference_path)
        self.configuration.update(CLCS=clcs)
        return clcs

    def initialize_vis(self,
                       figsize: tuple = (25, 15), plot_limit: Union[list, None] = None):
        self.rnd = MPRenderer(figsize=figsize, plot_limits=plot_limit)
        utils_vis.draw_sce_at_time_step(self.rnd, self.configuration, self.sce, self.time_step)

    def set_other_vehicles(self, vehicle_id: int):
        """
        Sets up the id for other metric-related vehicle.
        """
        if not self.sce.obstacle_by_id(vehicle_id):
            raise ValueError(f"<Criticality>: Vehicle (id: {vehicle_id}) is not contained in the scenario!")
        self.other_vehicle = self.sce.obstacle_by_id(vehicle_id)

    @abstractmethod
    def compute(self,  *args, **kwargs):
        pass

    @abstractmethod
    def visualize(self):
        pass
