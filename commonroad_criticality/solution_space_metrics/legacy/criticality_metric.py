import os
import time
from typing import Iterable, List, Union, Dict, Tuple

import numpy as np
import  pycrcc
import pycrreach
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.geometry.shape import ShapeGroup, Shape, Polygon
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_reach.initialization.initialization import create_lanelet_network
# from commonroad_reach.reach.common.reachability import ReachSetNode
from commonroad_reach.reach.common.util import area_of_reachable_set
# from commonroad_reach.reach.continuous.continuous_reach import ContinuousReachabilityAnalysis
from pycrreach import ContinuousReachabilityAnalysis
from pruningPy.graph_reach.graph_reach_veloctiy import GraphReachability4D

from pruningPy.graph_reach.online_graph_reach import ReachSetGridOnline

from testcaseopt.reachability.set_up import create_reachset_configuration_vehicle, generate_road_boundary_obstacles, \
    compute_initial_state, CPP, ReachabilitySingleVehicle
from testcaseopt.sequence_optimization.lanes import Lane
from testcaseopt.sequence_optimization.lane_manager import LaneManager


class ReachsetCriticalityMetric:
    """Class for evaluating criticality of a scenario using size of drivable area as a metric."""
    def __init__(self, length, width, settings, dt:float, t_f: int, use_curvilinear_coords, scenario_reference: Scenario=None,
                 reference_lane = None, lane_manager=None, generate_road_boundary=False,
                 graphbased_reachability_analysis=True, reachability_analysis:Union[ReachabilitySingleVehicle,ReachSetGridOnline]=None):
        self.dt = dt
        vehicle_settings = settings['vehicle_settings']
        self._border_scenario_path = settings['border_scenarios_path']
        self._scenario_reference = scenario_reference
        self._reference_lane = reference_lane
        self.t_f = t_f
        self.reachability_analysis = None
        self.curvilinear_coordinates = use_curvilinear_coords
        self.current_time_step = 0

        if use_curvilinear_coords is True:
            self._lanelet_network_vehicle = self._create_lanelet_network_vehicle(self._scenario_reference.lanelet_network,
                                                                                 reference_lane, lane_manager)
            center_line = reference_lane
        else:
            self._lanelet_network_vehicle = self._scenario_reference.lanelet_network
            center_line = None

        self.vehicle_configuration = create_reachset_configuration_vehicle(length, width, None,
                                                                           pycrcc.CollisionChecker(), None,
                                                                           settings['vehicle_settings'],
                                                                           reference_path=center_line,
                                                                           lanelet_network_vehicle=self._lanelet_network_vehicle,
                                                                           consider_traffic=
                                                                           settings['scenario_settings'][
                                                                               'consider_traffic'],
                                                                           draw=settings['scenario_settings']['draw'])

        # generate boundary nevertheless if file doesn't exist
        if generate_road_boundary is False:
            generate_road_boundary = not self._boundary_obstacle_file_exists(self._border_scenario_path, self._scenario_reference)

        if not generate_road_boundary:
            self._road_boundary_obstacles = self.load_road_border_obstacles_from_file(scenario_reference)
        else:
            # compute road boundary
            road_boundary_untransformed, self._road_boundary_obstacles, self.vehicle_configuration.collision_checker_curvilinear = generate_road_boundary_obstacles(
                self._scenario_reference,
                obstacles=[],
                road_boundaries=ShapeGroup([]),
                curvilinear_coordinate_system=self.vehicle_configuration.curvilinear_coordinate_system,
                generate_boundary_obs=generate_road_boundary,
                transform_obs_to_curvilinear=self.vehicle_configuration.collision_checks_in_curvilinear_cosy,#generate_road_boundary,
                lanelet_network_vehicle=self._lanelet_network_vehicle,
                vehicle_radius=self.vehicle_configuration.radius,
                reduce_distance_to_road_boundary=vehicle_settings['reduce_distance_to_road_boundary'],
                rasterize_obstacles=False)#vehicle_settings['rasterize_obstacles'])
            print('<ReachsetCriticalityMetric/init> WRITE BOUNDARY OBSTACLE FILE.')
            self.save_road_boundary_scenario(self._scenario_reference, self._road_boundary_obstacles,
                                             self._border_scenario_path)

        self.reference_area_profile = None
        # reachability object of latest criticality computation
        self.reachability_analysis: Union[ReachabilitySingleVehicle,ReachSetGridOnline] = reachability_analysis
        self.refined_reachable_set = {}
        self.reachable_set_reference = {}
        self.graphbased_reachability_analysis: bool = graphbased_reachability_analysis

    def _boundary_obstacle_file_exists(self, folder_path:str, scenario:Scenario):
        return os.path.isfile(os.path.join(folder_path, scenario.benchmark_id + '.xml'))

    def save_road_boundary_scenario(self, scenario, boundary_obstacles:Union[ShapeGroup, List[Shape]], folder_path:str):
        """Saves scenario_reference to file."""
        if not os.path.exists(self._border_scenario_path):
            os.makedirs(self._border_scenario_path)

        scenario_save = Scenario(scenario.dt, scenario.benchmark_id)
        scenario_save.add_objects(scenario.lanelet_network)
        # if type(boundary_obstacles) == ShapeGroup:
        for shape in boundary_obstacles.shapes:
            scenario_save.add_objects(StaticObstacle(scenario_save.generate_object_id(),ObstacleType.ROAD_BOUNDARY,shape,initial_state=State(position=np.array([0.0,0.0]), orientation=0.0,time_step=0)))

        fw = CommonRoadFileWriter(scenario_save, None, affiliation='TUM', author='Moritz Klischat',
                                  source='created with border tool from A.Zhu', tags=())
        filename = os.path.join(folder_path, scenario.benchmark_id + '.xml')
        fw.write_scenario_to_file(filename, overwrite_existing_file=OverwriteExistingFile.ALWAYS)

    def load_road_border_obstacles_from_file(self, scenario:Scenario) -> ShapeGroup:
        boundary_scenario, _ = CommonRoadFileReader(os.path.join(self._border_scenario_path, scenario.benchmark_id + '.xml')).open()
        return ShapeGroup([obs.obstacle_shape for obs in boundary_scenario.static_obstacles])

    def compute_reachable_area_profile(self, reachable_set: Dict[int, List[pycrreach.ReachSetNode]]) -> np.ndarray:
        """Computes area profile for given reachability analysis."""
        area_profile = []
        for t, reachset_t in reachable_set.items():
            area_profile.append(area_of_reachable_set(reachset_t))

        return np.array(area_profile)

    def _create_lanelet_network_vehicle(self, lanelet_network, reference_lane:Lane, lane_manager: LaneManager):
        assert isinstance(reference_lane.lane_id, Iterable)
        lanelets_leading_to_goal = set()
        for lane_id in reference_lane.lane_id:
            for lane in lane_manager.lane_section_network.sections[lane_id.section].lanes:
                lanelets_leading_to_goal.update(lane.lanelet_ids)

        return create_lanelet_network(lanelet_network, list(lanelets_leading_to_goal))

    def init_reachability_analysis(self, scenario:Scenario, initial_state, graph_based_path:str=None):
        self.current_time_step = 0
        if self.graphbased_reachability_analysis is True and self.reachability_analysis is None:
            if 'json' in graph_based_path:
                self.reachability_analysis = ReachSetGridOnline.init_from_file(graph_based_path)
            else:
                self.reachability_analysis = GraphReachability4D.init_from_file(graph_based_path)

        parameters_online = {'initial_state': initial_state,
                             'sensitivity': False, 'dynamic_only': False,
                             'collision_checker_params': {}, 'sparse': True}

        _, _, cc = generate_road_boundary_obstacles(
            self._scenario_reference, scenario.dynamic_obstacles,
            self._road_boundary_obstacles,
            self.vehicle_configuration.curvilinear_coordinate_system,
            generate_boundary_obs=False,
            transform_obs_to_curvilinear=self.vehicle_configuration.collision_checks_in_curvilinear_cosy,
            lanelet_network_vehicle=self._lanelet_network_vehicle,
            vehicle_radius=self.vehicle_configuration.radius,
            rasterize_obstacles=self.vehicle_configuration.rasterized_obstacles)

        if self.curvilinear_coordinates:
            self.vehicle_configuration._collision_checker_curvilinear = cc
        else:
            self.vehicle_configuration._collision_checker_world = cc
            self.vehicle_configuration._collision_checker_curvilinear = cc

        if self.graphbased_reachability_analysis is True:
            self.reachability_analysis.init_scenario(self.vehicle_configuration.collision_checker_curvilinear, scenario.benchmark_id,
                                                     parameters_online)
        else:
            self.vehicle_configuration.initial_state = compute_initial_state(initial_state,
                                                                             self.vehicle_configuration.curvilinear_coordinate_system,
                                                                             self._reference_lane,
                                                                             self.curvilinear_coordinates)
            self.vehicle_configuration.initial_time_idx = initial_state.time_step
            # self.vehicle_configuration.collision_checker_curvilinear = collision.CollisionChecker()
            self.vehicle_configuration.coordinates = 'cartesian'
            self.vehicle_configuration.radius = 0.0
            self.vehicle_configuration.width = 0.0
            self.vehicle_configuration.length = 0.0
            self.vehicle_configuration.wheelbase = 0.0

            if CPP is True:
                reachability_analysis = pycrreach.ContinuousReachabilityAnalysis(
                    self.dt, self.vehicle_configuration.convert_to_pycrreach_vehicle_parameters())

                self.reachability_analysis = ReachabilitySingleVehicle(reachability_analysis,
                                                                        self.vehicle_configuration)
            else:
                reachability_analysis = ContinuousReachabilityAnalysis(self.dt, self.vehicle_configuration)
                self.reachability_analysis = ReachabilitySingleVehicle(reachability_analysis)

            # self.vehicle_configuration.radius = radius_buffer

        return self.vehicle_configuration

    def compute_reachable_area(self, time_steps) -> Tuple[ReachabilitySingleVehicle, Dict[int, List[
        pycrreach.ReachSetNode]]]:
        """Returns reachability object and computes reachable area."""
        self.reachability_analysis.compute_next_time_steps(self.current_time_step + 1,
                                                            self.current_time_step + time_steps)
        self.current_time_step += time_steps
        # reachability_single_vehicle.eliminate_dead_ends_of_reachable_set()
        return self.reachability_analysis, self.reachability_analysis.drivable_area

    def compute_reachable_area_graphbased(self, time_end, backward=False) -> Union[ReachSetGridOnline, GraphReachability4D]:
        """Returns reachability object and computes reachable area."""
        assert type(self.reachability_analysis) in (ReachSetGridOnline, GraphReachability4D)
        # if obstacles is not None:
        #     self.init_reachability_analysis(obstacles, initial_state, graph_based_path=graph_based_path)

        self.reachability_analysis.forward_steps(time_end)
        if backward:
            self.reachability_analysis.backward_forward(8)
        return self.reachability_analysis

    def _criticality_metric(self, area_profile, reference_area_profile):
        assert np.size(area_profile) <= np.size(self.reference_area_profile)
        if any(np.isnan(area_profile)):
            return np.inf

        return np.mean(np.divide(area_profile, reference_area_profile[:np.size(area_profile)]))

    def criticality(self, initial_state: State, obstacles: List[DynamicObstacle]):
        """Computes critically using size of drivable area constrained by obstacles compared to
        unconstrained drivable area."""

        # Reference (without dynamic obstacles)
        # TODO: no reference implemented currently
        _, self.reachable_set_reference = self.compute_reachable_area(initial_state, time_end=self.t_f)
        self.reference_area_profile = self.compute_reachable_area_profile(self.reachable_set_reference)

        # Drivable area
        self.reachability_analysis, self.refined_reachable_set = self.compute_reachable_area(initial_state, self.t_f)
        area_profile = self.compute_reachable_area_profile(self.refined_reachable_set)
        return self._criticality_metric(area_profile, self.reference_area_profile)