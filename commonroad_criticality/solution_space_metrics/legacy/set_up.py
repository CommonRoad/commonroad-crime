import itertools
from typing import Dict, List, Tuple, Union
import warnings
import numpy as np
import commonroad_dc.pycrcc as pycrcc

from commonroad.geometry.shape import ShapeGroup, Polygon, Shape
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle, StaticObstacle

import commonroad_dc.collision.collision_detection.minkowski_sum as crcc_minkowski
import commonroad_dc.boundary.construction as construction

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object

# from commonroad_reach.geometry.polyline import calculate_orientation_from_polyline, compute_pathlength_from_polyline
# from commonroad_reach.initialization.initialization import create_curvilinear_coordinate_system
# from commonroad_reach.initialization.initialization_edmond import create_curvilinear_coordinate_system
from commonroad_reach.initialization.initialization import read_lut_longitudinal_enlargement
from commonroad_reach.initialization.initialize_collision_checker import create_curvilinear_collision_checker

# try:
import pycrccosy as pycrccosy
# from commonroad_dc.geometry import \
#     convert_shape_to_curvilinear_coords, \
#     convert_list_of_polygons_to_curvilinear_coordinates, \
#     convert_list_of_polygons_to_curvilinear_coordinates_and_rasterize, convert_shape_to_polygon_list
# except ImportError:
#     warnings.warn('pycrccosy module not found, installed commonroad-curvilinear-coordinate-system correctly?')

try:
    import pycrreach
except ImportError:
    warnings.warn('pycrreach module not found')

from commonroad_reach.reach.common.configuration import ReachSetConfigurationVehicle

from testcaseopt.common.mat import compute_curvature_from_polyline, compute_pathlength_from_polyline, \
    calculate_orientation_from_polyline
from testcaseopt.common.utils import dilate_vehicle_shape
from testcaseopt.sequence_optimization.lanes import Lane

# from commonroad_ccosy.geometry.trapezoid_coordinate_system import convert_shape_to_curvilinear_coords
# from pycrccosy import CurvilinearCoordinateSystem

# from commonroad_ccosy.geometry.trapezoid_coordinate_system import convert_shape_to_curvilinear_coords
from pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.boundary import boundary

CPP = True

if CPP == True:
    try:
        from commonroad_reach.reach.reachability_single_vehicle_cpp import ReachabilitySingleVehicle
    except ImportError:
       ValueError('Package commonroad-reachability-fork does not exist, set CPP=False!')
else:
    from commonroad_reach.reach.reachability_single_vehicle import ReachabilitySingleVehicle


def set_up(length, width, settings: Dict, dt, cc, initial_state: State, reference_lane: Lane,
           lanelet_network_vehicle: LaneletNetwork) -> Tuple[ReachabilitySingleVehicle, ReachSetConfigurationVehicle]:

    vehicle_configuration = create_reachset_configuration_vehicle(length, width, None, cc, initial_state,
                                                                  settings['vehicle_settings'],
                                                                  reference_path=reference_lane.center_line,
                                                                  lanelet_network_vehicle=lanelet_network_vehicle,
                                                                  consider_traffic=settings['scenario_settings'][
                                                                      'consider_traffic'],
                                                                  draw=settings['scenario_settings']['draw'])


    # Set up reachability analysis
    if CPP is True:
        reachability_analysis = pycrreach.ContinuousReachabilityAnalysis(
            dt, vehicle_configuration.convert_to_pycrreach_vehicle_parameters())
        reachability_single_vehicle = ReachabilitySingleVehicle(reachability_analysis, vehicle_configuration)
    else:
        reachability_analysis = pycrreach.ContinuousReachabilityAnalysis(dt, vehicle_configuration)
        reachability_single_vehicle = ReachabilitySingleVehicle(reachability_analysis, vehicle_configuration)

    # return configuration, vehicle_configuration
    return reachability_single_vehicle, vehicle_configuration


def create_curvilinear_coordinate_system(reference_path: np.ndarray) -> pycrccosy.CurvilinearCoordinateSystem:
    cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path, 25, 0.1)
    curvature_of_reference_path = np.zeros(len(cosy.segments_longitudinal_coordinates()))
    cosy.set_curvature(curvature_of_reference_path)
    return cosy

def create_reachset_configuration_vehicle(
        length, width,
        scenario: Scenario,
        cc: pycrcc.CollisionChecker,
        initial_state: State,
        vehicle_settings: Dict,
        reference_path: np.ndarray = None,
        lanelet_network_vehicle: LaneletNetwork=None,
        consider_traffic: bool = True, draw: bool = True):

    configuration = ReachSetConfigurationVehicle()

    configuration.lanelet_network = lanelet_network_vehicle
    if reference_path is not None:
        configuration.reference_path = reference_path

    configuration.vehicle_id = -1
    configuration.min_speed_x = vehicle_settings['min_speed_x']
    configuration.max_speed_x = vehicle_settings['max_speed_x']
    configuration.min_speed_y = vehicle_settings['min_speed_y']
    configuration.max_speed_y = vehicle_settings['max_speed_y']

    configuration.a_max_x = vehicle_settings['a_max_x']
    configuration.a_min_x = vehicle_settings['a_min_x']

    configuration.a_max_y = vehicle_settings['a_max_y']
    configuration.a_min_y = vehicle_settings['a_min_y']

    configuration.a_max = vehicle_settings['a_max']

    configuration.j_min_x = vehicle_settings['j_min_x']
    configuration.j_max_x = vehicle_settings['j_max_x']
    configuration.j_min_y = vehicle_settings['j_min_y']
    configuration.j_max_y = vehicle_settings['j_max_y']

    configuration.grid_x = vehicle_settings['grid_x']
    configuration.grid_y = vehicle_settings['grid_y']
    configuration.max_radius = vehicle_settings['max_radius']

    configuration.initial_time_idx = initial_state.time_step if initial_state is not None else 0  # vehicle_settings['initial_time_idx']

    configuration.length = length
    configuration.width = width
    radius, distance_radius = compute_approximating_circle_radius(configuration.length,
                                                                  configuration.width)
    configuration.radius = radius
    configuration.wheelbase = 0.7 * length
    configuration.lut_lon_enlargement = read_lut_longitudinal_enlargement(configuration,
                                                                          vehicle_settings['path_to_lut'])
    configuration.distance_circle_centers = distance_radius * 2 # CHANGED
    configuration.consider_traffic = consider_traffic
    # configuration.desired_speed = vehicle_settings['desired_speed']

    if 'position_uncertainty_x' in vehicle_settings.keys():
        configuration.position_uncertainty_x = vehicle_settings['position_uncertainty_x']
    if 'position_uncertainty_y' in vehicle_settings.keys():
        configuration.position_uncertainty_y = vehicle_settings['position_uncertainty_y']

    if reference_path is not None:
        configuration.curvilinear_coordinate_system = create_curvilinear_coordinate_system(
            configuration.reference_path)
        curvature_of_reference_path = compute_curvature_from_polyline(reference_path)
        # configuration.curvilinear_coordinate_system.set_maximum_curvature_radius(
        #     1 / abs(max(curvature_of_reference_path)))
        # configuration.curvilinear_coordinate_system.set_minimum_curvature_radius(
        #     1 / abs(min(curvature_of_reference_path)))

    if initial_state is not None:
        configuration.initial_state = compute_initial_state(initial_state, configuration.curvilinear_coordinate_system,
                                                            reference_path)
    else:
        configuration.initial_state = (())

    configuration.collision_checks_in_curvilinear_cosy = vehicle_settings['collision_checks_in_curvilinear_cosy']
    configuration.rasterized_obstacles = vehicle_settings['rasterize_obstacles']
    if vehicle_settings['collision_checks_in_curvilinear_cosy']:
        # configuration.collision_checker_curvilinear = cc
        configuration.collision_checker_curvilinear = create_curvilinear_collision_checker(
            scenario, configuration.lanelet_network, configuration.radius, configuration.curvilinear_coordinate_system,
            consider_traffic, vehicle_settings['rasterize_obstacles'],
            vehicle_settings['reduce_distance_to_road_boundary'])
    else:
        configuration.collision_checker_world = cc
            # create_cartesian_collision_checker(
            # scenario, configuration.lanelet_network, configuration.radius, consider_traffic,
            # vehicle_settings['reduce_distance_to_road_boundary'])

    return configuration


def compute_initial_state(
        initial_state: State, curvilinear_coordinate_system: CurvilinearCoordinateSystem,
        reference_path: Union[Lane,np.ndarray], curvilinear=True) -> Tuple:
    # import matplotlib.pyplot as plt
    # from commonroad_dc.collision.visualization.draw_dispatch import draw_object
    # plt.figure(figsize=(25, 10))
    # draw_object(initial_state)
    # domain = curvilinear_coordinate_system.get_projection_domain()
    # draw_object(Polygon(np.array(domain)))
    # plt.gca().set_aspect('equal')
    # plt.autoscale()
    # plt.show()
    # plt.pause(0.01)
    if curvilinear:
        if not isinstance(reference_path, np.ndarray):
            reference_path = reference_path.center_line

        pos = curvilinear_coordinate_system.convert_to_curvilinear_coords(
            initial_state.position[0],
            initial_state.position[1])

        ref_orientation = calculate_orientation_from_polyline(reference_path)
        ref_pathlength = compute_pathlength_from_polyline(reference_path)
        orientation_interpolated = np.interp(pos[0], ref_pathlength, ref_orientation)
    else:
        orientation_interpolated = 0.0
        pos = initial_state.position

    v_x = initial_state.velocity * np.cos(
        initial_state.orientation - orientation_interpolated)
    # print(v_x)
    v_y = initial_state.velocity * np.sin(
        initial_state.orientation - orientation_interpolated)

    return ((pos[0], v_x),
            (pos[1], v_y))


def generate_road_boundary_obstacles(scenario_reference: Scenario, obstacles: List[DynamicObstacle],
                                     road_boundaries: ShapeGroup,
                                     curvilinear_coordinate_system: CurvilinearCoordinateSystem,
                                     generate_boundary_obs=True,
                                     transform_obs_to_curvilinear=True,
                                     lanelet_network_vehicle = None,
                                     vehicle_radius=1.0,
                                     reduce_distance_to_road_boundary=0.0,
                                     rasterize_obstacles=False):

    if generate_boundary_obs:
        if lanelet_network_vehicle is None:
            ValueError('Specify lanelet_network_vehicle if generate_road_boundary=True')

        scenario_cc: Scenario = Scenario(scenario_reference.dt, scenario_id=scenario_reference.scenario_id)
        scenario_cc.add_objects(lanelet_network_vehicle)
        # static_bound = boundary.create_road_boundary_aligned_triangles(scenario_cc, axis=1)
        # scenario_cc.add_objects(static_bound)
        scenario_cc.add_objects(create_road_boundary(scenario_cc))
        static_polygon_list = list()
        for static in scenario_cc.static_obstacles:
            # print("static obstacle id: {}".format(static.obstacle_id))
            initial_time_step = static.initial_state.time_step
            occ = static.occupancy_at_time(initial_time_step)
            if static.obstacle_type == ObstacleType.ROAD_BOUNDARY:
                input_shape = crcc_minkowski.minkowski_sum_circle(
                    occ.shape, vehicle_radius - reduce_distance_to_road_boundary, resolution=0)
            else:
                input_shape = crcc_minkowski.minkowski_sum_circle(occ.shape, vehicle_radius, resolution=0)
            if isinstance(input_shape, ShapeGroup):
                static_polygon_list.extend(input_shape.shapes)
            else:
                static_polygon_list.append(input_shape)
            # static_polygon_list = ShapeGroup(static_polygon_list)
    else:
        static_polygon_list = road_boundaries

    # import matplotlib.pyplot as plt
    # from commonroad_dc.collision.visualization.draw_dispatch import draw_object
    # plt.figure(figsize=(25, 10))
    # draw_object(static_polygon_list)
    # plt.gca().set_aspect('equal')
    # plt.autoscale()
    # plt.draw()
    # plt.pause(0.01)

    if generate_boundary_obs and transform_obs_to_curvilinear is True:
        road_boundary_obs = convert_shapelist_to_curvilinear_coords(static_polygon_list, rasterize_obstacles, curvilinear_coordinate_system)
    else:
        if isinstance(static_polygon_list, ShapeGroup):
            road_boundary_obs = static_polygon_list
        else:
            road_boundary_obs = ShapeGroup(static_polygon_list)

    # road_boundary_obs = []
    # from commonroad_dc.collision.visualization.draw_dispatch import draw_object
    # plt.figure(figsize=(25, 10))
    # draw_object(road_boundary_obs)
    # plt.gca().set_aspect('equal')
    # plt.autoscale()
    # plt.draw()
    # plt.pause(0.01)

    cc = pycrcc.CollisionChecker()
    # static obstacles
    for element in road_boundary_obs.shapes:
        cc.add_collision_object(create_collision_object(element))

    # dynamic obstacles
    dilation_params = {'minkowski_sum_circle_radius':vehicle_radius, 'long_margin':1.5*vehicle_radius}
    for obstacle in obstacles:
        # print("obstacle id: {}".format(obstacle.obstacle_id))
        tvo = None
        initial_time_step = obstacle.initial_state.time_step
        # add occupancy of initial state


        if transform_obs_to_curvilinear:
            input_shape = transform_polygon_to_curvilinear(input_shape, rasterize_obstacles, curvilinear_coordinate_system)

        # add occupancies of prediction
        if hasattr(obstacle,'prediction'):
            if not isinstance(obstacle.prediction, TrajectoryPrediction):
                input_shape = dilate_vehicle_shape(dilation_params, obstacle,
                                                   obstacle.prediction.occupancy_at_time_step(initial_time_step),
                                                   obstacle.obstacle_role, vehicle_radius)
                if input_shape is not None:
                    tvo = pycrcc.TimeVariantCollisionObject(initial_time_step)
                    tvo.append_obstacle(create_collision_object(input_shape))

                for occ in obstacle.prediction.occupancy_set:
                    # print("prediction time step: {}".format(occ.time_step))
                    # if occupancy set is empty from spot, the prediction is finished
                    if isinstance(occ.shape, ShapeGroup) and len(occ.shape.shapes) == 0:
                        break
                    else:
                        input_shape = crcc_minkowski.minkowski_sum_circle(occ.shape, vehicle_radius, resolution=5)
                        if transform_obs_to_curvilinear:
                            input_shape = transform_polygon_to_curvilinear(input_shape, rasterize_obstacles, curvilinear_coordinate_system)

                        # if transformed_occ is None:
                        #     transformed_occ: ShapeGroup = transform_polygon_to_curvilinear(occ.shape, rasterize_obstacles,
                        #                                                                    curvilinear_coordinate_system)
                        #     if transformed_occ is None:
                        #         import matplotlib.pyplot as plt
                        #         from commonroad_dc.collision.visualization.draw_dispatch import draw_object
                        #         plt.figure(figsize=(25, 10))
                        #         draw_object(occ.shape)
                        #         domain = curvilinear_coordinate_system.get_projection_domain()
                        #         draw_object(Polygon(np.array(domain)))
                        #         plt.gca().set_aspect('equal')
                        #         plt.autoscale()
                        #         plt.show()
                        #         plt.pause(0.01)
                        #         raise ValueError('could not transform')

                    if input_shape is not None:
                        if tvo is None:
                            tvo = pycrcc.TimeVariantCollisionObject(occ.time_step)
                        tvo.append_obstacle(create_collision_object(input_shape))
                    elif input_shape is None and tvo is not None:
                        break
            else:
                obstacle._obstacle_shape = dilate_vehicle_shape(dilation_params, obstacle,
                                                   obstacle._obstacle_shape,
                                                   obstacle.obstacle_role, vehicle_radius)
                obstacle._prediction._shape = obstacle._obstacle_shape
                obstacle._prediction._occupancy_set = obstacle._prediction._create_occupancy_set()
                tvo = create_collision_object(obstacle)

            if tvo is not None:
                cc.add_collision_object(tvo)

    return static_polygon_list, road_boundary_obs, cc


def transform_polygon_to_curvilinear(polygon: Shape, rasterize: bool,
                                     curvilinear_coordinate_system: CurvilinearCoordinateSystem) -> ShapeGroup:
    """Transforms polygon to Shapegroup of AAR of polygon in curvilinear coordinates"""
    out_shape = convert_shapelist_to_curvilinear_coords([polygon], rasterize, curvilinear_coordinate_system, )
    # if out_shape is None:
    #     raise ValueError('could not convert shapes!')

    return out_shape


def convert_shapelist_to_curvilinear_coords(shape_list: List[Shape], rasterize: bool,
                                     curvilinear_coordinate_system: CurvilinearCoordinateSystem) -> ShapeGroup:

    """
    Converts a shape from CommonRoad to curvilinear coordinates.

    :param curvilinear_coordinate_system: trapezoid coordinate system
    :param shape: shape to be transformed, e.g., rectangle or polygon
    :param rasterize: if true, the transformed shape is overapproximated with axis-aligned rectangles
    :return: None, if transformation was not successful (e.g. shape is outside of projection domain); otherwise,
    the successfully transformed shape is returned.
    """
    polygon_list_list = [convert_shape_to_polygon_list(shape) for shape in shape_list]
    polygon_list = list(itertools.chain.from_iterable(polygon_list_list))

    if rasterize:
        success, transformed_shape = convert_list_of_polygons_to_curvilinear_coordinates_and_rasterize(
            curvilinear_coordinate_system, polygon_list)
    else:
        success, transformed_shape = convert_list_of_polygons_to_curvilinear_coordinates(
            curvilinear_coordinate_system, polygon_list)
    if success:
        return ShapeGroup(transformed_shape)
    else:
        raise ValueError('could not convert shapes!')

# copied from commonroad reachability
# def create_curvilinear_coordinate_system(reference_path: np.ndarray) -> pycrccosy.CurvilinearCoordinateSystem:
#     cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path, 25, 0.1)
#     curvature_of_reference_path = compute_curvature_from_polyline(reference_path)
#     cosy.set_curvature(curvature_of_reference_path)
#     return cosy


def compute_approximating_circle_radius(ego_length, ego_width) -> Tuple[float, float]:
    """
    From Julia Kabalar
    Computes parameters of the circle approximation of the ego_vehicle

    :param ego_length: Length of ego vehicle
    :param ego_width: Width of ego vehicle
    :return: radius of circle approximation, circle center point distance
    """
    assert ego_length >= 0 and ego_width >= 0, 'Invalid vehicle dimensions = {}'.format([ego_length, ego_width])

    if np.isclose(ego_length, 0.0) and np.isclose(ego_width, 0.0):
        return 0.0, 0.0

    # Divide rectangle into 3 smaller rectangles
    square_length = ego_length / 3

    # Calculate minimum radius
    diagonal_square = np.sqrt((square_length / 2) ** 2 + (ego_width / 2) ** 2)

    # Round up value
    if diagonal_square > round(diagonal_square, 1):
        approx_radius = round(diagonal_square, 1) + 0.1
    else:
        approx_radius = round(diagonal_square, 1)

    return approx_radius, round(square_length * 2, 1)


def create_road_boundary(scenario: Scenario, draw=False) -> StaticObstacle:
    return create_road_boundary_obstacle(scenario, method="obb_rectangles", return_scenario_obstacle=True)[0]

    method = 'triangulation'
    build = ['section_triangles', ('triangulation', {'max_area': '1'})]
    if draw:
        draw = ['triangulation']
        boundary = construction.construct(scenario, build, draw)
    else:
        boundary = construction.construct(scenario, build, [])

    road_boundary_shape_list = list()
    for r in boundary[method].unpack():
        p = Polygon(np.array(r.vertices()))
        road_boundary_shape_list.append(p)
    initial_state = State(position=np.array([0, 0]), orientation=0.0, time_step=0)
    road_boundary = StaticObstacle(obstacle_id=scenario.generate_object_id(), obstacle_type=ObstacleType.ROAD_BOUNDARY,
                                   obstacle_shape=ShapeGroup(road_boundary_shape_list), initial_state=initial_state)

    return road_boundary
