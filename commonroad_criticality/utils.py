import functools
import math

from stl_crmonitor.crmonitor.common.vehicle import Vehicle
from stl_crmonitor.crmonitor.common.road_network import RoadNetwork
from stl_crmonitor.crmonitor.common.helper import (_compute_jerk,
                                                   update_curvilinear_states_long,
                                                   create_curvilinear_states
                                                   )
from typing import List
from vehiclemodels.parameters_vehicle1 import VehicleParameters
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
import matplotlib.pyplot as plt
from commonroad.visualization.mp_renderer import MPRenderer


def visualize_state_list(collision_checker, state_list: List[State], scenario, obs_shape):
    rnd = MPRenderer()
    trajectory = transfer_state_list_to_obstacle(scenario, state_list, obs_shape)
    scenario.draw(rnd, draw_params={'time_begin': 0, 'trajectory': {'draw_trajectory': True},
                                    "occupancy": {"draw_occupancies": 1}})
    trajectory.draw(rnd, draw_params={'time_begin': 0, 'trajectory': {'draw_trajectory': True},
                                      "occupancy": {"draw_occupancies": 1}})
    collision_checker.draw(rnd, draw_params={'time_begin': 0, 'facecolor': 'blue', 'draw_mesh': False})
    rnd.render()
    plt.show()


def check_velocity_feasibility(state: State, parameters: VehicleParameters):
    # the vehicle model in highD doesn't comply with commonroad vehicle models, thus the velocity limit for bmw320i
    # doesn't work for highD scenarios
    if state.velocity < 0 or \
            state.velocity > 60:  # parameters.longitudinal.v_max:
        return False
    return True


def check_steering_angle_feasibility(state: State, parameters: VehicleParameters):
    # if not hasattr(state, "steering_angle")
    if state.steering_angle < parameters.steering.min or \
            state.steering_angle > parameters.steering.max:
        return False
    return True


def transfer_state_list_to_obstacle(scenario, state_list, shape):
    """
    Transfers given state list into a dummy vehicle.
    :param scenario: given scenario
    :param state_list: given state list
    :return:
    """
    dynamic_obstacle_prediction = transfer_state_list_to_prediction(state_list, shape, scenario.dt)

    dynamic_obstacle_id = scenario.generate_object_id()
    dynamic_obstacle_type = ObstacleType.CAR
    dynamic_obstacle_new = DynamicObstacle(dynamic_obstacle_id,
                                           dynamic_obstacle_type,
                                           shape,
                                           state_list[0],
                                           dynamic_obstacle_prediction)
    return dynamic_obstacle_new


def transfer_state_list_to_prediction(state_list, shape, dt):
    """
    Transfers given state list into a dummy vehicle.
    :param state_list: given state list
    :return:
    """
    dynamic_obstacle_trajectory = Trajectory(state_list[0].time_step, state_list)
    dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, shape)
    return dynamic_obstacle_prediction


def update_ego_vehicle(road_network: RoadNetwork,
                       ego_vehicle: Vehicle,
                       updated_ego_states: List[State],
                       cut_off_time: int,
                       dt):
    """
    Update the ego vehicle based on the new given trajectory
    """
    ego_initial_state = ego_vehicle.states_cr[0]
    if cut_off_time == 0:
        acceleration = 0.0
        jerk = 0.0
    else:
        cut_off_state = updated_ego_states[cut_off_time - 1]
        if cut_off_time == 1:
            pre_cut_off_state = ego_initial_state
        else:
            pre_cut_off_state = updated_ego_states[cut_off_time - 2]
        acceleration = cut_off_state.acceleration
        if not hasattr(pre_cut_off_state, "acceleration"):
            pre_cut_off_state.acceleration = 0
        jerk = _compute_jerk(acceleration, pre_cut_off_state.acceleration, dt)
    # cut-off state changes it's input values, but the states stay unchanged
    ego_vehicle.states_lon[cut_off_time] = update_curvilinear_states_long(ego_vehicle.states_lon[cut_off_time],
                                                                          acceleration, jerk)
    reference_lane = ego_vehicle.lane
    # print(ego_vehicle.lanelet_assignment)

    for state in updated_ego_states[cut_off_time:]:
        acceleration = state.acceleration
        if state.time_step - 1 in ego_vehicle.states_lon:
            previous_acceleration = ego_vehicle.states_lon[state.time_step - 1].a
        else:  # previous state out of projection domain
            previous_acceleration = 0.0
        jerk = _compute_jerk(acceleration, previous_acceleration,
                             dt)
        state_lon, state_lat = create_curvilinear_states(state.position,
                                                         state.velocity, acceleration, jerk, state.orientation,
                                                         reference_lane, )
        if state_lon is None or state_lat is None:
            break
        ego_vehicle.states_lon[state.time_step] = state_lon
        ego_vehicle.states_lat[state.time_step] = state_lat
        ego_vehicle.states_cr[state.time_step] = state
        # ego_vehicle.signal_series[state.time_step] = obstacle.signal_state_at_time_step(
        #     state.time_step) # todo: the signal state?

        ego_shape = ego_vehicle.shape.rotate_translate_local(state.position,
                                                             state.orientation)
        # use the shape lanelet assignment
        ego_vehicle.lanelet_assignment[state.time_step] = \
            set(road_network.lanelet_network.find_lanelet_by_shape(ego_shape))
    if ego_vehicle.end_time > updated_ego_states[-1].time_step:
        for time_step in range(updated_ego_states[-1].time_step + 1, ego_vehicle.end_time + 1):
            del ego_vehicle.states_lon[time_step]
            del ego_vehicle.states_lat[time_step]
            del ego_vehicle.states_cr[time_step]
            del ego_vehicle.lanelet_assignment[time_step]


@functools.lru_cache()
def int_round(some_float, tolerance=1):
    """
    Round function using int.
    :param some_float: number
    :param tolerance: float point
    :return: rounded number
    """
    p = float(10 ** tolerance)
    if some_float < 0:
        return int(some_float * p - 0.5) / p
    else:
        return int(some_float * p + 0.5) / p
