import math
from enum import Enum
from math import sqrt
from typing import Union
from abc import ABC, abstractmethod

from commonroad.common.solution import VehicleType
from commonroad.scenario.obstacle import DynamicObstacle, State
from commonroad_dc.feasibility.vehicle_dynamics import PointMassDynamics

from stl_crmonitor.crmonitor.common.world_state import WorldState
from commonroad_criticality.time_metrics.utils import check_velocity_feasibility


class CutOffAction(str, Enum):
    BRAKE = "brake"
    CONSTANT = "constant velocity"
    KICKDOWN = "kick-down"
    LANECHANGELEFT = "lane change to the left"
    LANECHANGERIGHT = "lane change to the right"
    STEADYSPEED = "steady speed"
    STEERLEFT = "steer to the left"
    STEERRIGHT = "steer to the right"


class SimulationBase(ABC):
    def __init__(self, action: Union[CutOffAction, None],
                 simulated_vehicle: DynamicObstacle,
                 start_time: Union[int, None],
                 dt: float = 0.1):
        self._dt = dt
        self._time_horizon = simulated_vehicle.prediction.final_time_step
        self._action = action
        self._simulated_vehicle = simulated_vehicle
        self._start_time = start_time
        self._input: State = State(steering_angle_speed=0,
                                   acceleration=0)
        # currently: point mass model since KS model has some infeasibility issues
        self._vehicle_dynamics = PointMassDynamics(VehicleType.BMW_320i)
        self._parameters = self._vehicle_dynamics.parameters
        self._state_list = []

    @property
    def cut_off_state(self):
        return self._simulated_vehicle.state_at_time(self._start_time)

    @property
    def action(self):
        return self._action

    @action.setter
    def action(self, action: CutOffAction):
        self._action = action

    @property
    def parameters(self):
        return self._parameters

    @property
    def vehicle_dynamics(self):
        return self._vehicle_dynamics

    @property
    def input(self):
        return self._input

    @abstractmethod
    def set_inputs(self, *args, **kwargs):
        """
        sets the input pairs
        """
        pass

    @abstractmethod
    def simulate_state_list(self):
        """
        forward simulation of the state list
        """
        pass

    @abstractmethod
    def check_action_type(self, action):
        pass

    def update_action(self,
                      action: CutOffAction,
                      start_time: int):
        """
        update the action and cut-off time step
        """
        self.check_action_type(action)
        self.action = action
        self._start_time = start_time
        self.initialize_state_list()

    def initialize_state_list(self):
        if self._start_time != 0:
            # if the state list doesn't start with time step 0, then states_in_time_interval returns None for it
            self._state_list = [self._simulated_vehicle.initial_state] + \
                               self._simulated_vehicle.prediction.trajectory.states_in_time_interval(1,
                                                                                                     self._start_time)
        else:
            self._state_list = []
        for state in self._state_list:
            if not hasattr(state, "velocity_y"):
                state.velocity_y = state.velocity * math.sin(state.orientation)


class SimulationLong(SimulationBase, ABC):
    def __init__(self, action: Union[CutOffAction, None],
                 simulated_vehicle: DynamicObstacle,
                 start_time: Union[int, None],
                 dt: float):
        self.j_limit = 5
        super().__init__(action, simulated_vehicle, start_time, dt=dt)

    def set_inputs(self, pre_state):
        """
        sets inputs for the longitudinal simulation
        """
        velocity = pre_state.velocity
        self._input.acceleration_y = 0
        a_max = self._vehicle_dynamics.parameters.longitudinal.a_max
        if self.action == CutOffAction.BRAKE:
            # braking
            self._input.acceleration = - a_max
        elif self.action == CutOffAction.KICKDOWN:
            # kickdowning
            v_switch = self._vehicle_dynamics.parameters.longitudinal.v_switch
            if velocity > v_switch:
                a_max = self._vehicle_dynamics.parameters.longitudinal.a_max * v_switch / velocity
            self._input.acceleration = a_max
        else:
            # constant velocity
            self._input.acceleration = min(0, pre_state.acceleration + self.j_limit * self._dt)

    def simulate_state_list(self):
        pre_state = self.cut_off_state
        self.set_inputs(pre_state)
        pre_state.velocity_y = 0
        pre_state.acceleration = self._input.acceleration
        while pre_state.time_step < self._time_horizon:
            self._input.time_step = pre_state.time_step
            suc_state = self._vehicle_dynamics.simulate_next_state(pre_state, self._input, self._dt, throw=False)
            if suc_state and check_velocity_feasibility(suc_state, self._vehicle_dynamics.parameters):
                check_elements_state(suc_state)
                if not hasattr(suc_state, "acceleration"):
                    suc_state.acceleration = (suc_state.velocity - pre_state.velocity) / self._dt
                # if abs(suc_state.orientation) > np.pi/2:
                #     suc_state.orientation = np.sign(suc_state.orientation)*abs(suc_state.orientation-np.pi/2)
                self._state_list.append(suc_state)
                pre_state = suc_state
                self.set_inputs(pre_state)
            else:
                self._input.acceleration = 0
        return self._state_list

    def check_action_type(self, action):
        assert action == CutOffAction.BRAKE or action == CutOffAction.KICKDOWN \
               or action == CutOffAction.STEADYSPEED, \
            "<SimulationLong>: provided action {} is not supported".format(action)


class SimulationLateral(SimulationBase, ABC):
    def __init__(self, action: Union[CutOffAction, None],
                 simulated_vehicle: DynamicObstacle,
                 start_time: Union[int, None],
                 world_state: WorldState,
                 dt: float):
        super().__init__(action, simulated_vehicle, start_time, dt=dt)
        self._world_state = world_state

    def set_target_lane(self):
        """
        based on the lane structure within stl monitor.
        """
        if self.action == CutOffAction.LANECHANGELEFT:
            return self._world_state.ego_vehicle.lane.adj_left
        elif self.action == CutOffAction.LANECHANGERIGHT:
            return self._world_state.ego_vehicle.lane.adj_right
        elif self.action in (CutOffAction.STEERLEFT, CutOffAction.STEERRIGHT):
            return self._world_state.ego_vehicle.lane
        else:
            return None

    def calc_total_time(self, lat_dist):
        """
        Modified from Eq. (11) in Pek, C., Zahn, P. and Althoff, M., Verifying the safety of lane change maneuvers of
         self-driving vehicles based on formalized traffic rules. In IV 2017 (pp. 1477-1483). IEEE.
        """
        return sqrt(4 * lat_dist / abs(self._input.acceleration_y))

    def calc_leave_time(self, lat_dist):
        """
        Miller, Christina, Christian Pek, and Matthias Althoff. "Efficient mixed-integer programming for longitudinal
        and lateral motion planning of autonomous vehicles." 2018 IEEE Intelligent Vehicles Symposium (IV). IEEE, 2018.
        """
        return sqrt(2 * abs(lat_dist / self._input.acceleration_y))

    def set_inputs(self, velocity):
        """
        for lane-related traffic rules, we specify the steering maneuver as lane change, i.e., lateral offsets are
        based on the lane width.
        """
        self._input.acceleration = 0
        v_switch = self._vehicle_dynamics.parameters.longitudinal.v_switch
        if velocity > v_switch:
            a_max = self._vehicle_dynamics.parameters.longitudinal.a_max * v_switch / velocity
        else:
            a_max = self._vehicle_dynamics.parameters.longitudinal.a_max
        if self.action in [CutOffAction.LANECHANGELEFT, CutOffAction.STEERLEFT]:
            # steering to the left
            self._input.acceleration_y = a_max
        elif self.action in [CutOffAction.LANECHANGERIGHT, CutOffAction.STEERRIGHT]:
            # steering to the right
            self._input.acceleration_y = - a_max
        else:
            self._input.acceleration_y = 0

    def set_bang_bang_time(self, ego_s, ego_d, target_lane):
        if self.action in [CutOffAction.LANECHANGELEFT, CutOffAction.LANECHANGERIGHT]:
            # todo: fix the lane of the ego
            ego_lane_width = self._world_state.ego_vehicle.lane.width(ego_s)
            ego_to_lane_boundary = ego_lane_width / 2 - abs(ego_d)
            lateral_distance = ego_to_lane_boundary + target_lane.width(ego_s) / 2
        else:
            # from paper: A flexible method for criticality assessment in driver assistance systems
            lateral_distance = 0.8
        total_time = self.calc_total_time(lateral_distance)
        bang_bang_time = int(total_time / (2 * self._dt))
        return bang_bang_time

    def set_maximal_orientation(self, lane_orientation):
        """
        adds additional constraints for the orientation.
        """
        if self.action in [CutOffAction.LANECHANGELEFT, CutOffAction.STEERLEFT]:
            return lane_orientation + math.pi / 4
        elif self.action in [CutOffAction.LANECHANGERIGHT, CutOffAction.STEERRIGHT]:
            return lane_orientation - math.pi / 4
        else:
            pass

    def bang_bang_simulation(self, state, simulation_time, max_orientation):
        pre_state = state
        suc_state = state
        while pre_state.time_step < state.time_step + simulation_time:
            self._input.time_step = pre_state.time_step
            suc_state = self._vehicle_dynamics.simulate_next_state(pre_state, self._input, self._dt, throw=False)
            if suc_state:  # and check_steering_angle_feasibility(suc_state, self._vehicle_dynamics.parameters):
                check_elements_state(suc_state)
                self._state_list.append(suc_state)
                pre_state = suc_state
                if abs(suc_state.orientation) > abs(max_orientation):
                    break
            else:
                # in case the suc_state is infeasible
                self._input.acceleration_y = 0
        return suc_state

    def simulate_state_list(self):
        self.set_inputs(self.cut_off_state.velocity)
        target_lane = self.set_target_lane()
        if target_lane is None:
            # lane change is impossible
            return None
        current_ego_s, current_ego_d = self._world_state.ego_vehicle.lane.clcs.convert_to_curvilinear_coords(
            self.cut_off_state.position[0], self.cut_off_state.position[1])
        bang_bang_time = self.set_bang_bang_time(current_ego_s, current_ego_d, target_lane)
        lane_orientation = self._world_state.ego_vehicle.lane.orientation(current_ego_s)
        max_orientation = self.set_maximal_orientation(lane_orientation)
        current_state = self.cut_off_state
        for i in range(2):
            current_state = self.bang_bang_simulation(current_state, bang_bang_time, max_orientation)
            self._input.acceleration_y = - self._input.acceleration_y
        while current_state.time_step < self._time_horizon:
            self._input.acceleration_y = 0
            self._input.time_step = current_state.time_step
            # drive along the lane direction
            current_state.velocity_y = current_state.velocity * math.sin(lane_orientation)
            current_state = self._vehicle_dynamics.simulate_next_state(current_state, self._input, self._dt,
                                                                       throw=False)
            self._state_list.append(current_state)
        return self._state_list

    def check_action_type(self, action):
        assert action in [CutOffAction.LANECHANGELEFT, CutOffAction.LANECHANGERIGHT,
                          CutOffAction.STEERLEFT, CutOffAction.STEERRIGHT], \
            "<SimulationLateral>: provided action {} is not supported".format(action)


def check_elements_state(state: State):
    """
    checks the missing elements needed for PM model
    """
    if not hasattr(state, "slip_angle"):
        state.slip_angle = 0
    if not hasattr(state, "yaw_rate"):
        state.yaw_rate = 0
    if not hasattr(state, "velocity_y"):
        state.velocity_y = state.velocity * math.cos(state.orientation)


def check_elements_state_list(state_list, dt):
    for k in range(len(state_list) - 1):
        if not hasattr(state_list[k], "yaw_rate"):
            state_list[k].yaw_rate = (state_list[k + 1].orientation - state_list[k].orientation) / dt
        if not hasattr(state_list[k], "slip_angle"):
            state_list[k].slip_angle = 0
        if not hasattr(state_list[k], "steering_angle"):
            state_list[k].steering_angle = 0
        if not hasattr(state_list[k], "acceleration"):
            state_list[k].acceleration = (state_list[k + 1].velocity - state_list[k].velocity) / dt
        if not hasattr(state_list[k], "velocity_y"):
            state_list[k].velocity_y = state_list[k].velocity * math.cos(state_list[k].orientation)
    state_list[-1].yaw_rate = 0
    state_list[-1].slip_angle = 0
    state_list[-1].steering_angle = 0
    state_list[-1].acceleration = 0
    state_list[-1].velocity_y = state_list[k].velocity * math.cos(state_list[k].orientation)