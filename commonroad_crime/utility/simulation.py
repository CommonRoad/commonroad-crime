__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
from enum import Enum
import numpy as np
import copy
from typing import Union, List
from abc import ABC, abstractmethod

from commonroad.scenario.obstacle import DynamicObstacle, State
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.general as utils_general


class Maneuver(str, Enum):
    BRAKE = "brake"
    CONSTANT = "constant velocity"
    KICKDOWN = "kick-down"
    STEERLEFT = "steer to the left"
    STEERRIGHT = "steer to the right"
    NONE = ''


class SimulationBase(ABC):
    def __init__(self, maneuver: Union[Maneuver, None],
                 simulated_vehicle: DynamicObstacle,
                 config: CriMeConfiguration):
        self._maneuver = maneuver
        # currently: point mass model since KS model has some infeasibility issues
        self._input: State = State(acceleration=0,
                                   acceleration_y=0)

        self.dt = config.scenario.dt
        self.time_horizon = simulated_vehicle.prediction.final_time_step

        self.simulated_vehicle = simulated_vehicle
        self.parameters = config.vehicle.cartesian
        self.vehicle_dynamics = config.vehicle.dynamic
        self.plot = config.debug.draw_visualization

    def update_maneuver(self, maneuver: Maneuver):
        self._maneuver = maneuver

    def update_inputs_x_y(self, ref_state: State, a_long: float, a_lat: float):
        # includes the jerk limits
        self.input.acceleration = np.clip(a_long * math.cos(ref_state.orientation) -
                                          a_lat * math.sin(ref_state.orientation),
                                          max(ref_state.acceleration + self.parameters.j_x_min * self.dt,
                                              self.parameters.a_x_min),
                                          min(ref_state.acceleration + self.parameters.j_x_max * self.dt,
                                              self.parameters.a_x_max))
        self.input.acceleration_y = np.clip(a_long * math.sin(ref_state.orientation) +
                                            a_lat * math.cos(ref_state.orientation),
                                            max(ref_state.acceleration_y + self.parameters.j_y_min * self.dt,
                                                self.parameters.a_y_min),
                                            min(ref_state.acceleration_y + self.parameters.j_y_max * self.dt,
                                                self.parameters.a_y_max))
        self.input.time_step = ref_state.time_step
        ref_state.acceleration = self.input.acceleration
        ref_state.acceleration_y = self.input.acceleration_y

    def initialize_state_list(self, time_step: int) -> List[State]:
        """
        Initializing the state list based on the given time step. All the states before the time step would be returned.
        """
        state_list = []
        if time_step is not 0:
            for ts in range(0, time_step):
                state = self.simulated_vehicle.state_at_time(ts)
                check_elements_state(state)
                state_list.append(state)
        return state_list

    @property
    def maneuver(self):
        return self._maneuver

    @maneuver.setter
    def maneuver(self, maneuver: Maneuver):
        self._maneuver = maneuver

    @property
    def input(self):
        return self._input

    @abstractmethod
    def set_inputs(self, ref_state: State) -> None:
        """
        sets the input pairs
        """
        pass

    @abstractmethod
    def simulate_state_list(self, start_time_step: int, rnd: MPRenderer = None) -> List[State]:
        """
        forward simulation of the state list
        """
        pass


class SimulationMonteCarlo(SimulationBase):
    """
    Simulate the trajectory using Monte-Carlo sampling
    """
    def __init__(self,
                 maneuver: Union[Maneuver],
                 simulated_vehicle: DynamicObstacle,
                 config: CriMeConfiguration):
        super(SimulationMonteCarlo, self).__init__(maneuver, simulated_vehicle, config)

    def set_inputs(self, ref_state: State) -> None:
        pass

    def simulate_state_list(self, start_time_step: int, rnd: MPRenderer = None) -> List[State]:
        pass


class SimulationLong(SimulationBase):
    """
    Simulate the trajectory in the longitudinal direction.
    """

    def __init__(self,
                 maneuver: Union[Maneuver],
                 simulated_vehicle: DynamicObstacle,
                 config: CriMeConfiguration):
        if maneuver is not Maneuver.BRAKE and not Maneuver.KICKDOWN and not Maneuver.CONSTANT:
            raise ValueError(
                f"<Criticality/Simulation>: provided maneuver {maneuver} is not supported or goes to the wrong category")
        super(SimulationLong, self).__init__(maneuver, simulated_vehicle, config)

    def set_inputs(self, ref_state: State) -> None:
        """
        Sets inputs for the longitudinal simulation
        """
        check_elements_state(ref_state)
        # set the lateral acceleration to 0
        a_lat = 0
        # set the longitudinal acceleration based on the vehicle's capability and the maneuver
        if self.maneuver is Maneuver.BRAKE:
            a_long = - self.parameters.longitudinal.a_max
        elif self.maneuver is Maneuver.KICKDOWN:
            v_switch = self.parameters.longitudinal.v_switch
            if ref_state.velocity > v_switch:
                a_long = self.parameters.longitudinal.a_max * v_switch / ref_state.velocity
            else:
                a_long = self.parameters.longitudinal.a_max
        else:
            a_long = 0
        self.update_inputs_x_y(ref_state, a_long, a_lat)

    def simulate_state_list(self, start_time_step: int, rnd: MPRenderer = None):
        """
        Simulates the longitudinal state list from the given start time step.
        """
        # using copy to prevent the change of the initial trajectory
        pre_state = copy.deepcopy(self.simulated_vehicle.state_at_time(start_time_step))
        state_list = self.initialize_state_list(start_time_step)
        # update the input
        self.set_inputs(pre_state)
        state_list.append(pre_state)
        while pre_state.time_step < self.time_horizon:  # not <= since the simulation stops at the final step
            suc_state = self.vehicle_dynamics.simulate_next_state(pre_state, self.input, self.dt, throw=False)
            if suc_state and self.check_velocity_feasibility(suc_state):
                check_elements_state(suc_state, pre_state, self.dt)
                state_list.append(suc_state)
                pre_state = suc_state
                # update the input
                self.set_inputs(pre_state)
            else:
                # the simulated state is infeasible, i.e., further acceleration/deceleration is not permitted
                self.maneuver = Maneuver.CONSTANT
                if suc_state is not None:
                    if suc_state.velocity < 0:
                        pre_state.velocity = 0
                        pre_state.velocity_y = 0
                self.set_inputs(pre_state)
        return state_list

    def check_velocity_feasibility(self, state: State) -> bool:
        # the vehicle model in highD doesn't comply with commonroad vehicle models, thus the velocity limit
        # doesn't work for highD scenarios
        if state.velocity < 0 or \
                state.velocity > self.parameters.longitudinal.v_max:  # parameters.longitudinal.v_max:
            return False
        return True


class SimulationLat(SimulationBase):
    """
    Simulate the trajectory in the lateral direction.
    """

    def __init__(self,
                 maneuver: Union[Maneuver],
                 simulated_vehicle: DynamicObstacle,
                 config: CriMeConfiguration):
        if maneuver is not Maneuver.STEERLEFT and not Maneuver.STEERRIGHT:
            raise ValueError(
                f"<Criticality/Simulation>: provided maneuver {maneuver} is not supported or goes to the wrong category")
        super(SimulationLat, self).__init__(maneuver, simulated_vehicle, config)
        self._scenario = config.scenario
        self._lateral_distance_mode = config.time_scale.steer_width

    def set_inputs(self, ref_state: State) -> None:
        """
        Sets inputs for the lateral simulation
        """
        check_elements_state(ref_state)
        # set the longitudinal acceleration to 0
        a_long = 0
        self.input.acceleration = np.clip(0,
                                          max(ref_state.acceleration + self.parameters.j_x_min * self.dt,
                                              self.parameters.a_x_min),
                                          min(ref_state.acceleration + self.parameters.j_x_max * self.dt,
                                              self.parameters.a_x_max))
        # set the lateral acceleration based on the vehicle's capability and the maneuver
        v_switch = self.parameters.longitudinal.v_switch
        if ref_state.velocity > v_switch:
            a_lat = self.parameters.longitudinal.a_max * v_switch / ref_state.velocity
        else:
            a_lat = self.parameters.longitudinal.a_max
        if self.maneuver in [Maneuver.STEERRIGHT]:
            # right-hand coordinate
            a_lat = -a_lat
        # includes the jerk limits
        self.update_inputs_x_y(ref_state, a_long, a_lat)

    def set_bang_bang_timestep_orientation(self, position: np.ndarray):
        """
        Sets the time for the bang–bang controller of the lane change.
        """
        lanelet_id = self._scenario.lanelet_network.find_lanelet_by_position([position])[0]
        lateral_dis, orientation = utils_general.compute_lanelet_width_orientation(self._scenario.lanelet_network.
                                                                                   find_lanelet_by_id(lanelet_id[0]),
                                                                                   position)
        if self._lateral_distance_mode == 1:
            lateral_dis = 0.8
        # Modified from Eq. (11) in Pek, C., Zahn, P. and Althoff, M., Verifying the safety of lane change maneuvers of
        # self-driving vehicles based on formalized traffic rules. In IV 2017 (pp. 1477-1483). IEEE.
        total_timestep = math.sqrt(4 * lateral_dis / min(abs(self.parameters.a_y_max), abs(self.parameters.a_y_min)))
        return int(total_timestep / (2 * self.dt)), orientation

    def simulate_state_list(self, start_time_step: int, rnd: MPRenderer = None):
        """
        Simulates the lateral state list from the given start time step.
        """
        pre_state = copy.deepcopy(self.simulated_vehicle.state_at_time(start_time_step))
        state_list = self.initialize_state_list(start_time_step)
        # update the input
        self.set_inputs(pre_state)
        state_list.append(pre_state)
        lane_orient = 0.
        for _ in range(2):
            bang_bang_ts, lane_orient = self.set_bang_bang_timestep_orientation(pre_state.position)
            max_orient = self.set_maximal_orientation(lane_orient)
            bang_state_list = self.bang_bang_simulation(pre_state, bang_bang_ts, max_orient)
            state_list += bang_state_list
            pre_state = bang_state_list[-1]
            if self.maneuver is Maneuver.STEERLEFT:
                self.maneuver = Maneuver.STEERRIGHT
            else:
                self.maneuver = Maneuver.STEERLEFT
        # updates the orientation
        while pre_state.time_step < self.time_horizon:  # not <= since the simulation stops at the final step
            self.set_inputs(pre_state)
            self.input.acceleration_y = 0
            # drives along the lane direction
            pre_state.velocity_y = pre_state.velocity * math.sin(lane_orient)
            suc_state = self.vehicle_dynamics.simulate_next_state(pre_state, self.input, self.dt, throw=False)
            state_list.append(suc_state)
            pre_state = suc_state
        self.set_inputs(state_list[-1])
        return state_list

    def set_maximal_orientation(self, lane_orientation):
        """
        adds additional constraints for the orientation.
        """
        if self.maneuver in [Maneuver.STEERLEFT]:
            return lane_orientation + math.pi / 4
        elif self.maneuver in [Maneuver.STEERRIGHT]:
            return lane_orientation - math.pi / 4
        else:
            pass

    def bang_bang_simulation(self, init_state: State, simulation_length: int, max_orientation: float):
        pre_state = init_state
        state_list = []
        while pre_state.time_step < init_state.time_step + simulation_length:
            suc_state = self.vehicle_dynamics.simulate_next_state(pre_state, self.input, self.dt, throw=False)
            if suc_state:
                check_elements_state(suc_state)
                state_list.append(suc_state)
                pre_state = suc_state
                self.set_inputs(pre_state)
                if abs(suc_state.orientation) > abs(max_orientation):
                    break
            else:
                self.input.acceleration_y = 0
        return state_list


def check_elements_state(state: State, prev_state: State = None, dt: float = None):
    """
    checks the missing elements needed for PM model
    """
    if not hasattr(state, "slip_angle"):
        state.slip_angle = 0
    if not hasattr(state, "yaw_rate"):
        state.yaw_rate = 0
    if not hasattr(state, "velocity_y"):
        state.velocity_y = state.velocity * math.sin(state.orientation)
        state.velocity = state.velocity * math.cos(state.orientation)

    if prev_state is not None:
        if not hasattr(state, "acceleration"):
            state.acceleration = (state.velocity - prev_state.velocity) / dt
    else:
        state.acceleration = 0
    if hasattr(state, "acceleration") and not hasattr(state, "acceleration_y"):
        state.acceleration_y = state.acceleration * math.sin(state.orientation)
        state.acceleration = state.acceleration * math.cos(state.orientation)