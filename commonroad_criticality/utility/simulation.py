
import math
from enum import Enum
import numpy as np
from typing import Union, List
from abc import ABC, abstractmethod

from commonroad.scenario.obstacle import DynamicObstacle, State

from commonroad_criticality.data_structure.configuration import CriticalityConfiguration


class Maneuver(str, Enum):
    BRAKE = "brake"
    CONSTANT = "constant velocity"
    KICKDOWN = "kick-down"
    STEERLEFT = "steer to the left"
    STEERRIGHT = "steer to the right"


class SimulationBase(ABC):
    def __init__(self, maneuver: Union[Maneuver, None],
                 simulated_vehicle: DynamicObstacle,
                 config: CriticalityConfiguration):
        self._maneuver = maneuver
        # currently: point mass model since KS model has some infeasibility issues
        self._input: State = State(acceleration=0,
                                   acceleration_y=0)

        self.dt = config.scenario.dt
        self.time_horizon = simulated_vehicle.prediction.final_time_step

        self.simulated_vehicle = simulated_vehicle
        self.parameters = config.vehicle.cartesian
        self.vehicle_dynamics = config.vehicle.dynamic

    def update_maneuver(self, maneuver: Maneuver):
        self._maneuver = maneuver

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
    def set_inputs(self, *args, **kwargs):
        """
        sets the input pairs
        """
        pass

    @abstractmethod
    def simulate_state_list(self, start_time_step: int):
        """
        forward simulation of the state list
        """
        pass


class SimulationLong(SimulationBase):
    """
    Simulate the trajectory in the longitudinal direction.
    """

    def __init__(self,
                 maneuver: Union[Maneuver],
                 simulated_vehicle: DynamicObstacle,
                 config: CriticalityConfiguration):
        assert maneuver is Maneuver.BRAKE or Maneuver.KICKDOWN or Maneuver.CONSTANT,\
            f"<Criticality/Simulation>: provided maneuver {maneuver} is not support or goes to the wrong category"
        super(SimulationLong, self).__init__(maneuver, simulated_vehicle, config)

    def set_inputs(self, ref_state: State) -> None:
        """
        sets inputs for the longitudinal simulation
        """
        # set the lateral acceleration to 0
        self.input.acceleration_y = 0
        # set the longitunal acceleration based on the vehicle's capability and the maneuver
        if self.maneuver is Maneuver.BRAKE:
            a_x = - self.parameters.longitudinal.a_max
        elif self.maneuver is Maneuver.KICKDOWN:
            v_switch = self.parameters.longitudinal.v_switch
            if ref_state.velocity > v_switch:
                a_x = self.parameters.longitudinal.a_max * v_switch / ref_state.velocity
            else:
                a_x = self.parameters.longitudinal.a_max
        else:
            a_x = 0
        # includes the jerk limits
        self.input.acceleration = np.clip(a_x,
                                          ref_state.acceleration + self.parameters.j_x_min * self.dt,
                                          ref_state.acceleration + self.parameters.j_x_max * self.dt)
        self.input.time_step = ref_state.time_step
        ref_state.acceleration = self.input.acceleration
        ref_state.acceleration_y = self.input.acceleration_y

    def simulate_state_list(self, start_time_step: int) -> List[State]:
        pre_state = self.simulated_vehicle.state_at_time(start_time_step)
        state_list = self.initialize_state_list(start_time_step)
        # update the input
        self.set_inputs(pre_state)
        state_list.append(pre_state)
        while pre_state.time_step < self.time_horizon:
            suc_state = self.vehicle_dynamics.simulate_next_state(pre_state, self.input, self.dt, throw=False)
            if suc_state and self.check_velocity_feasibility(suc_state):
                check_elements_state(suc_state, pre_state, self.dt)
                state_list.append(suc_state)
                pre_state = suc_state
                # update the input
                self.set_inputs(pre_state)
            else:
                # the simulated state is infeasible, i.e., further acceleration/deceleration is not permitted
                self.input.acceleration = 0
        return state_list

    def check_velocity_feasibility(self, state: State) -> bool:
        # the vehicle model in highD doesn't comply with commonroad vehicle models, thus the velocity limit
        # doesn't work for highD scenarios
        if state.velocity < 0 or \
                state.velocity > self.parameters.longitudinal.v_max:  # parameters.longitudinal.v_max:
            return False
        return True


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
