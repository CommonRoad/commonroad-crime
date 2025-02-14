__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.2"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
from enum import Enum
import numpy as np
from scipy.stats import norm
import copy
from typing import Union, List, Tuple
from abc import ABC, abstractmethod

from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Tag
from commonroad.scenario.state import PMInputState, PMState, KSState

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.utility.general import (
    check_elements_state,
    compute_curvature_from_polyline_start_end,
)
from commonroad_crime.utility.solver import (
    compute_lanelet_width_orientation,
    convert_to_0_2pi,
)


class Maneuver(str, Enum):
    BRAKE = "brake"
    CONSTANT = "constant velocity"
    KICKDOWN = "kick-down"
    STEERLEFT = "steer to the left"
    STEERRIGHT = "steer to the right"
    TURNLEFT = "turn to the left"
    TURNRIGHT = "turn to the right"
    OVERTAKELEFT = "overtake from the left"
    OVERTAKERIGHT = "overtake from the right"
    NONE = ""

    STOPMC = "stop with Monte Carlo"
    LANECHANGEMC = "lane change with Monte Carlo"
    TURNMC = "turn with Monte Carlo"
    OVERTAKEMC = "overtake with Monte Carlo"
    RANDOMMC = "random with Monte Carlo"


class SimulationBase(ABC):
    def __init__(
        self,
        maneuver: Union[Maneuver, None],
        simulated_vehicle: DynamicObstacle,
        config: CriMeConfiguration,
    ):
        # currently: point mass model since KS model has some infeasibility issues
        self._input: PMInputState = PMInputState(acceleration=0, acceleration_y=0)

        self.maneuver = maneuver
        self.initialize_simulator()
        self.dt = config.scenario.dt
        self.time_horizon = simulated_vehicle.prediction.final_time_step

        self.simulated_vehicle = simulated_vehicle
        self.parameters = config.vehicle.params
        self.cartesian = config.vehicle.cartesian
        self.vehicle_dynamics = config.vehicle.dynamic
        self.plot = config.debug.draw_visualization
        self.braking_vel_threshold = config.time.braking_vel_threshold

        self.a_long = 0
        self.a_lat = 0

    def update_inputs_x_y(self, ref_state: Union[PMInputState, PMState, KSState]):
        # includes the jerk limits
        if isinstance(ref_state, PMState):
            ref_orientation = math.atan2(ref_state.velocity_y, ref_state.velocity)
        else:
            ref_orientation = ref_state.orientation
        self.input.acceleration = np.clip(
            self.a_long * math.cos(ref_orientation)
            - self.a_lat * math.sin(ref_orientation),
            max(
                ref_state.acceleration + self.cartesian.j_x_min * self.dt,
                self.cartesian.a_x_min,
            ),
            min(
                ref_state.acceleration + self.cartesian.j_x_max * self.dt,
                self.cartesian.a_x_max,
            ),
        )
        self.input.acceleration_y = np.clip(
            self.a_long * math.sin(ref_orientation)
            + self.a_lat * math.cos(ref_orientation),
            max(
                ref_state.acceleration_y + self.cartesian.j_y_min * self.dt,
                self.cartesian.a_y_min,
            ),
            min(
                ref_state.acceleration_y + self.cartesian.j_y_max * self.dt,
                self.cartesian.a_y_max,
            ),
        )
        self.input.time_step = ref_state.time_step
        ref_state.acceleration = self.input.acceleration
        ref_state.acceleration_y = self.input.acceleration_y

    def initialize_state_list(self, time_step: int) -> List[KSState]:
        """
        Initializing the state list based on the given time step. All the states before the time step would be returned.
        """
        state_list = []
        # checking weather the time step is larger than the initial one
        initial_time_step = self.simulated_vehicle.initial_state.time_step
        if time_step > initial_time_step:
            for ts in range(initial_time_step, time_step):
                state = copy.deepcopy(self.simulated_vehicle.state_at_time(ts))
                check_elements_state(state)
                state_list.append(state)
        # additionally check the elements
        check_elements_state(self.simulated_vehicle.state_at_time(time_step))
        self.input.acceleration_y = self.simulated_vehicle.state_at_time(
            time_step
        ).acceleration_y
        self.input.acceleration = self.simulated_vehicle.state_at_time(
            time_step
        ).acceleration
        return state_list

    def update_maneuver(self, maneuver: Maneuver):
        self.maneuver = maneuver
        self.initialize_simulator()

    def initialize_simulator(self):
        pass

    @property
    def input(self):
        return self._input

    @abstractmethod
    def set_a_long_and_a_lat(
        self, ref_state: Union[PMState, KSState]
    ) -> Tuple[float, float]:
        """
        sets the longitudinal and lateral acceleration
        """
        pass

    @abstractmethod
    def set_inputs(self, ref_state: Union[PMState, KSState]) -> None:
        """
        sets the input pairs
        """
        pass

    @abstractmethod
    def simulate_state_list(
        self, start_time_step: int, given_time_limit: int = None
    ) -> List[Union[PMState, KSState]]:
        """
        forward simulation of the state list
        """
        pass

    def check_velocity_feasibility(self, state: Union[PMState, KSState]) -> bool:
        # the vehicle model in highD doesn't comply with commonroad vehicle models, thus the velocity limit
        # doesn't work for highD scenarios
        abs_velocity = np.sqrt(state.velocity**2 + state.velocity_y**2)
        if (
            abs_velocity < self.braking_vel_threshold
            or abs_velocity > self.parameters.longitudinal.v_max
        ):  # parameters.longitudinal.v_max:
            return False
        return True


class SimulationRandoMonteCarlo(SimulationBase):
    """
    Simulate the random behavior of vehicles.
    """

    def __init__(
        self,
        maneuver: Union[Maneuver],
        simulated_vehicle: DynamicObstacle,
        config: CriMeConfiguration,
    ):
        if maneuver not in [Maneuver.RANDOMMC]:
            raise ValueError(
                f"<Criticality/Simulation>: provided maneuver {maneuver} is not supported or goes to the wrong category"
            )
        super(SimulationRandoMonteCarlo, self).__init__(
            maneuver, simulated_vehicle, config
        )
        self.pdf = None  # probability density function

    def set_a_long_and_a_lat(self, ref_state: PMState):
        self.a_long = self.a_lat = self.parameters.longitudinal.a_max / 4

    def set_inputs(self, ref_state: PMState) -> None:
        self.set_a_long_and_a_lat(ref_state)
        a_long_norm = norm(0, abs(self.a_long))
        a_lat_norm = norm(0, abs(self.a_lat))
        self.a_long = np.random.normal(0, abs(self.a_long), 1)[0]
        self.a_lat = np.random.normal(0, abs(self.a_lat), 1)[0]
        self.pdf = a_lat_norm.pdf(self.a_lat) * a_long_norm.pdf(self.a_long)

    def simulate_state_list(
        self, start_time_step: int, given_time_limit: int = None
    ) -> List[PMState]:
        # using copy to prevent the change of the initial trajectory
        pre_state = copy.deepcopy(self.simulated_vehicle.state_at_time(start_time_step))
        state_list = self.initialize_state_list(start_time_step)
        # update the input
        check_elements_state(pre_state, self.input)
        self.set_inputs(pre_state)
        state_list.append(pre_state)
        if given_time_limit:
            self.time_horizon = given_time_limit
        while (
            pre_state.time_step < self.time_horizon
        ):  # not <= since the simulation stops at the final step
            self.update_inputs_x_y(pre_state)
            suc_state = self.vehicle_dynamics.simulate_next_state(
                PMState(
                    position=pre_state.position,
                    velocity=pre_state.velocity,
                    velocity_y=pre_state.velocity_y,
                    time_step=pre_state.time_step,
                ),
                self.input,
                self.dt,
                throw=False,
            )
            if suc_state and self.check_velocity_feasibility(suc_state):
                check_elements_state(suc_state, self.input)
                state_list.append(suc_state)
                pre_state = suc_state
                # update the input
                self.set_inputs(pre_state)
            else:
                # re-simulate for infeasible cases
                self.set_inputs(pre_state)
        return state_list


class SimulationLong(SimulationBase):
    """
    Simulate the trajectory in the longitudinal direction.
    """

    def __init__(
        self,
        maneuver: Union[Maneuver],
        simulated_vehicle: DynamicObstacle,
        config: CriMeConfiguration,
    ):
        if maneuver not in [
            Maneuver.BRAKE,
            Maneuver.KICKDOWN,
            Maneuver.CONSTANT,
            Maneuver.STOPMC,
            Maneuver.NONE,
        ]:
            raise ValueError(
                f"<Criticality/Simulation>: provided maneuver {maneuver} is not supported or goes to the wrong category"
            )
        super(SimulationLong, self).__init__(maneuver, simulated_vehicle, config)

    def set_a_long_and_a_lat(self, ref_state: PMState):
        # set the lateral acceleration to 0
        self.a_lat = 0
        # set the longitudinal acceleration based on the vehicle's capability and the maneuver
        v_switch = self.parameters.longitudinal.v_switch
        if self.maneuver is Maneuver.BRAKE:
            self.a_long = -self.parameters.longitudinal.a_max
        elif self.maneuver is Maneuver.KICKDOWN:
            if ref_state.velocity > v_switch:
                self.a_long = (
                    self.parameters.longitudinal.a_max * v_switch / ref_state.velocity
                )
            else:
                self.a_long = self.parameters.longitudinal.a_max
        elif self.maneuver is Maneuver.STOPMC:
            self.a_long = np.random.choice(
                [
                    self.parameters.longitudinal.a_max * v_switch / ref_state.velocity,
                    v_switch,
                ]
            )
        else:
            self.a_long = 0

    def set_inputs(self, ref_state: PMState) -> None:
        """
        Sets inputs for the longitudinal simulation
        """
        self.set_a_long_and_a_lat(ref_state)

    def simulate_state_list(self, start_time_step: int, given_time_limit: int = None):
        """
        Simulates the longitudinal state list from the given start time step.
        """
        state_list = self.initialize_state_list(start_time_step)
        # using copy to prevent the change of the initial trajectory
        pre_state = copy.deepcopy(self.simulated_vehicle.state_at_time(start_time_step))
        # update the input
        check_elements_state(pre_state, self.input)
        self.set_inputs(pre_state)
        state_list.append(pre_state)

        if given_time_limit:
            self.time_horizon = given_time_limit
        while (
            pre_state.time_step < self.time_horizon
        ):  # not <= since the simulation stops at the final step
            self.update_inputs_x_y(pre_state)
            suc_state = self.vehicle_dynamics.simulate_next_state(
                PMState(
                    position=pre_state.position,
                    velocity=pre_state.velocity,
                    velocity_y=pre_state.velocity_y,
                    time_step=pre_state.time_step,
                ),
                self.input,
                self.dt,
                throw=False,
            )
            if suc_state and self.check_velocity_feasibility(suc_state):
                check_elements_state(suc_state, self.input)
                state_list.append(suc_state)
                pre_state = suc_state
                # update the input
            else:
                # the simulated state is infeasible, i.e., further acceleration/deceleration is not permitted
                if suc_state is not None:
                    if (
                        suc_state.velocity**2 + suc_state.velocity_y**2
                        < self.braking_vel_threshold
                    ):
                        # slow down the vehicle immediately without changing the orientation to 0
                        pre_state.velocity = 1e-5 * pre_state.velocity
                        pre_state.velocity_y = 1e-5 * pre_state.velocity_y
                    for time_step in range(
                        pre_state.time_step + 1, self.time_horizon + 1
                    ):
                        stat_state = copy.deepcopy(pre_state)
                        stat_state.time_step = time_step
                        state_list.append(stat_state)
                break
        return state_list


class SimulationLongMonteCarlo(SimulationLong):
    """
    Simulate the longitudinal trajectory using Monte-Carlo sampling
    """

    def __init__(
        self,
        maneuver: Union[Maneuver],
        simulated_vehicle: DynamicObstacle,
        config: CriMeConfiguration,
    ):
        if maneuver not in [Maneuver.STOPMC]:
            raise ValueError(
                f"<Criticality/Simulation>: provided maneuver {maneuver} is not supported or goes to the wrong category"
            )
        super(SimulationLongMonteCarlo, self).__init__(
            maneuver, simulated_vehicle, config
        )
        self.pdf = 1  # probability density function

    def set_inputs(self, ref_state: PMState) -> None:
        self.set_a_long_and_a_lat(ref_state)
        a_long_norm = norm(0, abs(self.a_long))
        self.a_long = np.random.normal(0, abs(self.a_long), 1)[0]
        self.pdf = a_long_norm.pdf(self.a_long)
        if self.maneuver == Maneuver.STOPMC:
            self.a_long = -self.a_long


class SimulationLat(SimulationBase):
    """
    Simulate the trajectory in the lateral direction.
    """

    def __init__(
        self,
        maneuver: Maneuver,
        simulated_vehicle: DynamicObstacle,
        config: CriMeConfiguration,
    ):
        if maneuver not in [
            Maneuver.STEERLEFT,
            Maneuver.STEERRIGHT,
            Maneuver.OVERTAKELEFT,
            Maneuver.OVERTAKERIGHT,
            Maneuver.TURNLEFT,
            Maneuver.TURNRIGHT,
            Maneuver.LANECHANGEMC,
            Maneuver.TURNMC,
            Maneuver.OVERTAKEMC,
            Maneuver.NONE,
        ]:
            raise ValueError(
                f"<Criticality/Simulation>: provided maneuver {maneuver} is not supported or goes to the wrong category"
            )
        self._nr_stage = 0
        self._scenario = config.scenario
        self._lateral_distance_mode = config.time.steer_width
        self._direction = "left"  # 'right'
        super(SimulationLat, self).__init__(maneuver, simulated_vehicle, config)

    def initialize_simulator(self):
        if self.maneuver in [Maneuver.TURNLEFT, Maneuver.TURNRIGHT, Maneuver.TURNMC]:
            # one stage: a_y
            self._nr_stage = 1
        elif self.maneuver in [
            Maneuver.STEERRIGHT,
            Maneuver.STEERLEFT,
            Maneuver.LANECHANGEMC,
        ]:
            # first stage: a_y, second stage: - a_y
            self._nr_stage = 2
        else:
            # Overtake: a_y, -a_y, -a_y, -a_y
            self._nr_stage = 4

    def set_a_long_and_a_lat(self, ref_state: PMState):
        # set the longitudinal acceleration to 0
        self.a_long = 0
        # set the lateral acceleration based on the vehicle's capability and the maneuver
        # TODO: check setting of a_lat
        v_switch = self.parameters.longitudinal.v_switch
        if ref_state.velocity > v_switch:
            self.a_lat = (
                self.parameters.longitudinal.a_max * v_switch / ref_state.velocity
            )
        else:
            self.a_lat = self.parameters.longitudinal.a_max
        if self.maneuver in [
            Maneuver.STEERRIGHT,
            Maneuver.TURNRIGHT,
            Maneuver.OVERTAKERIGHT,
        ]:
            self.a_lat = -self.a_lat

    def sign_change(self):
        self.a_lat = -self.a_lat

    def set_inputs(self, ref_state: PMState) -> None:
        """
        Sets inputs for the lateral simulation
        """
        # set the longitudinal acceleration to 0
        self.set_a_long_and_a_lat(ref_state)
        if self.a_lat > 0:
            self._direction = "left"
        else:
            self._direction = "right"

    def set_bang_bang_timestep_orientation(self, position: np.ndarray):
        """
        Sets the time for the bang–bang controller of the lane change.
        """
        if (
            len(self._scenario.lanelet_network.intersections) == 0
            or Tag.INTERSECTION not in self._scenario.tags
            or self.maneuver not in [Maneuver.TURNLEFT, Maneuver.TURNRIGHT]
        ):
            lanelet_id = self._scenario.lanelet_network.find_lanelet_by_position(
                [position]
            )[0]
            if not lanelet_id:
                return None, None
            lanelet = self._scenario.lanelet_network.find_lanelet_by_id(lanelet_id[0])
        else:
            # intersection scenario
            # use preselected turning lanelet to avoid confusion of find_lanelet_by_position
            # extend predecessor and successors to avoid using orientation[0] and outside of projection
            occupied_lanelet_id = (
                self._scenario.lanelet_network.find_lanelet_by_position([position])[0]
            )
            if len(occupied_lanelet_id) == 0:
                return None, None
            if len(occupied_lanelet_id) == 1:
                lanelet = self._scenario.lanelet_network.find_lanelet_by_id(
                    occupied_lanelet_id[0]
                )
            else:
                lanelet_id = list(self.turning_lanelet_id)

                turning_lanelet = self._scenario.lanelet_network.find_lanelet_by_id(
                    lanelet_id[0]
                )
                pre_lanelet = self._scenario.lanelet_network.find_lanelet_by_id(
                    turning_lanelet.predecessor[0]
                )
                (
                    extend_suc_lanelet,
                    _,
                ) = Lanelet.all_lanelets_by_merging_successors_from_lanelet(
                    turning_lanelet, self._scenario.lanelet_network
                )
                lanelet = Lanelet.merge_lanelets(extend_suc_lanelet[0], pre_lanelet)
        if not lanelet:
            return None, None
        lateral_dis, orientation = compute_lanelet_width_orientation(lanelet, position)
        if self.maneuver in [Maneuver.TURNLEFT, Maneuver.TURNRIGHT] or self.a_lat == 0:
            return math.inf, orientation
        if self._lateral_distance_mode == 1:
            lateral_dis = 0.8
        # Modified from Eq. (11) in Pek, C., Zahn, P. and Althoff, M., Verifying the safety of lane change maneuvers of
        # self-driving vehicles based on formalized traffic rules. In IV 2017 (pp. 1477-1483). IEEE.
        total_timestep = math.sqrt(4 * lateral_dis / abs(self.a_lat))
        return (
            int(total_timestep / (2 * self.dt)) + (total_timestep % (2 * self.dt) > 0),
            orientation,
        )

    def check_intersection_limit(
        self, state_list: List[Union[PMState, KSState]], checked_state: PMState
    ) -> (List[PMState], PMState):
        # directly check whether it is an intersection scenario
        if (
            len(self._scenario.lanelet_network.intersections) == 0
            or Tag.INTERSECTION not in self._scenario.tags
            or self.maneuver not in [Maneuver.TURNLEFT, Maneuver.TURNRIGHT]
        ):
            return state_list, checked_state

        # for turning, finding the target lanelet
        current_lanelet_ids = self._scenario.lanelet_network.find_lanelet_by_position(
            [checked_state.position]
        )[0]
        # store selected incoming and turning lanelet
        self.turning_lanelet_id = None
        self.incoming = None
        for intersection in self._scenario.lanelet_network.intersections:
            for incoming in intersection.incomings:
                is_turn_left = self.maneuver in Maneuver.TURNLEFT
                is_turn_right = self.maneuver in Maneuver.TURNRIGHT

                # Check if the current_lanelet_id is in the correct incoming or successor lanelets based on the maneuver
                if (
                    set(current_lanelet_ids).intersection(incoming.incoming_lanelets)
                    or (
                        is_turn_left
                        and set(current_lanelet_ids).intersection(
                            incoming.successors_left
                        )
                    )
                    or (
                        is_turn_right
                        and set(current_lanelet_ids).intersection(
                            incoming.successors_right
                        )
                    )
                ):
                    self.turning_lanelet_id = (
                        incoming.successors_left
                        if is_turn_left
                        else incoming.successors_right
                    )
                    self.incoming = incoming
                    break

        if self.turning_lanelet_id:
            turning_lanelet = self._scenario.lanelet_network.find_lanelet_by_id(
                list(self.turning_lanelet_id)[0]
            )
            # !! for turning right, the curvatures are negative
            # curvature = np.max(
            #     np.abs(compute_curvature_from_polyline(turning_lanelet.center_vertices))
            # )
            # fixme: idea: assume the turning lane is a part of circle,
            #  calculate the curvature based on the function of chord
            curvature = abs(
                compute_curvature_from_polyline_start_end(
                    turning_lanelet.center_vertices
                )
            )
            # fixme: add rounding up to make the curvature larger
            curvature = np.ceil(curvature * 10) / 10
            desired_velocity = np.sqrt(np.abs(self.a_lat / curvature))
            if (
                np.sqrt(checked_state.velocity**2 + checked_state.velocity_y**2)
                - desired_velocity
                > 0
            ):
                # exceed the limit, decelerate first
                self.a_long = -abs(self.a_lat)
                self.a_lat = 0
            else:
                # too smaller than the limit, accelerate first
                self.a_long = abs(self.a_lat)
                self.a_lat = 0
            while True:
                # set a margin around the desired velocity
                if (
                    np.abs(
                        np.sqrt(checked_state.velocity**2 + checked_state.velocity_y**2)
                        - desired_velocity
                    )
                    < 0.5
                ):
                    break
                check_elements_state(checked_state, self.input)
                self.update_inputs_x_y(checked_state)
                suc_state = self.vehicle_dynamics.simulate_next_state(
                    PMState(
                        position=checked_state.position,
                        velocity=checked_state.velocity,
                        velocity_y=checked_state.velocity_y,
                        time_step=checked_state.time_step,
                    ),
                    self.input,
                    self.dt,
                    throw=False,
                )
                state_list.append(suc_state)
                checked_state = suc_state
            self.set_inputs(checked_state)
            check_elements_state(checked_state, self.input)
            # fixme: assume vehicle will be go straight at a constant velocity until entering the turning lanelet
            while True:
                current_lanelet_ids = (
                    self._scenario.lanelet_network.find_lanelet_by_position(
                        [checked_state.position]
                    )[0]
                )
                if (
                    self.incoming
                    and len(
                        self.incoming.incoming_lanelets.intersection(
                            current_lanelet_ids
                        )
                    )
                    != 0
                ):
                    self.a_long = 0
                    self.a_lat = 0
                    check_elements_state(checked_state, self.input)
                    self.update_inputs_x_y(checked_state)
                    suc_state = self.vehicle_dynamics.simulate_next_state(
                        PMState(
                            position=checked_state.position,
                            velocity=checked_state.velocity,
                            velocity_y=checked_state.velocity_y,
                            time_step=checked_state.time_step,
                        ),
                        self.input,
                        self.dt,
                        throw=False,
                    )
                    state_list.append(suc_state)
                    checked_state = suc_state
                else:
                    break
        self.set_inputs(checked_state)
        check_elements_state(checked_state, self.input)
        return state_list, checked_state

    def simulate_state_list(self, start_time_step: int, given_time_limit: int = None):
        """
        Simulates the lateral state list from the given start time step.
        """
        pre_state = copy.deepcopy(self.simulated_vehicle.state_at_time(start_time_step))
        state_list = self.initialize_state_list(start_time_step)
        # update the input
        check_elements_state(pre_state, self.input)
        self.set_inputs(pre_state)
        state_list.append(pre_state)
        lane_orient = 0.0
        max_orient = 0.0
        if given_time_limit:
            self.time_horizon = given_time_limit
        # check whether it is an intersection, if yes, the vehicle has to reach a desired velocity
        state_list, pre_state = self.check_intersection_limit(state_list, pre_state)
        for i in range(self._nr_stage):
            bang_bang_ts, lane_orient_updated = self.set_bang_bang_timestep_orientation(
                pre_state.position
            )
            if not bang_bang_ts:
                bang_bang_ts = math.inf
                if hasattr(pre_state, "orientation"):
                    max_orient = pre_state.orientation
                else:
                    max_orient = convert_to_0_2pi(
                        math.atan2(pre_state.velocity_y, pre_state.velocity)
                    )
            else:
                lane_orient = lane_orient_updated
                max_orient = convert_to_0_2pi(
                    self.set_maximal_orientation(lane_orient, i)
                )
            if i in [1, 3]:  # 1 for lane change; 1, 3 for overtaking
                self.sign_change()
            bang_state_list = self.bang_bang_simulation(
                pre_state, bang_bang_ts, max_orient
            )
            if bang_state_list:
                state_list += bang_state_list
                pre_state = bang_state_list[-1]
            else:
                break
        # updates the orientation
        while (
            pre_state.time_step < self.time_horizon
        ):  # not <= since the simulation stops at the final step
            _, lane_orient_updated = self.set_bang_bang_timestep_orientation(
                pre_state.position
            )
            if lane_orient_updated:
                lane_orient = lane_orient_updated
                max_orient = self.set_maximal_orientation(
                    lane_orient, 4
                )  # 4 as possible last stage for overtaking
            # only when the orientation is set, the inputs are updated, otherwise the a_lat/a_long is wrong
            self.a_lat = 0
            self.a_long = 0
            check_elements_state(pre_state, self.input)
            self.update_inputs_x_y(pre_state)
            self.adjust_velocity(pre_state, max_orient, lane_orient)
            suc_state = self.vehicle_dynamics.simulate_next_state(
                PMState(
                    position=pre_state.position,
                    velocity=pre_state.velocity,
                    velocity_y=pre_state.velocity_y,
                    time_step=pre_state.time_step,
                ),
                self.input,
                self.dt,
                throw=False,
            )
            state_list.append(suc_state)
            pre_state = suc_state
        check_elements_state(state_list[-1], self.input)
        return state_list

    def adjust_velocity(
        self, ref_state: PMState, max_orient: float, lane_orient: float
    ):
        # using P-controller
        pre_velocity_sum = math.sqrt(ref_state.velocity**2 + ref_state.velocity_y**2)
        if self.maneuver in [Maneuver.TURNLEFT, Maneuver.TURNRIGHT, Maneuver.TURNMC]:
            # drives along the target direction
            target_velocity_x = pre_velocity_sum * math.cos(max_orient)
            target_velocity_y = pre_velocity_sum * math.sin(max_orient)
        else:
            # drives along the lane direction
            target_velocity_x = pre_velocity_sum * math.cos(lane_orient)
            target_velocity_y = pre_velocity_sum * math.sin(lane_orient)
        self.input.acceleration = np.clip(
            (target_velocity_x - ref_state.velocity) / self.dt,
            max(
                ref_state.acceleration + self.cartesian.j_x_min * self.dt,
                self.cartesian.a_x_min,
            ),
            min(
                ref_state.acceleration + self.cartesian.j_x_max * self.dt,
                self.cartesian.a_x_max,
            ),
        )
        self.input.acceleration_y = np.clip(
            (target_velocity_y - ref_state.velocity_y) / self.dt,
            max(
                ref_state.acceleration_y + self.cartesian.j_y_min * self.dt,
                self.cartesian.a_y_min,
            ),
            min(
                ref_state.acceleration_y + self.cartesian.j_y_max * self.dt,
                self.cartesian.a_y_max,
            ),
        )
        self.input.time_step = ref_state.time_step
        ref_state.acceleration = self.input.acceleration
        ref_state.acceleration_y = self.input.acceleration_y

    def set_maximal_orientation(self, lane_orientation, nr_stage):
        """
        adds additional constraints for the orientation.
        """
        if nr_stage >= 4:
            return lane_orientation
        if (
            self.maneuver == Maneuver.STEERLEFT
            or (self.maneuver == Maneuver.OVERTAKELEFT and nr_stage <= 1)
            or (self.maneuver == Maneuver.OVERTAKERIGHT and nr_stage >= 2)
        ):
            return lane_orientation + math.pi / 4
        elif (
            self.maneuver == Maneuver.STEERRIGHT
            or (self.maneuver == Maneuver.OVERTAKELEFT and nr_stage >= 2)
            or (self.maneuver == Maneuver.OVERTAKERIGHT and nr_stage <= 1)
        ):
            return lane_orientation - math.pi / 4
        elif self.maneuver in [Maneuver.LANECHANGEMC, Maneuver.OVERTAKEMC]:
            if self._direction == "left":  # Lane change to the left
                return lane_orientation + math.pi / 4
            else:
                return lane_orientation - math.pi / 4
        elif self.maneuver in [Maneuver.TURNLEFT, Maneuver.TURNRIGHT, Maneuver.TURNMC]:
            if self._direction == "left":  # Lane change to the left
                return lane_orientation + math.pi / 2
            else:
                return lane_orientation - math.pi / 2
        else:
            return lane_orientation

    def bang_bang_simulation(
        self, init_state: PMState, simulation_length: int, max_orientation: float
    ):
        pre_state = init_state
        state_list = []
        while (
            pre_state.time_step < init_state.time_step + simulation_length
            and pre_state.time_step < self.time_horizon
        ):
            self.update_inputs_x_y(pre_state)
            suc_state = self.vehicle_dynamics.simulate_next_state(
                PMState(
                    position=pre_state.position,
                    velocity=pre_state.velocity,
                    velocity_y=pre_state.velocity_y,
                    time_step=pre_state.time_step,
                ),
                self.input,
                self.dt,
                throw=False,
            )
            if suc_state:
                check_elements_state(suc_state, self.input)
                state_list.append(suc_state)
                pre_state = suc_state
                suc_orientation = convert_to_0_2pi(
                    math.atan2(suc_state.velocity_y, suc_state.velocity)
                )
                if self._direction == "left":
                    if (
                        suc_orientation > max_orientation
                        and suc_orientation + math.pi * 2 > max_orientation
                    ):
                        break
                else:
                    if (
                        suc_orientation < max_orientation
                        and suc_orientation - math.pi * 2 < max_orientation
                    ):
                        break
            else:
                self.input.acceleration_y = 0
        return state_list


class SimulationLatMonteCarlo(SimulationLat):
    """
    Simulate the longitudinal trajectory using Monte-Carlo sampling
    """

    def __init__(
        self,
        maneuver: Union[Maneuver],
        simulated_vehicle: DynamicObstacle,
        config: CriMeConfiguration,
    ):
        if maneuver not in [
            Maneuver.LANECHANGEMC,
            Maneuver.TURNMC,
            Maneuver.OVERTAKEMC,
        ]:
            raise ValueError(
                f"<Criticality/Simulation>: provided maneuver {maneuver} is not supported or goes to the wrong category"
            )
        super(SimulationLatMonteCarlo, self).__init__(
            maneuver, simulated_vehicle, config
        )
        self.pdf = 1  # probability density function

    def set_inputs(self, ref_state: PMState) -> None:
        self.set_a_long_and_a_lat(ref_state)
        a_lat_norm = norm(0, self.a_lat)
        self.a_lat = np.random.normal(0, abs(self.a_lat), 1)[0]
        if self.a_lat > 0:
            self._direction = "left"
        else:
            self._direction = "right"
        self.pdf = a_lat_norm.pdf(self.a_lat)
