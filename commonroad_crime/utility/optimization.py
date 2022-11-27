__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math

import casadi as ca
from typing import List, Union
from abc import abstractmethod

import numpy as np
from commonroad.scenario.scenario import State, Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.lanelet import Lanelet
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.scene import Scene


class OptimizerBase:
    def __init__(self, config: CriMeConfiguration):
        self.config = config

    @abstractmethod
    def vehicle_model(self, *args, **kwargs):
        pass

    @abstractmethod
    def cost_function(self, *args, **kwargs):
        pass

    @abstractmethod
    def constraints(self, *args, **kwargs):
        pass

    @abstractmethod
    def optimize(self, *args, **kwargs):
        pass


class TCIOptimizer(OptimizerBase):
    def __init__(self, config: CriMeConfiguration, sce: Union[Scenario, Scene]):
        super(TCIOptimizer, self).__init__(config)
        self.tci_config = self.config.index_scale.tci
        self.veh_config = self.config.vehicle

        self.opti = ca.Opti()
        # define state and input variables
        self._opt_states = self.opti.variable(self.tci_config.N + 1, 4)
        self._x_w = self._opt_states[:, 0]
        self._y_w = self._opt_states[:, 1]
        self._v_v = self._opt_states[:, 2]
        self._psi = self._opt_states[:, 3]

        self._opt_controls = self.opti.variable(self.tci_config.N, 2)
        self._a_x = self._opt_controls[:, 0]
        self._a_y = self._opt_controls[:, 1]

        # # parameters, these parameters are the reference trajectories of the states
        # self.opt_x_ref = self.opti.parameter(self.tci_config.N + 1, 4)

        self.dt = sce.dt
        self.sce = sce

    def vehicle_model(self, ref_state: State):
        # initializing the model using the velocity in the reference path
        v_old = math.sqrt(ref_state.velocity ** 2 + ref_state.velocity_y ** 2)
        return lambda x_, u_: ca.vertcat(*[x_[2],
                                           x_[3] * v_old,
                                           u_[0],
                                           u_[1] / v_old])

    def constraints(self, x_initial: State, ref_state_list: List[State]):
        # initial condition
        ini_x = np.array([[x_initial.position[0]],
                          [x_initial.position[1]],
                          [math.sqrt(x_initial.velocity**2 + x_initial.velocity_y**2)],
                          [x_initial.orientation]]).T
        self.opti.subject_to(self._opt_states[0, :] == ini_x)
        for k in range(self.tci_config.N):
            x_next = self._opt_states[k, :] + self.vehicle_model(ref_state_list[k])(self._opt_states[k, :],
                                                                                    self._opt_controls[k,
                                                                                    :]).T * self.dt
            self.opti.subject_to(self._opt_states[k + 1, :] == x_next)
            self.opti.subject_to(self._opt_controls[k, 0]**2 + self._opt_controls[k, 1]**2 <=
                                 self.veh_config.cartesian.longitudinal.a_max**2)
            # todo: add road boundary

    def cost_function(self, x_initial: State, ref_state_list: List[State], d_y: float, r_y: float, d_x: float):
        obj = 0.
        for k in range(min(self.tci_config.N, len(ref_state_list))):
            v_old = math.sqrt(ref_state_list[k].velocity ** 2 + ref_state_list[k].velocity_y ** 2)
            obj += self.tci_config.w_y * (self._opt_states[k, 1] - r_y) ** 2 * v_old / (d_y ** 2 * self.veh_config.
                                                                                        cartesian.longitudinal.v_max) + \
                self.tci_config.w_x * (ca.fmax(0, self._opt_states[k, 0] - 0.5 * self._opt_states[k, 2])) / d_x + \
                self.tci_config.w_ax * self._opt_controls[k, 0] ** 2/self.veh_config.cartesian.longitudinal.a_max**2 + \
                self.tci_config.w_ay * self._opt_controls[k, 1] ** 2/self.veh_config.cartesian.longitudinal.a_max**2
        return obj

    def optimize(self,
                 ego_vehicle,
                 time_step: int) ->  ca.OptiSol:
        # find the state with the maximum lateral distance to all obstacles
        d_y = 0.
        d_x = 0.
        r_y = 0.
        for obs in self.sce.obstacles:
            if obs is not ego_vehicle:
                for k in range(self.tci_config.N + 1):
                    if ego_vehicle.state_at_time(k) and obs.state_at_time(k):
                        r_y = abs(ego_vehicle.state_at_time(k).position[1] - obs.state_at_time(k).position[1])
                        if r_y > d_y:
                            d_y = r_y
                            d_x = abs(ego_vehicle.state_at_time(k).position[0] - obs.state_at_time(k).position[0])
                            r_y = ego_vehicle.state_at_time(k).position[1]
        obj = self.cost_function(ego_vehicle.initial_state, ego_vehicle.prediction.trajectory.state_list, d_y, r_y, d_x)
        self.constraints(ego_vehicle.initial_state, ego_vehicle.prediction.trajectory.state_list)
        self.opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0,
                        'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6, }
        self.opti.solver('ipopt', opts_setting)
        sol = self.opti.solve()
        return sol

    def convert_result_to_cr_trajectory(self, sol: ca.OptiSol):
        """
        Converts the current result to the CommonRoad trajectory
        """
        state_list = []
        opt_x = sol.value(self._opt_states)
        for k in range(self.tci_config.N):
            kwarg = {
                'position': np.array([opt_x[k, 0], opt_x[k, 1]]),
                'velocity': opt_x[k, 2],
                'orientation': opt_x[k, 3],
                'time_step': k
            }
            state_list.append(State(**kwarg))
        return Trajectory(0, state_list)

