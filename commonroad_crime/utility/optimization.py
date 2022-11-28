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
from scipy.spatial.distance import cdist

import numpy as np
from commonroad.scenario.scenario import State, Scenario, DynamicObstacle
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.scene import Scene
import commonroad_crime.utility.solver as utils_sol


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
        # x_w, y_w, v_v, psi
        self._opt_states = self.opti.variable(self.tci_config.N + 1, 4)

        # a_x, a_y
        self._opt_controls = self.opti.variable(self.tci_config.N, 2)

        self.dt = sce.dt
        self.sce = sce

    def vehicle_model(self, ref_state: State):
        # initializing the model using the velocity in the reference path
        v_old = math.sqrt(ref_state.velocity ** 2 + ref_state.velocity_y ** 2)
        return lambda x_, u_: ca.vertcat(*[x_[2],
                                           x_[3] * v_old,
                                           u_[0],
                                           u_[1] / v_old])

    def constraints(self, x_initial: State, ref_state_list: List[State], boundary_limit_list: List[List]):
        # initial condition
        ini_x = np.array([[x_initial.position[0]],
                          [x_initial.position[1]],
                          [math.sqrt(x_initial.velocity ** 2 + x_initial.velocity_y ** 2)],
                          [x_initial.orientation]]).T
        self.opti.subject_to(self._opt_states[0, :] == ini_x)
        for k in range(self.tci_config.N):
            x_next = self._opt_states[k, :] + self.vehicle_model(ref_state_list[k])(self._opt_states[k, :],
                                                                                    self._opt_controls[k,
                                                                                    :]).T * self.dt
            self.opti.subject_to(self._opt_states[k + 1, :] == x_next)
            self.opti.subject_to(self._opt_controls[k, 0] ** 2 + self._opt_controls[k, 1] ** 2 <=
                                 self.veh_config.cartesian.longitudinal.a_max ** 2)
            # road boundary
            self.opti.subject_to(self.opti.bounded(boundary_limit_list[k][0],
                                                   self._opt_states[k, 1],
                                                   boundary_limit_list[k][1]))
        # todo: three circle approximation

    def cost_function(self, x_initial: State, ref_state_list: List[State],
                      d_y: float, r_y: float, d_x: float):
        obj = 0.
        for k in range(x_initial.time_step,
                       min(x_initial.time_step + self.tci_config.N + 1, len(ref_state_list))):
            if d_x is not math.inf and d_x and d_y:
                v_old = math.sqrt(ref_state_list[k].velocity ** 2 + ref_state_list[k].velocity_y ** 2)
                obj += self.tci_config.w_y * (self._opt_states[k, 1] - r_y) ** 2 * v_old / \
                       (d_y ** 2 * self.veh_config.cartesian.longitudinal.v_max) + \
                       self.tci_config.w_x * (ca.fmax(0, self._opt_states[k, 0] - 0.5 * self._opt_states[k, 2])) / d_x
            if k != x_initial.time_step + self.tci_config.N:
                obj += self.tci_config.w_ax * \
                       self._opt_controls[k, 0] ** 2 / self.veh_config.cartesian.longitudinal.a_max ** 2 + \
                       self.tci_config.w_ay * \
                       self._opt_controls[k, 1] ** 2 / self.veh_config.cartesian.longitudinal.a_max ** 2
        return obj

    def compute_params(self, vehicle: DynamicObstacle, time_step: int):
        """
        Computes the state with the maximum lateral distance to all obstacles
        """
        d_x_list = []
        d_y_list = []
        r_y_list = []
        boundary_limit_list = []
        for obs in self.sce.obstacles:
            if obs is not vehicle:
                for k in range(time_step, time_step + self.tci_config.N + 1):
                    if vehicle.state_at_time(k):
                        dis_bound = utils_sol.compute_veh_dis_to_boundary(vehicle.state_at_time(k),
                                                                          self.sce.lanelet_network)
                        d_y = max(dis_bound)
                        boundary_limit_list.append([obs.state_at_time(k).position[1] - dis_bound[1],
                                                    obs.state_at_time(k).position[1] + dis_bound[0]])
                        d_x = math.inf
                        if obs.state_at_time(k):
                            dis_other = abs(vehicle.state_at_time(k).position[1] - obs.state_at_time(k).position[1])
                            d_y = max(dis_other, d_y)
                            d_x = abs(vehicle.state_at_time(k).position[0] - obs.state_at_time(k).position[0])
                        d_y_list.append(d_y)
                        d_x_list.append(d_x)
                        r_y_list.append(vehicle.state_at_time(k).position[1])
        idx = np.argmax(d_y_list)
        return r_y_list[idx], d_y_list[idx], d_x_list[idx], boundary_limit_list

    def optimize(self,
                 ego_vehicle: DynamicObstacle,
                 time_step: int) -> ca.OptiSol:
        r_y, d_y, d_x, boundary_limit_list = self.compute_params(ego_vehicle, time_step)
        obj = self.cost_function(ego_vehicle.state_at_time(time_step),
                                 ego_vehicle.prediction.trajectory.state_list, d_y, r_y, d_x)
        self.constraints(ego_vehicle.initial_state, ego_vehicle.prediction.trajectory.state_list, boundary_limit_list)
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
