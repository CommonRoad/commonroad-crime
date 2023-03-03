__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math

import casadi as ca
from typing import List, Union, Tuple
from abc import abstractmethod

import numpy as np
from commonroad.scenario.state import PMState, CustomState, InitialState
from commonroad.scenario.scenario import Scenario, DynamicObstacle
from commonroad.scenario.trajectory import Trajectory
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
        self.tci_config = self.config.index.tci
        self.veh_config = self.config.vehicle

        self.opti = ca.Opti()
        # define state and input variables
        # x_w, y_w, v_v, psi
        self._opt_states = self.opti.variable(self.tci_config.N + 1, 4)

        # a_x, a_y
        self._opt_controls = self.opti.variable(self.tci_config.N, 2)

        self.dt = sce.dt
        self.sce = sce

    def vehicle_model(self, ref_state: PMState):
        # initializing the model using the velocity in the reference path
        v_old = math.sqrt(ref_state.velocity ** 2 + ref_state.velocity_y ** 2)
        return lambda x_, u_: ca.vertcat(*[x_[2],
                                           x_[3] * v_old,
                                           u_[0],
                                           u_[1] / v_old])

    def constraints(self, ego_veh: DynamicObstacle,
                    boundary_limit_list: List[List]):
        x_initial = ego_veh.initial_state
        ref_state_list = ego_veh.prediction.trajectory.state_list
        # initial condition
        ini_x = np.array([[x_initial.position[0]],
                          [x_initial.position[1]],
                          [math.sqrt(x_initial.velocity ** 2 + x_initial.velocity_y ** 2)],
                          [x_initial.orientation]]).T
        self.opti.subject_to(self._opt_states[0, :] == ini_x)
        # three circle approximation
        rad_ego, dis_ego = utils_sol.compute_disc_radius_and_distance(ego_veh.obstacle_shape.length,
                                                                      ego_veh.obstacle_shape.width)
        for k in range(self.tci_config.N):
            x_next = self._opt_states[k, :] + self.vehicle_model(ref_state_list[k])(self._opt_states[k, :],
                                                                                    self._opt_controls[k,
                                                                                    :]).T * self.dt
            self.opti.subject_to(self._opt_states[k + 1, :] == x_next)
            self.opti.subject_to(self._opt_controls[k, 0] ** 2 + self._opt_controls[k, 1] ** 2 <=
                                 self.veh_config.cartesian.longitudinal.a_max ** 2)
            # road boundary + ego's shape
            self.opti.subject_to(self.opti.bounded(boundary_limit_list[k][0] + rad_ego,
                                                   self._opt_states[k, 1],
                                                   boundary_limit_list[k][1] - rad_ego))
            for obs in self.sce.obstacles:
                if obs is not ego_veh:
                    rad_obs, dis_obs = utils_sol.compute_disc_radius_and_distance(obs.obstacle_shape.length,
                                                                                  obs.obstacle_shape.width)
                    # distance between vehicles
                    if obs.state_at_time(k):
                        obs_state = obs.state_at_time(k)
                        for i in range(0, 3):
                            for j in range(0, 3):
                                self.opti.subject_to(ca.sqrt(
                                    (self._opt_states[k, 0] + (i - 1) * dis_ego / 2 * ca.cos(self._opt_states[k, 3]) -
                                     obs_state.position[0] - (j - 1) * dis_obs / 2 * ca.cos(
                                                obs_state.orientation)) ** 2 +
                                    (self._opt_states[k, 1] + (i - 1) * dis_ego / 2 * ca.sin(self._opt_states[k, 3]) -
                                     obs_state.position[1] - (j - 1) * dis_obs / 2 * ca.sin(
                                                obs_state.orientation)) ** 2) >= rad_obs + rad_ego)

    def cost_function(self, x_initial: InitialState, ref_state_list: List[PMState],
                      d_y: float, r_y: float, d_x: float):
        obj = 0.
        for k in range(x_initial.time_step,
                       min(x_initial.time_step + self.tci_config.N + 1, len(ref_state_list))):
            v_old = math.sqrt(ref_state_list[k].velocity ** 2 + ref_state_list[k].velocity_y ** 2)
            if d_x:
                obj += self.tci_config.w_x * (ca.fmax(0, self._opt_states[k, 0] - 0.5 * self._opt_states[k, 2])) / d_x
            if d_y:
                obj += self.tci_config.w_y * (self._opt_states[k, 1] - r_y) ** 2 * v_old / \
                       (d_y ** 2 * self.veh_config.cartesian.longitudinal.v_max)
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
        boundary_limit_list = []
        r_y = vehicle.state_at_time(time_step).position[1]
        d_x = None
        d_y = None
        for k in range(time_step, time_step + self.tci_config.N + 1):
            if vehicle.state_at_time(k):
                dis_bound = utils_sol.compute_veh_dis_to_boundary(vehicle.state_at_time(k),
                                                                  self.sce.lanelet_network)
                d_y = max(dis_bound)
                boundary_limit_list.append([vehicle.state_at_time(k).position[1] - dis_bound[0],
                                            vehicle.state_at_time(k).position[1] + dis_bound[1]])
                for obs in self.sce.obstacles:
                    if obs is not vehicle:
                        if obs.state_at_time(k):
                            dis_other = abs(vehicle.state_at_time(k).position[1] - obs.state_at_time(k).position[1])
                            if dis_other > d_y:
                                d_y = dis_other
                                d_x = abs(vehicle.state_at_time(k).position[0] - obs.state_at_time(k).position[0])
                                r_y = vehicle.state_at_time(k).position[1]
        return r_y, d_y, d_x, boundary_limit_list

    def optimize(self,
                 ego_vehicle: DynamicObstacle,
                 time_step: int) -> Tuple[ca.OptiSol, float]:
        r_y, d_y, d_x, boundary_limit_list = self.compute_params(ego_vehicle, time_step)
        obj = self.cost_function(ego_vehicle.state_at_time(time_step),
                                 ego_vehicle.prediction.trajectory.state_list, d_y, r_y, d_x)
        self.constraints(ego_vehicle, boundary_limit_list)
        self.opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 1000, 'ipopt.print_level': 0, 'print_time': 0,
                        'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6, }
        self.opti.solver('ipopt', opts_setting)
        sol = self.opti.solve()
        return sol, self.opti.value(obj)

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
            state_list.append(CustomState(**kwarg))
        return Trajectory(0, state_list)
