import numpy as np

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.visualization as utils_vis
import matplotlib.pyplot as plt

from commonroad_crime.metric.time_scale.ttc import TTC


class TIT(CriMeBase):

    metric_name = TypeTimeScale.TET

    def __init__(self, config: CriMeConfiguration):
        super(TIT, self).__init__(config)
        self.ttc_object = TTC(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        """
        Iterate through all states, calculate ttc, compare it to tau and then add dt to the result
        if ttc is smaller than tau
        """
        # init
        tau = self.configuration.time_scale.tau
        state_list = self.ego_vehicle.prediction.trajectory.state_list

        """# all time steps in array
        array = np.arange(time_step, len(state_list), 1)
        # tau-array
        tau_array = np.full(len(array), tau)
        # vectorize ttc
        ttc_compute_vec = np.vectorize(self.ttc_object.compute)
        # call ttc for each element
        array = ttc_compute_vec(vehicle_id, array)
        # compare each element to tau
        id_array = np.less_equal(array, tau_array)
        # subtract ttc from tau
        array = np.subtract(tau_array, array)
        # multiply indicator function with array
        array = np.multiply(id_array, array)
        # sum all array values, multiply by dt
        self.value = np.sum(array) * self.dt"""
        self.value = 0
        for i in range(time_step, len(state_list)):
            ttc_result = self.ttc_object.compute(vehicle_id, i)
            if ttc_result <= tau:
                self.value += (tau - ttc_result) * self.dt
        return self.value

    def visualize(self):
        pass
