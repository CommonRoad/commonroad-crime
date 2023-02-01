import numpy as np

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.visualization as utils_vis
import matplotlib.pyplot as plt

from commonroad_crime.measure.time_scale.ttc import TTC


class TIT(CriMeBase):

    measure_name = TypeTimeScale.TET

    def __init__(self, config: CriMeConfiguration):
        super(TIT, self).__init__(config)
        self.ttc_object = TTC(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        """
        Iterate through all states, calculate ttc, compare it to tau and then add dt to the result
        if ttc is smaller than tau
        """
        # init
        tau = self.configuration.time.tau
        state_list = self.ego_vehicle.prediction.trajectory.state_list

        self.value = 0
        for i in range(time_step, len(state_list)):
            ttc_result = self.ttc_object.compute(vehicle_id, i)
            if ttc_result <= tau:
                self.value += (tau - ttc_result) * self.dt
        return self.value

    def visualize(self):
        pass
