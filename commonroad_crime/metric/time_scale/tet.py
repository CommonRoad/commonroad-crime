import numpy as np

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.visualization as utils_vis
import matplotlib.pyplot as plt


class TET(CriMeBase):

    metric_name = TypeTimeScale.TET

    def __init__(self, config: CriMeConfiguration):
        super(TET, self).__init__(config)

    def compute(self, time_step: int = 0):
        """
        Iterate through all states, calculate ttc, compare it to tau and then add dt to the result
        if ttc is smaller than tau
        """
        # init
        tau = self.configuration.time_scale.tau
        state_list = self.ego_vehicle.prediction.trajectory.state_list

        # all time steps in array
        array = np.arange(time_step, len(state_list), 1)
        # vectorize ttc
        ttc_compute_vec = np.vectorize(self.ttc_object.compute)
        # call ttc for each element
        array = ttc_compute_vec(array)
        # compare each element to tau
        array = np.less_equal(array, np.full(len(array), tau))
        # sum all true values, multiply by dt
        self.value = np.sum(array) * self.dt
        return self.value

    def visualize(self):
        pass
        """
        self._initialize_vis(plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                              self.ego_vehicle.prediction.
                                                                              trajectory.state_list,
                                                                              margin=10))
        self.other_vehicle.draw(self.rnd, {'time_begin': self.time_step, **utils_vis.OTHER_VEHICLE_DRAW_PARAMS})
        self.rnd.render()
        plt.title(f"{self.metric_name} at time step {self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()"""
