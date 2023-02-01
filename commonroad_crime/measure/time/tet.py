import numpy as np

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime
import commonroad_crime.utility.visualization as utils_vis
import matplotlib.pyplot as plt

from commonroad_crime.measure.time.ttc import TTC


class TET(CriMeBase):

    measure_name = TypeTime.TET

    def __init__(self, config: CriMeConfiguration):
        super(TET, self).__init__(config)
        self.ttc_object = TTC(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        """
        Iterate through all states, calculate ttc, compare it to tau and then add dt to the result
        if ttc is smaller than tau
        """
        # init
        self.time_step = time_step
        tau = self.configuration.time.tau
        state_list = self.ego_vehicle.prediction.trajectory.state_list

        """# all time steps in array
        array = np.arange(self.time_step, len(state_list), 1).astype(int)
        # vectorize ttc
        ttc_compute_vec = np.vectorize(self.ttc_object.compute)
        # call ttc for each element
        array = ttc_compute_vec(vehicle_id, array.astype(int))
        # compare each element to tau
        array = np.less_equal(array, np.full(len(array), tau))
        # sum all true values, multiply by dt
        self.value = np.sum(array) * self.dt"""
        self.value = 0
        for i in range(time_step, len(state_list)):
            if self.ttc_object.compute(vehicle_id, i) <= tau:
                self.value += self.dt
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
        plt.title(f"{self.measure_name} at time step {self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()"""
