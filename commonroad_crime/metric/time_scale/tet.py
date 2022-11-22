from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.visualization as utils_vis
import matplotlib.pyplot as plt


class TET(CriMeBase):

    metric_name = TypeTimeScale.TET

    def __init__(self, config: CriMeConfiguration):
        super(TET, self).__init__(config)

    def compute(self, time_step: int = 0, tau: int = 0):
        self.value = 0;
        state_list = self.ego_vehicle.prediction.trajectory.state_list
        for i in range(time_step, len(state_list)):
            if self.ttc_object.compute(time_step) <= tau:
                self.value += self.dt
        return self.value

    def visualize(self):
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
            plt.show()
