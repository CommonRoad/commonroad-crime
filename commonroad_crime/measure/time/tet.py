__author__ = "Oliver Specht, Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone

from commonroad_crime.measure.time.ttc import TTC


class TET(CriMeBase):
    measure_name = TypeTime.TET
    monotone = TypeMonotone.POS

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

        self.value = 0
        for i in range(time_step, len(state_list)):
            if self.ttc_object.compute(vehicle_id, i) <= tau:
                self.value += self.dt
        return self.value

    def visualize(self):
        pass
