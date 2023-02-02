__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime
from commonroad_crime.measure.time.ttm import TTM
from commonroad_crime.utility.simulation import Maneuver


class TTB(TTM):
    measure_name = TypeTime.TTB
    
    def __init__(self, config: CriMeConfiguration):
        super(TTB, self).__init__(config, Maneuver.BRAKE)




