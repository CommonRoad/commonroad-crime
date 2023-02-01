__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

"""
Types of different criticality measures categorized by the output, see https://criticality-metrics.readthedocs.io/
"""

from enum import Enum


class TypeMonotone(str, Enum):
    """
    Monotonic relationship between the metric value and the criticality
    """
    POS = "positive monotonic"
    NEG = "negative monotonic"


class TypeNone(str, Enum):
    NONE = "not defined"


class TypeTime(str, Enum):
    THW = "time headway"
    ET = "encroachment time"
    PET = "post encroachment time"
    AGS = "accepted gap size"
    TTC = "time-to-collision"
    TTCStar = "time-to-collision with given prediction"
    PTTC = "potential time to collision"
    WTTC = "worst-time-to-collision"
    TET = "time-exposed time-to-collision"
    TIT = "time-integrated time-to-collision"
    TTCE = "time to closest encounter"
    TTZ = "time-to-zabra"
    TTB = "time-to-brake"
    TTK = "time-to-kickdown"
    TTS = "time-to-steer"
    TTM = "time-to-maneuver"
    TTR = "time-to-react"
    WTTR = "worst-time-to-react"
    TV = "time-to-violation"
    TC = "time-to-Compliance"


class TypeDistance(str, Enum):
    HW = "headway"
    DCE = "distance of closest encounter"
    MSD = "Acceptable minimum stopping distance"
    PSD = "proportion of stopping distance"


class TypeVelocity(str, Enum):
    Delta_V = "Delta-v"
    CS = "conflict severity"


class TypeAcceleration(str, Enum):
    DST = "deceleration to safety time"
    ALongReq = "required longitudinal acceleration"
    ALatReq = "required lateral acceleration"
    AReq = "required acceleration"


class TypeJerk(str, Enum):
    LatJ = "lateral jerk"
    LongJ = "longitudinal jerk"


class TypeIndex(str, Enum):
    CI = "conflict index"
    CPI = "crash potential index"
    ACI = "aggregated crash index"
    TCI = "trajectory criticality index"
    PRI = "pedestrian risk index"
    SOI = "space occupancy index"
    BTN = "brake threat number"
    STN = "steer threat number"
    RSS = "responsibility sensitive safety dangerous situation"


class TypeReachableSet(str, Enum):
    DA = "drivable area"


class TypeProbability(str, Enum):
    P_MC = "collision probability via Monte Carlo"
    P_SMH = "collision probability via scoring multiple hypotheses"
    P_SRS = "collision probability via stochastic reachable sets"


class TypePotential(str, Enum):
    PF = "potential functions as superposition of scoring functions"
    SP = "safety potential"

