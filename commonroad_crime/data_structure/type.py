from enum import Enum

# see https://criticality-metrics.readthedocs.io/


class TypeTimeScale(str, Enum):
    TTC = "time-to-collision"
    TTR = "time-to-react"
    TTB = "time-to-brake"
    TTM = "time-to-maneuver"
    TTK = "time-to-kickdown"
    TTS = "time-to-steer"
    THW = "time headway"
    WTTC = "worst-time-to-collision"
    WTTR = "worst-time-to-react"
    # not implemented yet
    ET = "encroachment time"
    PET = "post encroachment time"
    PTTC = "potential time to collision"
    PrET = "predictive encroachment time"
    TET = "time exposed time-to-collision"
    TIT = "time integrated time-to-collision"
    TTCE = "time to closest encounter"
    TTZ = "time-to-zabra"


class TypeDistanceScale(str, Enum):
    THW = "headway time"
    AGS = "accepted gap size"
    DCE = "distance of closest encounter"
    PSE = "proportion of stopping distance"


class TypeVelocityScale(str, Enum):
    CS = "conflict severity"
    Delta_V = "Delta-v"


class TypeAccelerationScale(str, Enum):
    DST = "deceleration to safety time"
    ALongReq = "required longitudinal acceleration"
    ALatReq = "required lateral acceleration"


class TypeJerkScale(str, Enum):
    LatJ = "lateral jerk"
    LongJ = "longitudinal jerk"


class TypeIndexScale(str, Enum):
    ACI = "aggregated Crash Index"
    AM = "accident metric"
    CI = "conflict index"
    CPI = "crash potential index"
    PRI = "pedestrian risk index"
    RSS_DS = "responsibility sensitive safety dangerous situation"
    SOI = "space occupancy index"
    STN = "steer threat number"
    TCI = "trajectory criticality index"


class TypeProbabilityScale(str, Enum):
    P_MC = "collision probability via Monte Carlo"
    P_SMH = "collision probability via scoring multiple hypotheses"
    P_SRS = "collision probability via stochastic reachable sets"


class TypePotentialScale(str, Enum):
    PF = "potential functions as superposition of scoring functions"
    SF = "safety potential"
    

class TypeReachableSetScale(str, Enum):
    DA = "drivable area"
