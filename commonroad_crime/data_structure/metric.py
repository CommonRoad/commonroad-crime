from enum import Enum


class TimeScaleMetricType(str, Enum):
    TTC = "time-to-collision"
    TTR = "time-to-react"
    TTB = "time-to-brake"
    TTM = "time-to-maneuver"
    TTK = "time-to-kickdown"
    TTS = "time-to-steer"
    THW = "time-headway"
    WTTC = "worst-time-to-collision"
    WTTR = "worst-time-to-react"


class DistanceScaleMetricType(str, Enum):
    THW = "headway time"


class ReachableSetScaleMetricType(str, Enum):
    DA = "drivable area"
