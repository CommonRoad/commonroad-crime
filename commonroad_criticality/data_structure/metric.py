from enum import Enum


class TimeScaleMetricType(str, Enum):
    TTC = "time-to-collision"
    TTR = "time-to-react"
    TTB = "time-to-brake"
    TTM = "time-to-maneuver"
    TTK = "time-to-kickdown"
    WTTC = "worst-time-to-collision"
    WTTR = "worst-time-to-react"


class DistanceMetricType(str, Enum):
    THW = "headway time"
