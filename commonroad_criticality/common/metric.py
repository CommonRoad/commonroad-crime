from enum import Enum


class TimeMetricType(Enum):
    TTC = "time-to-collision"
    TTR = "time-to-react"
    WTTC = "worst-time-to-collision"
    WTTR = "worst-time-to-react"


class DistanceMetricType(Enum):
    THW = "headway time"
