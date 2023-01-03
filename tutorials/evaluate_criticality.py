
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface
from commonroad_crime.metric.time_scale import TTC, TTCStar, TTB, TTS, TTK, TTR, THW, WTTC
from commonroad_crime.metric.distance_scale import HW
from commonroad_crime.metric.time_scale.wttr import WTTR
from commonroad_crime.metric.reachable_set_scale.drivable_area import DA
from commonroad_crime.metric.acceleration_scale import ALatReq, ALongReq
from commonroad_crime.metric.jerk_scale import LatJ, LongJ
from commonroad_crime.metric.index_scale import BTN, STN
from commonroad_crime.metric.probability_scale import P_MC
from commonroad_crime.metric.potential_scale import PF
from commonroad_crime.metric.velocity_scale import DeltaV


def main():

    scenario_id = 'ZAM_Urban-7_1_S-2'
    # scenario_id = "DEU_Gar-1_1_T-1"

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # ==== compute the criticality using CriMe interface
    crime_interface = CriMeInterface(config)
    # crime_interface.evaluate([THW])
    # crime_interface.evaluate([HW, THW, TTC, WTTC, TTCStar, TTS, TTK, TTB, TTR, ])
    # crime_interface.evaluate([WTTR, DA])
    # crime_interface.evaluate([ALatReq, ALongReq, LongJ, LatJ, BTN, STN])
    # crime_interface.evaluate([P_MC, PF])
    crime_interface.evaluate([DeltaV])
    # ==== select the criticality metric you want to evaluate and then compute the value at a given time step

    # # ==== visualize the result
    # config.debug.save_plots = False


if __name__ == "__main__":
    main()

