from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface

from commonroad_crime.measure import (TTC, TTCStar, TTB, TTS, TTK, TTR, THW, WTTC, ET, PET, BTN, PF, HW, TTCE,
                                      ALongReq, ALatReq, STN, P_MC, LongJ, LatJ, DeltaV)
from commonroad_crime.measure.time.wttr import WTTR
from commonroad_crime.measure.reachable_set.drivable_area import DA


def main():
    # scenario I
    scenario_id = "DEU_Gar-1_1_T-1"
    # scenario II
    # scenario_id = 'ZAM_Urban-7_1_S-2'

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # ==== compute the criticality using CriMe interface
    crime_interface = CriMeInterface(config)
    crime_interface.evaluate_scene([HW, THW, ET, PET, TTC,
                                    WTTC, TTCStar, TTCE, TTS, TTK, TTB,
                                    TTR, WTTR, ALongReq, ALatReq, LongJ, LatJ,
                                    DeltaV, BTN, STN, DA, P_MC, PF], time_step=0)


if __name__ == "__main__":
    main()
