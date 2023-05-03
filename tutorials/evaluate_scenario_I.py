"""
See paper: CommonRoad-CriMe: A Toolbox for Criticality Measures of Autonomous Vehicles

Experiment: Sec. IV.B Evaluation on Scenarios
"""


from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface

from commonroad_crime.measure import (TTC, BTN, PF, ALongReq, ALatReq, STN, P_MC)
from commonroad_crime.measure.reachable_set.drivable_area import DA

import commonroad_crime.utility.visualization as utils_vis


def main():
    # scenario I
    scenario_id = "DEU_Gar-1_1_T-1"

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # ==== compute the criticality using CriMe interface
    crime_interface = CriMeInterface(config)
    crime_interface.evaluate_scenario([TTC, ALongReq, BTN, ALatReq, DA, STN, P_MC, PF],
                                      time_start=0, time_end=20)

    # # ==== visualize the result
    utils_vis.plot_criticality_curve(crime_interface)


if __name__ == "__main__":
    main()
