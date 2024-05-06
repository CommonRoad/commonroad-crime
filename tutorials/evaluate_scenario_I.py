"""
See paper: CommonRoad-CriMe: A Toolbox for Criticality Measures of Autonomous Vehicles

Experiment: Sec. IV.B Evaluation on Scenarios
"""
from pathlib import Path

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.crime_interface import CriMeInterface

from commonroad_crime.measure import TTC, BTN, PF, ALongReq, ALatReq, STN, P_MC, DA

import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.logger as utils_log


def main():
    # scenario I
    scenario_id = "DEU_Gar-1_1_T-1"

    # ==== build configuration
    config = CriMeConfiguration.load(f"../config_files/{scenario_id}.yaml", scenario_id)
    config.update()
    utils_log.initialize_logger(config)
    config.print_configuration_summary()

    # ==== compute the criticality using CriMe interface
    crime_interface = CriMeInterface(config)
    crime_interface.evaluate_scenario(
        [TTC, ALongReq, BTN, ALatReq, STN, P_MC, PF, DA],
        time_start=0,
        time_end=20,
        verbose=True,
    )

    # # ==== visualize the result
    # utils_vis.plot_criticality_curve(crime_interface)

    # # ==== save data to file
    # output_path = Path.cwd().joinpath("..", "output").absolute()
    # crime_interface.safe_to_file(str(output_path))


if __name__ == "__main__":
    main()
