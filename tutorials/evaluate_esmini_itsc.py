from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.measure import (
    TTC,
    TTCStar,
    WTTC,
    HW,
    BTN,
    DeltaV,
    STN,
    TTR,
    TTB,
    TTK,
)
from commonroad_crime.measure.reachable_set.drivable_area import DA
from commonroad_crime.data_structure.crime_interface import CriMeInterface


def main():
    scenario_id = "OSC_PedestrianCollision-1_1_T-38"
    # ==== build configuration
    config = CriMeConfiguration.load(f"../config_files/{scenario_id}.yaml", scenario_id)
    config.update()
    config.print_configuration_summary()

    crime_interface = CriMeInterface(config)
    # STN: not so good since the last value is too large
    # crime_interface.evaluate_scenario([TTC, WTTC, TTR, DA], time_start=26, time_end=56)
    # crime_interface.visualize(time_step=56)
    crime_interface.evaluate_scene([TTB], time_step=0)
    config.debug.save_plots = False
    crime_interface.visualize(time_step=0)
    # ==== visualize the scenario at given time steps
    # utils_vis.plot_criticality_curve(crime_interface, 4)
    # utils_vis.visualize_scenario_at_time_steps(config.scenario,
    #                                            plot_limit=config.debug.plot_limits,
    #                                            time_steps=[26, 37, 54, 56])
    # utils_vis.save_fig('sce', config.general.path_output, 26)


if __name__ == "__main__":
    main()
