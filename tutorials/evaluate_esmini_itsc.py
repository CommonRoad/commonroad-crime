from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.measure import (TTC, TTCStar)
from commonroad_crime.data_structure.crime_interface import CriMeInterface


def main():

    scenario_id = 'OSC_PedestrianCollision-1_1_T-1'
    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    crime_interface = CriMeInterface(config)
    crime_interface.evaluate_scene([TTCStar],
                                time_step=12)
    crime_interface.visualize(time_step=12)
    # ==== visualize the scenario at given time steps
    utils_vis.visualize_scenario_at_time_steps(config.scenario,
                                               plot_limit=config.debug.plot_limits,
                                               time_steps=[12, 37, 56])


if __name__ == "__main__":
    main()
