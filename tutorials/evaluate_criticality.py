
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.data_structure.crime_interface import CriMeInterface
from commonroad_crime.metric.time_scale import TTC, TTCStar, TTB, TTS, TTK, TTR


def main():

    scenario_id = 'ZAM_Urban-3_3_Repair'

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # ==== compute the criticality using CriMe interface
    crime_interface = CriMeInterface(config)
    crime_interface.evaluate([TTC, TTCStar, TTB, TTS, TTK, TTR])

    # ==== select the criticality metric you want to evaluate and then compute the value at a given time step

    # ttr_interface = TTR(config)
    # ttr_interface.compute_criticality(0, verbose=True)
    #
    # # ==== visualize the result
    # config.debug.save_plots = False
    # ttr_interface.visualize()


if __name__ == "__main__":
    main()

