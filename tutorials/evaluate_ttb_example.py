"""
Exemplary evaluation of TTB using set-based prediction
"""
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.metric.time_scale.ttb import TTB


def main():

    scenario_id = 'ZAM_Urban-7_1_S-2'

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # ********** TTB **********
    # ==== select the criticality metric
    ttb_interface = TTB(config)
    # ==== evaluate at time step 0
    ttb_interface.compute(0, verbose=True)

    # ==== visualize the result
    config.debug.save_plots = False
    ttb_interface.visualize()
    # ************************


if __name__ == "__main__":
    main()

