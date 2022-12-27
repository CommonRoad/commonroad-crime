"""
Exemplary evaluation of TTB using set-based prediction
"""
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.metric.time_scale.ttb import TTB
from commonroad_crime.metric.time_scale.ttr import TTR
from commonroad_crime.metric.time_scale.wttr import WTTR
from commonroad_crime.metric.time_scale.ttc_star import TTCStar


def main():

    scenario_id = 'ZAM_Urban-7_1_S-2'

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # ********** TTC **********
    # ==== select the criticality metric
    # ttc_interface = TTCStar(config)
    # # ==== evaluate at time step 0
    # ttc_interface.compute(0)
    #
    # # ==== visualize the result
    # #config.debug.save_plots = False
    # ttc_interface.visualize()

    # ********** TTR **********
    # # ==== select the criticality metric
    # ttr_interface = TTR(config)
    # # ==== evaluate at time step 0
    # ttr_interface.compute(0, verbose=True)
    #
    # # ==== visualize the result
    # #config.debug.save_plots = False
    # ttr_interface.visualize()

    # ************************
    # ********** WTTR **********
    # ==== select the criticality metric
    wttr_interface = WTTR(config)
    # # ==== evaluate at time step 0
    wttr_interface.compute(0, verbose=True)
    #
    # # ==== visualize the result
    #config.debug.save_plots = False
    wttr_interface.visualize()
    # ************************


if __name__ == "__main__":
    main()

