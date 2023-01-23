"""
Exemplary evaluation of TTB using set-based prediction
"""
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure.time_scale.wttr import WTTR
from commonroad_crime.measure import TTB, TTR, WTTC, TTCStar
from commonroad_crime.measure.reachable_set_scale.drivable_area import DA


def main():

    scenario_id = 'ZAM_Urban-7_1_S-2'
    #scenario_id = "DEU_Gar-1_1_T-1"

    other_id = 99
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

    # ********** WTTC **********
    # ==== select the criticality metric
    # wttc_interface = WTTC(config)
    # # # ==== evaluate at time step 0
    # wttc_interface.compute(other_id, 0)
    # #
    # # # ==== visualize the result
    # # config.debug.save_plots = False
    # wttc_interface.visualize()

    # ********** TTR **********
    # # ==== select the criticality metric
    ttr_interface = WTTR(config)
    # ==== evaluate at time step 0
    ttr_interface.compute(time_step=0, vehicle_id=202, verbose=True)
    ttr_interface.visualize()
    #
    # # ==== visualize the result
    # #config.debug.save_plots = False
    # ttr_interface.visualize()
    #
    # # ************************
    # #********** WTTR **********
    # #==== select the criticality metric
    # wttr_interface = WTTR(config)
    # # # ==== evaluate at time step 0
    # wttr_interface.compute(0, verbose=True)
    # #
    # # # ==== visualize the result
    # #config.debug.save_plots = False
    # wttr_interface.visualize()
    #************************
    #********** WTTR **********
    #==== select the criticality metric
    # da_interface = DA(config)
    # # # ==== evaluate at time step 0
    # da_interface.compute(0, verbose=True)
    #
    # # ==== visualize the result
    #config.debug.save_plots = False
    # da_interface.visualize()
    #************************


if __name__ == "__main__":
    main()

