
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
# from commonroad_crime.metric.time_scale.ttc import TTC
# from commonroad_crime.metric.time_scale.ttk import TTK
from commonroad_crime.metric.time_scale.ttr import TTR


def main():

    scenario_id = 'ZAM_Urban-3_3_Repair'

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # steering based on the lane width
    config.time_scale.steer_width = 2

    # ==== select the criticality metric you want to evaluate and then compute the value at a given time step
    # ttk_interface = TTK(config)
    # ttc_interface = TTC(config)
    ttr_interface = TTR(config)
    ttr_interface.compute_criticality(0, verbose=True)

    # ==== visualize the result
    config.debug.save_plots = False
    ttr_interface.visualize()


if __name__ == "__main__":
    main()

