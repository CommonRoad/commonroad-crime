
from commonroad_criticality.data_structure.configuration_builder import ConfigurationBuilder
# from commonroad_criticality.metric.time_scale.ttc import TTC
# from commonroad_criticality.metric.time_scale.ttk import TTK
from commonroad_criticality.metric.time_scale.ttr import TTR


def main():

    scenario_id = 'ZAM_Urban-3_3_Repair'

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()

    # steering based on the lane width
    config.time_metrics.steer_width = 2

    # ==== select the criticality metric you want to evaluate and then compute the value at a given time step
    # ttk_interface = TTK(config)
    # ttc_interface = TTC(config)
    ttr_interface = TTR(config)
    ttr = ttr_interface.compute(0)
    print(f"The {ttr_interface.metric_name} of the scenario is {ttr}.")

    # ==== visualize the result
    ttr_interface.visualize()


if __name__ == "__main__":
    main()

