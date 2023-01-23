from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure import (TTC, TTCStar, TTB, TTS, TTK, TTR, THW, WTTC, BTN, PF,
                                      ALongReq, ALatReq, STN, P_MC)
from commonroad_crime.measure.reachable_set_scale.drivable_area import DA
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.data_structure.crime_interface import CriMeInterface


def main():
    scenario_id = 'ZAM_Urban-7_1_S-2'
    scenario_id = "DEU_Gar-1_1_T-1"
    scenario_id = "OSC_CutIn-1_2_T-1"

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(scenario_id)
    config.update()
    config.print_configuration_summary()

    # ==== compute the criticality using CriMe interface
    crime_interface = CriMeInterface(config)
    # crime_interface.evaluate_scenario([THW], 0, 50)
    # crime_interface.evaluate([HW, THW, TTC, WTTC, TTCStar, TTS, TTK, TTB, TTR, ])
    # crime_interface.evaluate_scene([WTTR])
    # WTTR_obj = WTTR(config)
    # WTTR_obj.compute(0)
    # WTTR_obj.visualize()
    # crime_interface.evaluate_scene([ALatReq, ALongReq, LongJ, LatJ, BTN, STN])
    # ==== Experiment B: evaluation on scenarios
    # crime_interface.evaluate_scenario([TTC, DA, ALongReq, BTN,
    #                                    ALatReq, STN, P_MC, PF], 0, 20)
    crime_interface.config.vehicle.ego_id = 4
    crime_interface.evaluate_scenario([TTC], time_start=38, time_end=50)
    # crime_interface.evaluate_scenario([ALatReq], 0, 20)
    #
    # crime_interface.evaluate_scenario([PF], 0, 20)
    # crime_interface.evaluate_scene([PF], 12)
    # crime_interface.evaluate_scene([TTC], 12, vehicle_id=202)
    # crime_interface.evaluate_scene([BTN], 0)
    # crime_interface.evaluate_scenario([DeltaV], 0, 30)
    # ==== select the criticality metric you want to evaluate and then compute the value at a given time step
    # da = DA(config)

    # ==== for the sketch visualization of the paper
    # utils_vis.visualize_scenario_at_time_steps(crime_interface.config.scenario,
    #                                            plot_limit=crime_interface.config.debug.plot_limits,
    #                                            time_steps=[20, 34, 99])
    # # ==== visualize the result
    # config.debug.save_plots = False
    #utils_vis.plot_criticality_curve(crime_interface)


if __name__ == "__main__":
    main()
