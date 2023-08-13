from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.visualization as utils_vis


def main():
    scenario_id = "OSC_CutIn-1_2_T-1"
    # time_steps=[20, 34, 99])
    scenario_id = "OSC_Overtake-1_1_T-1"
    # ==== build configuration
    config = CriMeConfiguration.load(f"../config_files/{scenario_id}.yaml", scenario_id)
    config.update()
    config.print_configuration_summary()

    # ==== visualize the scenario at given time steps
    utils_vis.visualize_scenario_at_time_steps(
        config.scenario, plot_limit=config.debug.plot_limits, time_steps=[69, 121, 129]
    )


if __name__ == "__main__":
    main()
