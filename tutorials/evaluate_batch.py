import commonroad_crime.utility.batch_evaluation as utils_batch
from commonroad_crime.measure import PSD


def main():
    scenario_path = "../scenarios_subset"
    config_root = "../config_files"
    utils_batch.run_sequential(scenario_path=scenario_path,
                               config_root=config_root,
                               measures=[PSD])


if __name__ == "__main__":
    main()
