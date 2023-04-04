import commonroad_crime.utility.batch_evaluation as utils_batch
from commonroad_crime.measure import TTC


def main():
    scenario_path = " "
    config_root = " "
    utils_batch.run_sequential(scenario_path=scenario_path,
                               config_root=config_root,
                               measures=[TTC])


if __name__ == "__main__":
    main()
