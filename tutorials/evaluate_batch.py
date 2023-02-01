
import os
import sys
import commonroad_crime.utility.batch_evaluation as utils_batch
from commonroad_crime.measure import TTC


def main():
    batch_path = "/home/yuanfei/commonroad/commonroad-criticality-measures/scenarios"
    utils_batch.run_sequential(batch_path, [TTC])


if __name__ == "__main__":
    main()
