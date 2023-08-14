import commonroad_crime.utility.batch_evaluation as utils_batch
from commonroad_crime.measure import TTC
import time


def main():
    scenario_path_4p = scenario_path_4s = "./../scenarios/batch/"
    start = time.time()
    print("Testing run_parallel()...")
    utils_batch.run_parallel(
        scenario_path_4p, config_root="./../config_files", measures=[TTC], verbose=False
    )
    end_par = time.time()
    print("Testing run_sequential()...")
    utils_batch.run_sequential(
        scenario_path_4s,  # config_root='./../config_files',
        measures=[TTC],
        verbose=False,
    )
    end_seq = time.time()
    par_t = end_par - start
    seq_t = end_seq - end_par

    print("Time used for batch evaluation:")
    print("=========================")
    print(f"Par: {par_t} s.")
    print(f"Seq: {seq_t} s.")
    print(f"Speedup: {seq_t/par_t}.")


if __name__ == "__main__":
    main()
