import commonroad_crime.utility.batch_evaluation as utils_batch
from commonroad_crime.measure import TTC
import time

def main():
    scenario_path_4p = "./batches/par/"
    scenario_path_4s = "./batches/seq/"
    start = time.time()
    print("Testing run_parallel()...")
    end_par=time.time()
    utils_batch.run_parallel(scenario_path_4p, measures=[TTC])
    end_par=time.time()
    print("Testing run_sequential()...")
    end_par=time.time()
    utils_batch.run_sequential(scenario_path_4s, measures=[TTC])
    end_seq=time.time()
    par_t = end_par-start
    seq_t = end_seq-end_par

    print("Time used for batch evaluation:")
    print("=========================")
    print(f"Par: {par_t} s.")
    print(f"Seq: {seq_t} s.")
    print(f"Speedup: {seq_t/par_t}.")

if __name__ == "__main__":
    main()
