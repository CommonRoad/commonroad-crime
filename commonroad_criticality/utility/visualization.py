from pathlib import Path
import matplotlib.pyplot as plt


def save_fig(metric_name: str, path_output: str, time_step: int):
    # save as svg
    Path(path_output).mkdir(parents=True, exist_ok=True)
    plt.savefig(f'{path_output}{metric_name}_{time_step:03d}.svg', format="svg", bbox_inches="tight",
                transparent=False)
