# CommonRoad-Criticality

Toolbox to compute criticality measures (e.g. time-to-collision, time-to-react, time-to-comply,...). Such measures can be used to trigger warnings and emergency maneuvers in driver assistance systems or repair an infeasible trajectory. 


## Installation Guide
We recommend using [Anaconda](https://www.anaconda.com/) to manage your environment so that even if you mess something up, you can always have a safe and clean restart. A guide for managing python environments with Anaconda can be found [here](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

After installing Anaconda, create a new environment with:
``` sh
$ conda create -n commonroad-py37 python=3.7 -y
```

Here the name of the environment is called **commonroad-py37**. You may also change this name as you wish. In such case, don't forget to change it in the following commands as well. **Always activate** this environment before you do anything related:

```sh
$ conda activate commonroad-py37
or
$ source activate commonroad-py37
```
Then, install the dependencies with:

```sh
$ cd <path-to-this-repo>
$ pip install -r requirements.txt
$ conda develop .
```
(optional) In order to run reachability analysis related metrics (e.g., WTTR), you have to install the following repositories as well:

- [commonroad-reach](https://gitlab.lrz.de/tum-cps/commonroad-reach)

To test the installition, run unittest:
```bash
$ cd tests
$ python -m unittest -v
```

This will install related dependencies specified in `requirements.txt`. Or simply install the dependencies listed in `requirements.txt` and add this repository to your python path.

To get started your journey with our criticality measures, check the `tutorials`.