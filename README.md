# commonroad-criticality-assessment

Toolbox to compute criticality measures (e.g. time-to-collision, time-to-react, time-to-comply,...). Such measures can be used to trigger warnings and emergency maneuvers in driver assistance systems or repair an infeasible trajectory. 

## The required Python dependencies

You have to mannually install the following packages:
* [CommonRoad Drivability Checker](https://commonroad.in.tum.de/drivability-checker)>=2021.1
* [CommonRoad Reach](https://gitlab.lrz.de/cps/commonroad-reachable-set)
* [STL Monitor](https://gitlab.lrz.de/ge69xek/stl_crmonitor): under branch feature_interface

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
$ pip install -r requirements.txt
```
This will install related dependencies specified in `requirements.txt`. Or simply install the dependencies listed in `requirements.txt` and add this repository to your python path.
