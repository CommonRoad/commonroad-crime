# CommonRoad-CriMe
![image info](./docs/figures/CriMe-banner.png)
Toolbox to compute **Cri**ticality **Me**asures 
(e.g. time-to-collision, time-to-react,...). Such measures
can be used to trigger warnings and emergency maneuvers 
in driver assistance systems or repair an infeasible 
trajectory. 

![stl-robustness](https://img.shields.io/badge/CommonRoad-CriMe-blue?style=plastic&logo=Coding-Ninjas)
![test-passed](https://img.shields.io/badge/tests-passed-red?style=plastic&logo=Python)
## Installation Guide

`commonroad-crime` can be installed with:

``` bash
$ pip install commonroad-crime
```
For adding new measures, we recommend using [Anaconda](https://www.anaconda.com/) to manage your environment so that even if you mess something up, you can always have a safe and clean restart. A guide for managing python environments with Anaconda can be found [here](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

After installing Anaconda, create a new environment with:
``` bash
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
**(optional)** In order to run reachability analysis related measures 
(e.g., ``worst-time-to-react (WTTR)``, `drivable area (DA)`), you have to install the following repositories as well:

- [commonroad-reach](https://gitlab.lrz.de/tum-cps/commonroad-reach)

To test the installition, run unittest:
```bash
$ cd tests
$ python -m unittest -v
```

This will install related dependencies specified in `requirements.txt`. Or simply install the dependencies listed in `requirements.txt` and add this repository to your python path.

To get started your journey with our criticality measures, check the `tutorials` and the following tips.

### How to add new criticality measure
1. create a new branch with `feature-<measure-name>` and checkout the branch
2. navigate to `commonroad_crime/data_structure/type.py` to find the correct category of the measure and add an 
enumeration entry `<abbreviation>: <explanation>`
3. navigate to `commonroad_crime/measure` to find the above-mentioned category and create a python file named
`<abbreviation>.py`. Then create a class inheriting the `CriMeBase` under `commonroad_crime/data_structure/base.py`
4. similar to other measures, you need to implement the `compute()` and `visualize()` functions

### How to define configuration parameters of the measure
1. navigate to `config_files/default` to find the correct category of the measure
2. add your parameter to the `yaml` file in the format as `<parameter>: <default value>`. Please do add the source
and some explanations there
3. navigate to `commonroad_crime/data_structure/configuation.py` to find the above-mentioned category and add a new 
instance to the class as `self.<parameter> = config_relevant.<parameter>`
4. you can then directly call the values using `self.configuration.<category>.<parameter>` in your measure class