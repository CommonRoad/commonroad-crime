# CommonRoad-CriMe
![image info](https://gitlab.lrz.de/tum-cps/commonroad-crime/-/raw/master/docs/figures/CriMe-banner.png)
[![Linux](https://img.shields.io/badge/os-linux?&logo=Linux&logoColor=white&labelColor=gray)](https://pypi.python.org/pypi/commonroad-openscenario-converter/)
[![PyPI version fury.io](https://badge.fury.io/py/commonroad-crime.svg?style=plastic)](https://pypi.python.org/pypi/commonroad-crime/)
[![PyPI license](https://img.shields.io/pypi/l/commonroad-crime.svg?style=plastic)](https://pypi.python.org/pypi/commonroad-crime/)<br>
[![PyPI download month](https://img.shields.io/pypi/dm/commonroad-crime.svg?style=plastic&label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-crime/) 
[![PyPI download week](https://img.shields.io/pypi/dw/commonroad-crime.svg?style=plastic&label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-crime/)<br>

Toolbox to compute **Cri**ticality **Me**asures 
(e.g. time-to-collision, time-to-react,...). Such measures
can be used to trigger warnings and emergency maneuvers 
in driver assistance systems or repair an infeasible 
trajectory. 

## Installation Guide

`commonroad-crime` can be installed with:

``` bash
$ pip install commonroad-crime
```
For adding new measures, we recommend using [Anaconda](https://www.anaconda.com/) to manage your environment so that even if you mess something up, you can always have a safe and clean restart. A guide for managing python environments with Anaconda can be found [here](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

After installing Anaconda, create a new environment with:
``` bash
$ conda create -n commonroad-py38 python=3.8 -y
```

Here the name of the environment is called **commonroad-py38**. You may also change this name as you wish. In such case, don't forget to change it in the following commands as well. **Always activate** this environment before you do anything related:

```sh
$ conda activate commonroad-py38
or
$ source activate commonroad-py38
```
Then, install the dependencies with:

```sh
$ cd <path-to-this-repo>
$ pip install -e .
$ conda develop .
```

To test the installition, run unittest:
```bash
$ cd tests
$ python -m unittest -v
```


To get started your journey with our criticality measures, check the `tutorials` and the following tips.

### How to add new criticality measure
1. create a new branch with `feature-<measure-name>` and checkout the branch
2. navigate to `commonroad_crime/data_structure/type.py` to find the correct category of the measure and add an 
enumeration entry `<abbreviation>: <explanation>`
3. navigate to `commonroad_crime/measure` to find the above-mentioned category and create a python file named
`<abbreviation>.py`. Then create a class inheriting the `CriMeBase` under `commonroad_crime/data_structure/base.py`
4. similar to other measures, you need to implement the `compute()` and `visualize()` functions

### How to define configuration parameters of the measure
1. navigate to `commonroad_crime/data_structure/configuation.py` to find the above-mentioned category and add a new 
instance to the class as `self.<parameter> = config_relevant.<parameter>`
2. you can then directly call the values using `self.configuration.<category>.<parameter>` in your measure class
3. to override the default parameter values, create a `yaml` file (name it the same as the scenario) in `./config_files` and modify the values there
## Documentation

The documentation of our toolbox is available on our website: https://cps.pages.gitlab.lrz.de/commonroad/commonroad-criticality-measures/.

In order to generate the documentation via Sphinx locally, run the following commands in the root directory:

```bash
$ pip install -r ./docs/requirements_doc.txt
$ cd docs/sphinx
$ make html
```

The documentation can then be launched by browsing ``./docs/sphinx/build/html/index.html/``.

### Contributors (in alphabetical order by last name)
- Liguo Chen
- Yuanfei Lin
- Sebastian Maierhofer
- Ivana Peneva
- Kun Qian
- Oliver Specht
- Sicheng Wang
- Zekun Xing
- Ziqian Xu

### Citation
If you use `commonroad-crime` for academic work, we highly encourage you to cite our paper:
```text
@InProceedings{lin2023crime,
      title     = {{CommonRoad-CriMe}: {A} Toolbox for Criticality Measures of Autonomous Vehicles},
      author    = {Yuanfei Lin and Matthias Althoff},
      booktitle = {Proc. of the IEEE Intell. Veh. Symp.},     
      pages     = {1-8}, 
      year      = {2023},
}
```
If you use this project's code in industry, we'd love to hear from you as well; 
feel free to reach out to [Yuanfei Lin](mailto:yuanfei.lin@tum.de) directly.
