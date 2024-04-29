# Changelog
## [0.4.0] - 2024.xx.xx
### Updated
- Visualization of criticality curve: now the nan and inf are distinguished!! The tick for inf is automatically generated!
### Added
- For evaluating an interval, the starting time step should be always smaller or equal than the ending one.
### Fixed
- the memory issues for P_MC measure due to all the simulated vehicle states being stored during the evaluation for subsequent visualization, now the default mode for visualization is off.
- for all measures, check whether the time step is valid in the function `validate_update_states_log`. If no, `NaN` is returned. Similar to `compute_criticality`: only compute for the valid ego vehicle and other vehicles, otherwise `NaN` is returned
- check whether the vehicles are in the same lanelet: now the lanelet is extended by its successors and predecessors
- adds the error handling of the out-of-projection-domain for many measures, such as TTC, THW, ALongReq, ALatReq
- Initialization of the ego vehicle in the base class
- fix the pf and soi measure, when the other vehicle is not present in the scenario for some time steps
- fix the dce and ttce measure of which the initial value should be math.inf
- fix the thw and soi with the support of obstacles in circular shapes
## [0.3.2 & 0.3.3] - 2024.03.16
### Added
- Error handling for check in same lanelet
### Fixed
- The extension of the lanelet vertices for projecting between different coordinate domains
- The compatability with the new commonroad-io version
- The floating-point precision errors for orientation computation
- The setup of the CLCS attribute of each measure can be updated correctly using the `update` function in the configuration
## [0.3.1] - 2023.11.08
### Added
- Add the log verbose for batch evaluation
### Changed
- The lanelet vertices are now extended and smoothed before constructing the curvlinear coordinate system; therefore, some of the test results vary a bit
### Fixed
- The evasive maneuver simulator: now the initial time step of the simulated vehicle doesn't have to start from 0
- The adaptive turning behaviors at intersections for the TTM simulation, i.e., the velocity needs to first satisfy the desired requirements based on the intersection's curvature
- When an empty yaml file is provided for batch evaluation, it would now use the default configuration setup.
- The bridge to compute the drivable area, now it is able to get the scenario from the same folder
- The turning simulation: the last stage orientation error
## [0.3.0] - 2023.08.14
### Fixed
- The original equation for computing TTC is wrong, now its fixed (also updated in the paper)
- fix the TCI, whose optimization problem was not reset each time before
- verbose for each measure and the batch evaluation
- without updating the default vehicle states when creating the other vehicle (together with the `CommonRoad-io==2023.3`)
### Added
- support for Python 3.10 and preparation for 3.11
- add code style checking (black)
- sequential batch evaluation
- add the following new measures
  - MSD, PSD
  - CI, CPI
  - ET, PET
  - SOI
### Updated
- update the installation of commonroad-reach using pip install, the corresponding code part is updated
## [0.2.3 & 0.2.4] - 2023.05.08
### Fixed
- fix the way of obtaining the default configurations, which didn't work for using the crime with the pip package
- fix the initialization of simulators with NONE maneuver
## [0.2.2] - 2023.05.03
### Fixed
- fixing the version of numpy
## ~~[0.2.1] - 2023.05.03~~
### Changed
- making changes for the final submission of IV2023
### Fixed
- the set prediction used for computing the DCE
- the wrong update of vehicle state (acceleration values) during simulation for TTM 
- the time step issue of some measures
### Added
- consideration of pedestrian as the dynamic obstable, i.e., the circular shape
## [0.2.0] - 2023.04.05
### Changed
- Putting the deault configs within the package
- Upgraded to commonroad-io==2023.1
- Updated the ReadMe
### Added
- Plotting functions for TIT and TET
- Scripts for rerunning the evaluation for paper IV2023
- Implementation of new measures: DCE, TTCE
### Fixed
- The computation of THW, the dimension was not correctly considered
- The naming of measures, now the hyphens are added if needed
- Sign of computing the absolute acceleration values
- Plotting of some measures for multiple time steps, where the axes need to be cleared
- Errors of some functions, which still comply with the deprecated commonroad-io
- version of casadi (>3.6.0: ipopt cannot be found in the ci)

## [0.1.1] - 2023.03.23
### Changed
- Support for shapely>=2.0.0
- Upgraded to commonroad-io==2023.1

