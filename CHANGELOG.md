# Changelog
## [0.3.0] - 2023.xx.xx
### Fixed
- The original equation for computing TTC is wrong, now its fixed (also updated in the paper)
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

