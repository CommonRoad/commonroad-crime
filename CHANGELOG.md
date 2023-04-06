# Changelog

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

