__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import dataclasses
import inspect
from dataclasses import dataclass, field
from typing import Union, Any, Dict, List, Optional
import pathlib
import os
import logging
from omegaconf import OmegaConf


from commonroad.scenario.scenario import Scenario
from commonroad.common.solution import VehicleType
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.feasibility.vehicle_dynamics import PointMassDynamics
from commonroad_crime.data_structure.scene import Scene
import commonroad_crime.utility.general as utils_general

from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3

logger = logging.getLogger(__name__)


def _dict_to_params(dict_params: Dict[str, Any], cls: Any) -> Any:
    """
    Converts dictionary to parameter class.

    :param dict_params: Dictionary containing parameters.
    :param cls: Parameter dataclass to which dictionary should be converted to.
    :return: Parameter class.
    """
    fields = dataclasses.fields(cls)
    cls_map = {f.name: f.type for f in fields}
    kwargs = {}
    for k, v in cls_map.items():
        if k not in dict_params:
            continue
        if inspect.isclass(v) and issubclass(v, BaseConfig):
            kwargs[k] = _dict_to_params(dict_params[k], cls_map[k])
        else:
            kwargs[k] = dict_params[k]
    return cls(**kwargs)


@dataclass
class BaseConfig:
    """Base CriMe parameters."""

    __initialized: bool = field(init=False, default=False, repr=False)

    def __post_init__(self):
        """Post initialization of base parameter class."""
        # pylint: disable=unused-private-member
        self.__initialized = True
        # Make sure that the base parameters are propagated to all sub-parameters
        # This cannot be done in the init method, because the sub-parameters are not yet initialized.
        # This is not a noop, as it calls the __setattr__ method.
        # Do not remove!
        # See commonroad-io how to set the base parameters

    def __getitem__(self, item: str) -> Any:
        """
        Getter for base parameter value.

        :param: Item for which content should be returned.
        :return: Item value.
        """
        try:
            value = self.__getattribute__(item)
        except AttributeError as e:
            raise KeyError(
                f"{item} is not a parameter of {self.__class__.__name__}"
            ) from e
        return value

    def __setitem__(self, key: str, value: Any):
        """
        Setter for item.

        :param key: Name of item.
        :param value: Value of item.
        """
        try:
            self.__setattr__(key, value)
        except AttributeError as e:
            raise KeyError(
                f"{key} is not a parameter of {self.__class__.__name__}"
            ) from e

    @classmethod
    def load(
        cls,
        file_path: Union[pathlib.Path, str],
        scenario_name: str,
        validate_types: bool = True,
    ) -> "CriMeConfiguration":
        """
        Loads config file and creates parameter class.

        :param scenario_name: Name of scenario which should be used.
        :param file_path: Path to yaml file containing config parameters.
        :param validate_types: overwrite the default params
        :return: Base parameter class.
        """
        file_path = pathlib.Path(file_path)
        assert (
            file_path.suffix == ".yaml"
        ), f"File type {file_path.suffix} is unsupported! Please use .yaml!"
        loaded_yaml = OmegaConf.load(file_path)
        if validate_types:
            OmegaConf.merge(OmegaConf.structured(CriMeConfiguration), loaded_yaml)
        params = _dict_to_params(OmegaConf.to_object(loaded_yaml), cls)
        params.general.set_scenario_name(scenario_name)
        return params


@dataclass
class VehicleConfiguration(BaseConfig):
    def __post_init__(self):
        # vehicle configuration in the cartesian frame
        self.dynamic = PointMassDynamics(
            VehicleType(VehicleConfiguration.id_type_vehicle)
        )
        self.params = self.to_vehicle_parameter(VehicleConfiguration.id_type_vehicle)

    @dataclass
    class Cartesian(BaseConfig):
        """Parameters in the Cartesian coordinate system."""

        # jerk in m/s^3
        j_x_min: str = -25.0
        j_x_max: str = 25.0
        j_y_min: str = -25.0
        j_y_max: str = 25.0

        # acceleration in m/s^2
        a_x_min: str = -8.0
        a_x_max: str = 8.0
        a_y_min: str = -8.0
        a_y_max: str = 8.0

    @dataclass
    class Curvilinear(BaseConfig):
        """Parameters in the curvilinear coordinate system."""

        # velocity in m/s
        v_lon_min: float = 0.0
        v_lon_max: float = 50.0
        v_lat_min: float = -3.0
        v_lat_max: float = 3.0

        # acceleration in m/s^2
        a_lon_max: float = 11.5
        a_lon_min: float = -11.5  # braking acceleration
        a_lat_max: float = 8.0
        a_lat_min: float = -8.0
        # engine power
        a_max: float = 8.0

        # jerk in m/s^3
        j_lon_min: float = -25.0
        j_lon_max: float = 25.0
        j_lat_min: float = -25.0
        j_lat_max: float = 25.0

        reference_point: str = "rear"

    @dataclass
    class OtherVehicle(BaseConfig):
        """
        Parameters for other vehicles
        """

        m_b: Union[float, None] = None

    @staticmethod
    def to_vehicle_parameter(vehicle_type: int):
        if vehicle_type == 1:
            return parameters_vehicle1()
        elif vehicle_type == 2:
            return parameters_vehicle2()
        elif vehicle_type == 3:
            return parameters_vehicle3()
        else:
            raise TypeError(f"Vehicle type {vehicle_type} not supported!")

    width: Union[None, float] = None
    length: Union[None, float] = None
    ego_id: Union[None, int] = None
    id_type_vehicle: int = 2  # 1 = Ford Escord, 2 = BMW 320i, 3 = VW Vanagon
    other: OtherVehicle = field(default_factory=OtherVehicle)
    cartesian: Cartesian = field(default_factory=Cartesian)
    curvilinear: Curvilinear = field(default_factory=Curvilinear)


@dataclass
class GeneralConfiguration(BaseConfig):
    """General parameters for evaluations."""

    # paths are relative to the root directory
    path_root_abs = os.path.normpath(os.path.join(os.path.dirname(__file__), "../.."))
    path_scenarios: str = os.path.join(path_root_abs, "scenarios")
    path_scenarios_batch: str = os.path.join(path_root_abs, "scenarios/batch")
    path_output_abs: str = os.path.join(path_root_abs, "output")
    path_logs: str = os.path.join(path_root_abs, "output/logs")
    path_icons: str = os.path.join(path_root_abs, "docs/icons")
    name_scenario: Optional[str] = None

    @property
    def path_scenario(self):
        return os.path.join(self.path_scenarios, f"{self.name_scenario}.xml")

    @property
    def path_output(self):
        return os.path.join(self.path_output_abs, self.name_scenario)

    def set_scenario_name(self, scenario_name: str):
        """
        Setter for scenario name.
        :param scenario_name: Name of CommonRoad scenario.
        """
        self.name_scenario = scenario_name


@dataclass
class TimeDomainConfiguration(BaseConfig):
    """Parameters for time-domain measures"""

    # mode for computing the steering width
    # 1. default value from the paper "A flexible method for criticality assessment in driver assistance systems"
    # 2. based on the lane width
    steer_width: int = 1
    # threshold for TET and TIT
    # default value as recommended by Yuanfei
    tau: float = 2.0
    # for computing the TTX with the braking maneuver, the threshold for determining whether the car should stop
    braking_vel_threshold: float = 0.2


@dataclass
class ReachableSetDomainConfiguration(BaseConfig):
    """Parameters for reachable-set-domain measures"""

    # nr of time steps
    time_horizon: int = 10
    # 1 for cartesian frame
    # 2 for curvilinear coordinate system
    coordinate_system: int = 2


@dataclass
class VelocityDomainConfiguration(BaseConfig):
    """Parameters for velocity-domain measures"""

    m_b: Union[float, None] = VehicleConfiguration.OtherVehicle.m_b


class AccelerationDomainConfiguration(BaseConfig):
    """Parameters for acceleration-domain measures"""

    # from Schubert, Robin, Karsten Schulze, and Gerd Wanielik. "Situation assessment for automatic lane-change
    # maneuvers." IEEE Transactions on Intelligent Transportation Systems 11.3 (2010): 607-616.
    safety_time: float = 2  # s

    # the assumption of acceleration values
    # 1. constant acceleration
    # 2. piecewise constant motion: all vehicles will remain stationary when zero velocity is reached
    acceleration_mode: int = 1


@dataclass
class PotentialDomainConfiguration(BaseConfig):
    """Parameters for potential-domain measures"""

    # from Wolf, M.T. and Burdick, J.W., 2008, May. Artificial potential functions for highway driving with collision
    # avoidance. In 2008 IEEE International Conference on Robotics and Automation (pp. 3731-3736). IEEE.
    A_lane: float = 2.0  # maximum amplitude of the lane divider potential
    A_car: float = 10.0  # Yukawa amplitude
    sigma_factor: float = 3.0  # * lane width - how quickly the potential rises/falls
    scale_factor: float = 100.0  # scaling factor for road potential
    slope_scale: float = 0.2  # slope scale of velocity potential
    alpha: float = 0.5  # Yukawa scale
    beta: float = 0.6  # exponential scale of car potential
    follow_time: float = 3.0  # desired following time
    wedge_vertex: float = -0.5  # the location of the rear-most vertex of the triangle
    desired_speed: float = 20.0  # !!null # to guide the vehicle towards a desired speed
    d_0: float = (
        50.0  # the maximum distance at which the car potential exerts an influence
    )
    u_max: float = (
        100.0  # maximum potential for visualization purposes, otherwise it would be inf
    )


@dataclass
class ProbabilityDomainConfiguration(BaseConfig):
    @dataclass
    class MonteCarlo(BaseConfig):
        # params are obtained from Eidehall A, Petersson L. Statistical threat assessment for general road scenes
        # using Monte Carlo sampling. IEEE Transactions on intelligent transportation systems. 2008 Feb 26;9(1):
        # 137-47.
        prediction_horizon: float = 3.0  # second
        nr_samples: int = 50
        # weights for maneuvers [stop, turn, lane change, overtake, random]
        mvr_weights: List[int] = field(default_factory=lambda: [0, 0, 0, 0, 1])

    monte_carlo: MonteCarlo = field(default_factory=MonteCarlo)


@dataclass
class IndexDomainConfiguration(BaseConfig):
    @dataclass
    class TCI(BaseConfig):
        """Trajectory Criticality Index"""

        # params are obtained from P. Junietz, F. Bonakdar, B. Klamann, and H. Winner,
        # “Criticality metric for the safety validation of automated driving using model predictive trajectory
        # optimization,” in 21st International Conference on Intelligent Transportation Systems (ITSC),
        # pp. 60–65, IEEE, 2018.
        w_x: float = 1.0
        w_y: float = 1.0
        w_ax: float = 0.1
        w_ay: float = 1.0
        N: int = 20  # nr of time steps

    @dataclass
    class CI(BaseConfig):
        """Conflict Index"""

        # params from W. K. Alhajyaseen, “The integration of conflict probability and severity for the safety
        # assessment of intersections”, Arabian Journal for Science and Engineering, vol. 40, no. 2, pp. 421–430,
        # 2015.
        m_b: Union[float, None] = VehicleConfiguration.OtherVehicle.m_b
        alpha: float = 1.0
        beta: float = 1.0
        pet_threshold: float = 5.0

    @dataclass
    class CPI(BaseConfig):
        """Crash Potential Index"""

        # params are obtained from Cunto, Flavio Jose Craveiro, and Frank F. Saccomanno. Microlevel
        # traffic simulation method for assessing crash potential at intersections. No. 07-2180. 2007.
        madr_mean: float = 8.45
        madr_devi: float = 1.40
        madr_uppb: float = 12.68
        madr_lowb: float = 4.23

    @dataclass
    class SOI(BaseConfig):
        """Space Occupancy Index"""

        # Default values for SOI, defining minimum personal space around a vehicle in meters.
        # Will be added to the calculated personal space.
        margin_front: float = 2
        margin_back: float = 0.5
        margin_side: float = 0.5

    tci: TCI = field(default_factory=TCI)
    ci: CI = field(default_factory=CI)
    cpi: CPI = field(default_factory=CPI)
    soi: SOI = field(default_factory=SOI)


@dataclass
class DebugConfiguration(BaseConfig):
    save_plots: bool = True
    plot_limits: Union[List[float], None] = None
    # visualization settings
    draw_visualization: bool = False
    # visualize dynamic obstacles with icons
    draw_icons: bool = True


@dataclass
class CriMeConfiguration(BaseConfig):
    """Class to hold criticality-measures-related configurations"""

    general: GeneralConfiguration = field(default_factory=GeneralConfiguration)
    vehicle: VehicleConfiguration = field(default_factory=VehicleConfiguration)
    debug: DebugConfiguration = field(default_factory=DebugConfiguration)
    time: TimeDomainConfiguration = field(default_factory=TimeDomainConfiguration)
    velocity: VelocityDomainConfiguration = field(
        default_factory=VelocityDomainConfiguration
    )
    acceleration: AccelerationDomainConfiguration = field(
        default_factory=AccelerationDomainConfiguration
    )
    potential: PotentialDomainConfiguration = field(
        default_factory=PotentialDomainConfiguration
    )
    probability: ProbabilityDomainConfiguration = field(
        default_factory=ProbabilityDomainConfiguration
    )
    reachable_set: ReachableSetDomainConfiguration = field(
        default_factory=ReachableSetDomainConfiguration
    )
    index: IndexDomainConfiguration = field(default_factory=IndexDomainConfiguration)

    def __post_init__(self):
        self.scenario: Optional[Scenario] = None
        self.scene: Optional[Scene] = None

    def update(
        self,
        ego_id: int = None,
        sce: Union[Scene, Scenario] = None,
        CLCS: Optional[CurvilinearCoordinateSystem] = None,
    ):
        """
        Updates criticality configuration based on the given attributes.

        Possible ways of updating the configuration:
        1. No attribute is given: load the scenario and evaluate the criticality of all obstacles.
        2. scenario
        3. scene
        4. scenario + clcs
        5. scene + clcs
        """
        if isinstance(sce, Scene):
            self.scene = sce
            self.general.name_scenario = str(sce.scenario_id)
        elif isinstance(sce, Scenario):
            self.scenario = sce
            self.general.name_scenario = str(sce.scenario_id)
        elif not sce and (
            self.scene or self.scenario
        ):  # prevent reloading the existing scenario
            pass
        else:
            self.scenario = utils_general.load_scenario(
                self
            )  # if none is provided, scenario is at default
        if CLCS:
            self.vehicle.curvilinear.clcs = CLCS
        if ego_id:
            if self.scenario:
                if self.scenario.obstacle_by_id(ego_id) is None:
                    assert f"<Criticality>: Vehicle (id: {ego_id}) is not contained in the scenario!"
            if self.scene:
                if self.scene.obstacle_by_id(ego_id) is None:
                    assert f"<Criticality>: Vehicle (id: {ego_id}) is not contained in the scenario!"
            self.vehicle.ego_id = ego_id

    def print_configuration_summary(self):
        string = "# ===== Configuration Summary ===== #\n"
        string += f"# Scene/Scenario: {self.general.name_scenario}\n"
        string += f"# ego vehicle: id {self.vehicle.ego_id}\n"
        string += "# ================================= #"

        print(string)
        for line in string.split("\n"):
            logger.info(line)
