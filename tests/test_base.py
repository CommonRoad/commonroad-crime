import unittest

import numpy as np
import pytest
import os

from commonroad.scenario.state import InitialState
from commonroad.scenario.trajectory import Trajectory

from commonroad_crime.data_structure.scene import Scene
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.measure import TTC
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.crime_interface import CriMeInterface
import commonroad_crime.utility.logger as util_logger

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem


class TestBase(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = "DEU_Test-1_1_T-1"
        current_dir = os.path.dirname(__file__)
        self.config = CriMeConfiguration.load(
            os.path.join(current_dir, "../config_files", f"{scenario_id}.yaml"),
            scenario_id,
        )

        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()

    def test_construction(self):
        """
        Test the construction of base classes.
        """
        self.config.update()
        base_1 = CriMeBase(self.config)

        scene = Scene(0, self.config.scenario)
        self.config.update(sce=scene)
        base_2 = CriMeBase(self.config)

        self.assertIsInstance(base_1, CriMeBase)
        self.assertIsInstance(base_2, CriMeBase)

    def test_ego_vehicle_initialization(self):
        """
        Test whether it is fine to initialize the ego vehicle not starting from initial step equal to 0.
        """
        self.config.update()
        ego_vehicle = self.config.scenario.obstacle_by_id(self.config.vehicle.ego_id)
        target_state = ego_vehicle.state_at_time(10)
        ego_vehicle.initial_state = InitialState(
            position=target_state.position,
            orientation=target_state.orientation,
            velocity=target_state.velocity,
            time_step=target_state.time_step,
            yaw_rate=0.0,
            slip_angle=0.0,
        )
        ego_vehicle.prediction.trajectory = Trajectory(
            10, ego_vehicle.prediction.trajectory.state_list[9:]
        )
        CriMeBase(self.config)

    def test_clcs(self):
        """
        Test the update of the CLCS.
        """
        self.config.update()
        base = CriMeBase(self.config)
        with pytest.raises(AttributeError):
            base.clcs = self.config.vehicle.curvilinear.clcs

        example_list = [
            np.array([1.0, 2.0]),
            np.array([3.0, 4.0]),
            np.array([5.0, 6.0]),
        ]
        new_clcs = CurvilinearCoordinateSystem(example_list)
        self.config.update(CLCS=new_clcs)
        self.assertEqual(base.clcs, new_clcs)

    def test_nan_evaluation(self):
        scenario_id = "USA_US101-5_1_T-1"
        current_dir = os.path.dirname(__file__)

        config = CriMeConfiguration.load(
            os.path.join(current_dir, "../config_files", f"{scenario_id}.yaml"),
            scenario_id,
        )

        config.update()
        config.vehicle.ego_id = 439
        crime_interface = CriMeInterface(config)

        crime_interface.evaluate_scene([TTC], time_step=30)
        self.assertEqual(crime_interface.criticality_dict[30][TTC.measure_name], np.inf)

        crime_interface.evaluate_scene([TTC], time_step=35)
        self.assertTrue(
            np.isnan(crime_interface.criticality_dict[35][TTC.measure_name]),
            "The result should be NaN.",
        )

        crime_interface.save_to_file(config.general.path_output)
