import unittest

import numpy as np
import pytest

from commonroad_crime.data_structure.scene import Scene
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.logger as util_logger

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem


class TestBase(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = "DEU_Test-1_1_T-1"
        self.config = CriMeConfiguration.load(
            f"../config_files/{scenario_id}.yaml", scenario_id
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
