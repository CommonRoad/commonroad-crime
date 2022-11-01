
import unittest

from commonroad_crime.data_structure.scene import Scene
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.logger as util_logger


class TestBase(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'DEU_Test-1_1_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
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
