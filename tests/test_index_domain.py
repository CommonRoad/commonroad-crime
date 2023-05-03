"""
Unit tests of the module index-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure import BTN, STN, TCI
import commonroad_crime.utility.logger as util_logger


class TestIndexDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Zip-1_56_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_bnt(self):
        btn_object = BTN(self.config)
        # vehicle in the same lanelet and in front
        btn_1 = btn_object.compute(3, 0)
        btn_object.visualize()
        self.assertAlmostEqual(btn_1, 0.1043)

        # vehicle in another lanelet
        btn_2 = btn_object.compute(1, 0)
        self.assertAlmostEqual(btn_2, 0.0)

    def test_snt(self):
        stn_object = STN(self.config)
        # vehicle in the same lanelet and in front
        stn_1 = stn_object.compute(3, 0)
        self.assertAlmostEqual(stn_1, 0.0263)

        # vehicle in another lanelet
        stn_2 = stn_object.compute(1, 0)
        self.assertAlmostEqual(stn_2, 0.0)

    def test_tci(self):
        self.config.vehicle.ego_id = 1
        tci_object = TCI(self.config)
        tci_1 = tci_object.compute(0)
        tci_object.visualize()
        self.assertEqual(tci_1, 0.0)
