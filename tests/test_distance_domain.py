"""
Unit tests of the module distance-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure import PSD,MSD
import commonroad_crime.utility.logger as util_logger


class TestDistanceDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'BEL_Putte-8_2_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_MSD(self):
        MSD_object = MSD(self.config)
        msd = msd_object.compute(0)
        self.assertGreater(msd, 0)
        self.assertEqual(msd, 7.6)
        

    def test_dce(self):
        PSD_object = PSD(self.config)
        PSD = psd_object.compute(328, 0)
        PSD_object.visualize()
        self.assertEqual(PSD, 1.54)

