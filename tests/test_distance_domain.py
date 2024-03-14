"""
Unit tests of the module distance-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.measure import PSD, MSD, HW, DCE
import commonroad_crime.utility.logger as util_logger

from commonroad.common.file_reader import CommonRoadFileReader


class TestDistanceDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = "ZAM_Urban-3_3_Repair"
        self.config = CriMeConfiguration()
        self.config.general.set_scenario_name(scenario_id)
        self.config.vehicle.ego_id = 8
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_MSD_PSD(self):
        self.config.general.name_scenario = "BEL_Putte-8_2_T-1"
        sce_crosswalk, _ = CommonRoadFileReader(self.config.general.path_scenario).open(
            lanelet_assignment=True
        )
        self.config.update(ego_id=349, sce=sce_crosswalk)
        msd_object = MSD(self.config)
        msd = msd_object.compute(328, 0)
        self.assertEqual(msd, 0.32)
        self.config.debug.plot_limits = [-320, -380, 210, 290]
        msd_object.visualize()

        psd_object = PSD(self.config)
        psd = psd_object.compute(328, 0)
        self.assertEqual(psd, 38.49)
        psd_object.visualize()

    def test_hw(self):
        hw_object = HW(self.config)
        hw = hw_object.compute(6, 0)
        self.assertGreater(hw, 0)
        self.assertEqual(hw, 20.75)
        hw_object.visualize()

    def test_dce(self):
        dce_object_1 = DCE(self.config)
        dce_1 = dce_object_1.compute(6, 0)
        dce_object_1.visualize()
        self.assertEqual(dce_1, 0.00)

        dce_object_2 = DCE(self.config)
        dce_2 = dce_object_2.compute(7, 0)
        dce_object_2.visualize()
        self.assertEqual(dce_2, 0.98)
