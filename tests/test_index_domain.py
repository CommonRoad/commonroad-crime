"""
Unit tests of the module index-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.measure import BTN, STN, TCI, CPI, CI, SOI
import commonroad_crime.utility.logger as util_logger

from commonroad.common.file_reader import CommonRoadFileReader


class TestIndexDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = "ZAM_Zip-1_56_T-1"
        self.config = CriMeConfiguration.load(
            f"../config_files/{scenario_id}.yaml", scenario_id
        )
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
        self.assertAlmostEqual(stn_1, 0.0238)

        # vehicle in another lanelet
        stn_2 = stn_object.compute(1, 0)
        self.assertAlmostEqual(stn_2, 0.0)

    def test_tci(self):
        self.config.vehicle.ego_id = 1
        tci_object = TCI(self.config)
        tci_1 = tci_object.compute(0)
        tci_object.visualize()
        self.assertEqual(tci_1, 0.0)

    def test_cpi(self):
        cpi_object = CPI(self.config)
        cpi = cpi_object.compute_criticality(0)
        cpi_object.visualize()
        self.assertAlmostEqual(cpi, 4.4345e-06)

    def test_ci(self):
        ci_object = CI(self.config)
        ci = ci_object.compute(3, 0)
        self.assertEqual(ci, 0.0)

        # test scenario with valid conflict area
        self.config.general.name_scenario = "ZAM_Tjunction-1_97_T-1"
        sce_pet, _ = CommonRoadFileReader(self.config.general.path_scenario).open(
            lanelet_assignment=True
        )
        self.config.update(ego_id=5, sce=sce_pet)
        ci_object = CI(self.config)
        ci = ci_object.compute(1, 0)
        self.assertAlmostEqual(ci, 2688.3)
        self.config.debug.plot_limits = [-100, 100, -10, 80]
        ci_object.visualize()

    def test_soi(self):
        scenario_id = "ZAM_Urban-3_3_Repair"
        self.config = CriMeConfiguration()
        self.config.general.set_scenario_name(scenario_id)
        self.config.vehicle.ego_id = 8
        self.config.update()

        soi_object_1 = SOI(self.config)
        soi_1 = soi_object_1.compute()
        soi_object_1.visualize()
        self.assertEqual(soi_1, 39.0)

        scenario_id = "USA_Lanker-1_3_T-1"
        self.config = CriMeConfiguration.load(
            f"../config_files/{scenario_id}.yaml", scenario_id
        )
        self.config.update()

        soi_object_2 = SOI(self.config)
        soi_2 = soi_object_2.compute()
        soi_object_2.visualize()
        self.assertEqual(soi_2, 32.0)

        scenario_id = "DEU_Moabit-4_1_T-1"
        self.config = CriMeConfiguration.load(
            f"../config_files/{scenario_id}.yaml", scenario_id
        )
        self.config.update()

        soi_object_3 = SOI(self.config)
        soi_3 = soi_object_3.compute()
        soi_object_3.visualize()
        self.assertEqual(soi_3, 0.0)
