import unittest

from caesar.planner.kop import prepare_last_data_at_kop


class TestPrepareLastDataAtKop(unittest.TestCase):
    def setUp(self):
        self.survey = {
            "depth": 1000.0, "inclination": 30.0, "azimuth": 90.0,
            "verticalDepth": 900.0, "ns": 100.0, "ew": 200.0,
            "displacement": 150.0,
        }

    def test_projects_hold_to_absolute_kop_md(self):
        result = prepare_last_data_at_kop(self.survey, 1200.0)
        self.assertAlmostEqual(result["depth"], 1200.0)
        self.assertAlmostEqual(result["verticalDepth"], 900.0 + 200.0 * 3**0.5 / 2)
        self.assertAlmostEqual(result["ns"], 100.0)
        self.assertAlmostEqual(result["ew"], 300.0)
        self.assertEqual(result["dls"], 0.0)
        self.assertEqual(self.survey["depth"], 1000.0)

    def test_rejects_kop_before_last_survey(self):
        with self.assertRaises(ValueError):
            prepare_last_data_at_kop(self.survey, 999.0)

    def test_same_md_keeps_position(self):
        result = prepare_last_data_at_kop(self.survey, 1000.0)
        self.assertEqual(result["verticalDepth"], self.survey["verticalDepth"])
        self.assertEqual(result["ns"], self.survey["ns"])
        self.assertEqual(result["ew"], self.survey["ew"])


if __name__ == "__main__":
    unittest.main()
