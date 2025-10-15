import types
import unittest

from pyvesc.VESC.VESC import VESC


class TestMeasurementAttributeResolution(unittest.TestCase):
    def test_returns_first_matching_attribute(self):
        measurements = types.SimpleNamespace(avg_motor_current=12.34)
        value = VESC._resolve_measurement_attribute(
            measurements,
            "avg_motor_current",
            "current_motor",
        )
        self.assertEqual(value, 12.34)

    def test_falls_back_to_legacy_attribute_name(self):
        measurements = types.SimpleNamespace(current_motor=5.67)
        value = VESC._resolve_measurement_attribute(
            measurements,
            "avg_motor_current",
            "current_motor",
        )
        self.assertEqual(value, 5.67)

    def test_raises_attribute_error_when_missing(self):
        measurements = types.SimpleNamespace()
        with self.assertRaises(AttributeError):
            VESC._resolve_measurement_attribute(
                measurements,
                "avg_motor_current",
                "current_motor",
            )


if __name__ == "__main__":
    unittest.main()
