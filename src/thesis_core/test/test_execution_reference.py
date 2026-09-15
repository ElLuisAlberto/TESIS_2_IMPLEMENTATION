import math
import unittest

from thesis_core.execution_reference import normalize_target
from thesis_core.execution_reference import sample_reference


class TestExecutionReference(unittest.TestCase):

    def test_sample_reference_uses_absolute_elapsed_time(self):
        start = (0.0, 1.0, 2.0, 0.0, 0.0, 0.0)
        target = (math.pi, 2.0, 3.0, 0.0, 0.0, 0.0)

        samples = sample_reference(start, target, 2.0, 0.5, 1.0, 3)

        self.assertAlmostEqual(samples[0][0], math.pi / 4.0)
        self.assertAlmostEqual(samples[1][0], math.pi / 2.0)
        self.assertAlmostEqual(samples[2][0], 3.0 * math.pi / 4.0)

        late = sample_reference(start, target, 2.0, 1.5, 1.0, 3)
        self.assertAlmostEqual(late[-1][0], math.pi)
        self.assertEqual(late[-1][1:], (2.0, 3.0, 0.0, 0.0, 0.0))


    def test_continuous_joint_uses_shortest_turn(self):
        start = (math.radians(170.0), 1.0, 2.0, 0.0, 0.0, 0.0)
        target = (math.radians(-170.0), 1.0, 2.0, 0.0, 0.0, 0.0)

        normalized = normalize_target(start, target)

        self.assertAlmostEqual(normalized[0], math.radians(190.0))


    def test_sample_reference_rejects_invalid_input(self):
        vector = (0.0, 1.0, 2.0, 0.0, 0.0, 0.0)

        with self.assertRaises(ValueError):
            sample_reference(vector, vector, 0.0, 0.0, 1.0, 3)

        with self.assertRaises(ValueError):
            sample_reference(vector, vector, 1.0, 0.0, 1.0, 1)


if __name__ == '__main__':
    unittest.main()
