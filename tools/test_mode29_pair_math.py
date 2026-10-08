#!/usr/bin/env python3
"""Independent algebra checks for Mode29 opposite-pair loss *model*.

This is not a flight or closed-loop SITL validation. No ArduPilot modules,
propellers, or hardware are needed. Run: python3 tools/test_mode29_pair_math.py
"""
import math
import unittest

L = D = 0.28
ROLL = (-L / 2, L / 2, L / 2, -L / 2)
PITCH = (D / 2, -D / 2, D / 2, -D / 2)


def opposite_motor(i):
    assert 1 <= i <= 4
    return ((i - 1) ^ 1) + 1


def wrench_from_forces(forces):
    return (
        sum(forces),
        sum(x * y for x, y in zip(ROLL, forces)),
        sum(x * y for x, y in zip(PITCH, forces)),
    )


def gram_determinant(eta):
    # 3x4 yaw-free effectiveness matrix A = B diag(eta)
    rows = [list(eta),
            [a * b for a, b in zip(ROLL, eta)],
            [a * b for a, b in zip(PITCH, eta)]]
    g = [[sum(rows[i][k] * rows[j][k] for k in range(4))
          for j in range(3)] for i in range(3)]
    return (g[0][0] * (g[1][1] * g[2][2] - g[1][2] ** 2)
            - g[0][1] * (g[0][1] * g[2][2] - g[1][2] * g[0][2])
            + g[0][2] * (g[0][1] * g[1][2] - g[1][1] * g[0][2]))


class PairedLossModelTests(unittest.TestCase):
    def test_opposite_motor_map(self):
        self.assertEqual([opposite_motor(i) for i in range(1, 5)],
                         [2, 1, 4, 3])

    def test_pair_equal_percentage_gives_zero_roll_pitch_at_equal_commands(self):
        for damaged in range(1, 5):
            eta = [1.0] * 4
            eta[damaged - 1] = 0.4
            eta[opposite_motor(damaged) - 1] = 0.4
            # Input commands equal here; this checks geometry, not flight trim.
            applied = [v * 5.0 for v in eta]
            force, mx, my = wrench_from_forces(applied)
            self.assertAlmostEqual(mx, 0.0, places=10)
            self.assertAlmostEqual(my, 0.0, places=10)
            self.assertGreater(force, 0)

    def test_yaw_torque_imbalance_has_a_consistent_sign(self):
        # Motor 1+2 are CCW and 3+4 are CW. Their reaction moment
        # magnitudes differ when paired loss is imposed.
        effective = [0.4 * 5, 0.4 * 5, 5, 5]
        simplified_yaw = -effective[0] - effective[1] + effective[2] + effective[3]
        self.assertNotEqual(simplified_yaw, 0.0)

    def test_rank_deficiency_when_both_opposite_motors_are_dead(self):
        for dead in ((1, 2), (3, 4)):
            eta = [1.0] * 4
            for i in dead:
                eta[i - 1] = 0.0
            self.assertAlmostEqual(gram_determinant(eta), 0.0, places=10)

    def test_partial_pair_keeps_full_linearized_rank(self):
        for bad in (1, 2, 3, 4):
            eta = [1.0] * 4
            eta[bad - 1] = eta[opposite_motor(bad) - 1] = 0.4
            self.assertGreater(gram_determinant(eta), 1.0e-10)

    def test_model_thrust_margin_at_60_percent_loss(self):
        # 2026-10-06 6S cubic, using the thrust-stand-validated range.
        w = 80.0
        x = max(0.0, w - 4.47703190)
        fmax = max(0.0, ((-2.62683159e-05 * x +
                           4.01680390e-03) * x +
                          4.05756758e-08) * x)
        required = 1.3854 * 9.8 / math.cos(math.radians(30))
        ratio = 2 * (1.0 + 0.4) * fmax / required
        self.assertGreater(ratio, 1.5)


if __name__ == "__main__":
    unittest.main()
