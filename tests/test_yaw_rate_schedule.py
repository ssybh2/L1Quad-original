"""Unit tests for independent 10%-step gain/yaw limits and physical yaw allocation."""
import unittest
import math
import numpy as np

from yaw_rate_schedule import YawRateEnvelope, allocate_pair_yaw_control
from mode29_mujoco import GainScheduler


class ToyMotor:
    def thrust(self, command):
        return 0.14 * float(command)

    def w_from_thrust(self, thrust):
        return max(0., min(100., float(thrust) / 0.14))

    def moment(self, command):
        return 0.006 * float(command)


class ToyMixer:
    motor = ToyMotor()
    L = 0.28
    D = 0.28


class IndependentAnchorTests(unittest.TestCase):
    def setUp(self):
        self.base = dict(kpx=4., kpy=4., kpz=10., kvx=4., kvy=4., kvz=2.,
                         krx=1., kry=0.5, krz=0.25, kox=0.1, koy=0.2, koz=0.1)
        self.nodes = {
            "mode": "oracle",
            "loss_0": {"kp": [4., 4., 10.], "kv": [4., 4., 2.],
                       "kr": [1., 0.5, 0.25], "ko": [0.1, 0.2, 0.1],
                       "max_tilt_deg": 30., "max_yaw_rate_deg_s": 150.},
            "loss_30": {"kp": [8., 8., 10.], "kv": [5., 6., 2.],
                        "kr": [1., 0.5, 0.25], "ko": [0.2, 0.2, 0.1],
                        "max_tilt_deg": 30., "max_yaw_rate_deg_s": 80.},
            "loss_40": {"kp": [20., 20., 10.], "kv": [8., 8., 2.],
                        "kr": [0.5, 0.5, 0.25], "ko": [1.5, 1.5, 0.1],
                        "max_tilt_deg": 30., "max_yaw_rate_deg_s": 120.},
        }

    def test_each_anchor_can_have_distinct_gain_values(self):
        scheduler = GainScheduler(self.nodes, self.base, {"max_tilt_deg": 30.})
        self.assertAlmostEqual(scheduler.at_loss(30)["kp"][0], 8.)
        self.assertAlmostEqual(scheduler.at_loss(40)["kp"][0], 20.)
        self.assertAlmostEqual(scheduler.at_loss(37)["kp"][0], 16.4)
        self.assertAlmostEqual(scheduler.at_loss(37)["kv"][0], 7.1)
        self.assertAlmostEqual(scheduler.at_loss(37)["ko"][0], 1.11)
        self.assertAlmostEqual(scheduler.at_loss(0)["kp"][0], 4.)

    def test_each_anchor_can_have_distinct_yaw_speed_limit(self):
        speed = YawRateEnvelope({"enabled": True}, self.nodes)
        self.assertAlmostEqual(speed.limit_deg_s(0), 150.)
        self.assertAlmostEqual(speed.limit_deg_s(30), 80.)
        self.assertAlmostEqual(speed.limit_deg_s(40), 120.)
        self.assertAlmostEqual(speed.limit_deg_s(37), 108.)

    def test_anticipatory_braking_preserves_heading_freedom(self):
        speed = YawRateEnvelope({
            "enabled": True,
            "brake_start_fraction": .75,
            "rate_gain_nm_per_rps": .08,
            "max_corrective_moment_nm": .2
        }, self.nodes)
        self.assertEqual(speed.requested_moment(30, 0.), 0.)
        self.assertEqual(speed.requested_moment(30, math.radians(45)), 0.)
        self.assertLess(speed.requested_moment(30, math.radians(100)), 0.)
        self.assertGreater(speed.requested_moment(30, math.radians(-100)), 0.)

    def test_primary_wrench_feasible_with_yaw_priority_second(self):
        mixer = ToyMixer()
        for requested_yaw in (-0.1, 0.0, 0.1):
            cmd, diagnostics = allocate_pair_yaw_control(mixer, [10., 0., 0.], 1, 50., requested_yaw)
            self.assertEqual(cmd.shape, (4,))
            self.assertTrue(np.isfinite(cmd).all())
            actual = np.array([mixer.motor.thrust(w) for w in cmd]) * [0.5, 0.5, 1., 1.]
            self.assertAlmostEqual(sum(actual), 10., places=5)
            roll = .14 * (-actual[0] + actual[1] + actual[2] - actual[3])
            pitch = .14 * (actual[0] - actual[1] + actual[2] - actual[3])
            self.assertAlmostEqual(roll, 0., places=5)
            self.assertAlmostEqual(pitch, 0., places=5)
            self.assertTrue(diagnostics["primary_wrench_feasible"])

    def test_total_opposite_loss_reports_rank_deficiency(self):
        with self.assertRaisesRegex(RuntimeError, "uncontrollable"):
            allocate_pair_yaw_control(ToyMixer(), [10., 0., 0.], 1, 100., 0.)


if __name__ == "__main__":
    unittest.main()
