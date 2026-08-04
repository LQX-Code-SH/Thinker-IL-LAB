import unittest

import numpy as np

from constants import TASK_RESET_BODY_POSE
from dual_arm_ik import ArmIK


class DualArmIKTest(unittest.TestCase):
    def test_both_arms_recover_reset_pose_and_small_translation(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                solver = ArmIK(side)
                seed = [TASK_RESET_BODY_POSE[name] for name in solver.joint_names]
                anchor = solver.fk(seed)
                recovered = solver.solve(
                    anchor[:3, 3], anchor[:3, :3], seed,
                    max_position_error=0.001,
                    max_rotation_error=0.01,
                )
                self.assertIsNotNone(recovered)

                target = anchor[:3, 3] + np.array([0.015, 0.0, 0.010])
                moved = solver.solve(
                    target, anchor[:3, :3], recovered,
                    max_position_error=0.005,
                    max_rotation_error=0.05,
                )
                self.assertIsNotNone(moved)
                reached = solver.fk(moved)
                self.assertLess(float(np.linalg.norm(reached[:3, 3] - target)), 0.005)


if __name__ == "__main__":
    unittest.main()
