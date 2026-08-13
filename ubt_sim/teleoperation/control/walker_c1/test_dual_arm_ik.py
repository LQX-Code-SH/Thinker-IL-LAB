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

    def test_distant_target_is_advanced_by_a_small_cartesian_step(self):
        solver = ArmIK("right")
        seed = [TASK_RESET_BODY_POSE[name] for name in solver.joint_names]
        anchor = solver.fk(seed)
        distant_target = anchor[:3, 3] + np.array([0.12, 0.0, 0.0])
        moved = solver.solve(
            distant_target,
            anchor[:3, :3],
            seed,
            max_position_error=0.005,
            max_rotation_error=0.05,
            max_solution_jump=0.35,
            max_target_position_step=0.012,
            max_target_rotation_step=0.08,
        )
        self.assertIsNotNone(moved, solver.last_rejection_reason)
        reached = solver.fk(moved)
        step = float(np.linalg.norm(reached[:3, 3] - anchor[:3, 3]))
        self.assertGreater(step, 0.004)
        self.assertLess(step, 0.017)

    def test_commanded_elbow_never_crosses_straight_arm_boundary(self):
        solver = ArmIK("right")
        seed = [TASK_RESET_BODY_POSE[name] for name in solver.joint_names]
        straight_target_joints = list(seed)
        straight_target_joints[solver.elbow_pitch_index] = -0.05
        straight_target = solver.fk(straight_target_joints)
        limit = -0.50
        solved = solver.solve(
            straight_target[:3, 3],
            straight_target[:3, :3],
            seed,
            max_position_error=1.0,
            max_rotation_error=3.2,
            max_solution_jump=2.0,
            orientation_weight=0.1,
            seed_regularization_weight=0.0,
            max_nfev=100,
            max_elbow_pitch=limit,
        )
        self.assertIsNotNone(solved, solver.last_rejection_reason)
        self.assertLessEqual(solved[solver.elbow_pitch_index], limit + 1e-6)

if __name__ == "__main__":
    unittest.main()
