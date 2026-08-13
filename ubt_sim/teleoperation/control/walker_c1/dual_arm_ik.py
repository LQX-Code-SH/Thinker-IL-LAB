"""C1 left/right palm IK extracted at runtime from the canonical full URDF."""
from __future__ import annotations

import copy
import os
import tempfile
import warnings
import xml.etree.ElementTree as ET
from typing import Optional, Sequence

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    from ikpy.chain import Chain

try:
    from .constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES
    from .pico_math import rotation_error
except ImportError:
    from constants import LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES
    from pico_math import rotation_error


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FULL_URDF = os.path.abspath(os.path.join(
    _THIS_DIR, "..", "..", "..", "assets", "robots", "walker_c1",
    "walker_astron_v2_hand_v3_no_sixforce_mesh.urdf",
))


def _chain_urdf(source_path: str, tip_link: str) -> bytes:
    root = ET.parse(source_path).getroot()
    joints_by_child = {}
    links_by_name = {link.attrib["name"]: link for link in root.findall("link")}
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None:
            joints_by_child[child.attrib["link"]] = joint

    joints = []
    links = [tip_link]
    child_name = tip_link
    while child_name != "base_link":
        if child_name not in joints_by_child:
            raise ValueError(f"cannot find URDF path base_link -> {tip_link}")
        joint = joints_by_child[child_name]
        joints.append(joint)
        child_name = joint.find("parent").attrib["link"]
        links.append(child_name)
    joints.reverse()
    links.reverse()

    trimmed = ET.Element("robot", {"name": f"walker_c1_{tip_link}_chain"})
    for name in links:
        if name not in links_by_name:
            raise ValueError(f"URDF link {name!r} is missing")
        # Kinematics only needs named links; omitting meshes also avoids path warnings.
        ET.SubElement(trimmed, "link", {"name": name})
    for joint in joints:
        trimmed.append(copy.deepcopy(joint))
    return ET.tostring(trimmed, encoding="utf-8", xml_declaration=True)


class ArmIK:
    def __init__(self, side: str, urdf_path: str = DEFAULT_FULL_URDF):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        prefix = "L" if side == "left" else "R"
        self.joint_names = LEFT_ARM_JOINT_NAMES if side == "left" else RIGHT_ARM_JOINT_NAMES
        payload = _chain_urdf(urdf_path, f"{prefix}_palm_link")
        path = ""
        try:
            with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as handle:
                handle.write(payload)
                path = handle.name
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                self.chain = Chain.from_urdf_file(path, base_elements=["base_link"])
        finally:
            if path:
                os.unlink(path)

        link_names = [link.name for link in self.chain.links]
        self.indices = [link_names.index(name) for name in self.joint_names]
        mask = np.zeros(len(self.chain.links), dtype=bool)
        mask[self.indices] = True
        self.chain.active_links_mask = mask
        self.bounds = [self.chain.links[index].bounds for index in self.indices]
        self.last_rejection_reason = ""

    def clamp_joints(self, joints: Sequence[float]) -> list[float]:
        result = []
        for value, bounds in zip(joints, self.bounds):
            low, high = bounds
            result.append(float(np.clip(value, float(low) + 1e-4, float(high) - 1e-4)))
        return result

    def _vector(self, joints: Sequence[float]) -> np.ndarray:
        vector = np.zeros(len(self.chain.links))
        for index, value in zip(self.indices, joints):
            vector[index] = value
        return vector

    def fk(self, joints: Sequence[float]) -> np.ndarray:
        return self.chain.forward_kinematics(self._vector(joints))

    def solve(
        self,
        position: Sequence[float],
        rotation: np.ndarray,
        seed: Sequence[float],
        max_position_error: float,
        max_rotation_error: float,
        max_solution_jump: float = float("inf"),
        orientation_weight: float = 0.10,
        seed_regularization_weight: float = 0.02,
        max_nfev: int = 60,
    ) -> Optional[list[float]]:
        self.last_rejection_reason = ""
        target_position = np.asarray(position, dtype=float)
        target_rotation = np.asarray(rotation, dtype=float)
        seed_joints = np.asarray(self.clamp_joints(seed), dtype=float)
        lower_bounds = np.asarray([float(bounds[0]) + 1e-4 for bounds in self.bounds])
        upper_bounds = np.asarray([float(bounds[1]) - 1e-4 for bounds in self.bounds])

        def residuals(joints: np.ndarray) -> np.ndarray:
            reached = self.fk(joints)
            position_error = reached[:3, 3] - target_position
            rotation_error_vector = Rotation.from_matrix(
                target_rotation.T @ reached[:3, :3]
            ).as_rotvec()
            seed_error = joints - seed_joints
            return np.concatenate((
                position_error,
                float(orientation_weight) * rotation_error_vector,
                float(seed_regularization_weight) * seed_error,
            ))

        try:
            result = least_squares(
                residuals,
                seed_joints,
                bounds=(lower_bounds, upper_bounds),
                max_nfev=int(max_nfev),
                ftol=1e-6,
                xtol=1e-6,
                gtol=1e-6,
            )
        except Exception as exc:
            self.last_rejection_reason = f"solver error: {exc}"
            return None
        joints = self.clamp_joints(result.x)
        solution_jump = float(np.max(np.abs(np.asarray(joints) - np.asarray(seed))))
        if solution_jump > max_solution_jump:
            self.last_rejection_reason = (
                f"solution jump {solution_jump:.3f} rad > {max_solution_jump:.3f} rad"
            )
            return None
        reached = self.fk(joints)
        position_residual = float(np.linalg.norm(reached[:3, 3] - target_position))
        rotation_residual = rotation_error(target_rotation, reached[:3, :3])
        if position_residual > max_position_error or rotation_residual > max_rotation_error:
            self.last_rejection_reason = (
                f"residual position={position_residual:.4f} m/{max_position_error:.4f}, "
                f"rotation={rotation_residual:.3f} rad/{max_rotation_error:.3f}"
            )
            return None
        return joints
