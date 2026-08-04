"""C1 left/right palm IK extracted at runtime from the canonical full URDF."""
from __future__ import annotations

import copy
import os
import tempfile
import warnings
import xml.etree.ElementTree as ET
from typing import Optional, Sequence

import numpy as np

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
    ) -> Optional[list[float]]:
        target_position = np.asarray(position, dtype=float)
        target_rotation = np.asarray(rotation, dtype=float)
        try:
            solution = self.chain.inverse_kinematics(
                target_position,
                target_orientation=target_rotation,
                orientation_mode="all",
                initial_position=self._vector(self.clamp_joints(seed)),
            )
        except Exception:
            return None
        joints = self.clamp_joints([solution[index] for index in self.indices])
        if float(np.max(np.abs(np.asarray(joints) - np.asarray(seed)))) > max_solution_jump:
            return None
        reached = self.fk(joints)
        position_residual = float(np.linalg.norm(reached[:3, 3] - target_position))
        rotation_residual = rotation_error(target_rotation, reached[:3, :3])
        if position_residual > max_position_error or rotation_residual > max_rotation_error:
            return None
        return joints
