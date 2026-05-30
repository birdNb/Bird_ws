#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串联臂正向运动学。"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np

from .transforms import axis_angle_matrix, origin_matrix
from .urdf_parser import ArmChain


class ForwardKinematics:
    def __init__(self, chain: ArmChain):
        self.chain = chain
        self._static: List[np.ndarray] = [
            origin_matrix(j.origin_xyz, j.origin_rpy) for j in chain.joints
        ]

    def joint_transform(self, index: int, q: float) -> np.ndarray:
        j = self.chain.joints[index]
        return self._static[index] @ axis_angle_matrix(j.axis, q)

    def compute(self, q: np.ndarray) -> np.ndarray:
        """T_base_ee, 4x4。"""
        if q.shape[0] != self.chain.dof:
            raise ValueError(f"期望 {self.chain.dof} 关节角，得到 {q.shape[0]}")
        t = np.eye(4, dtype=float)
        for i in range(self.chain.dof):
            t = t @ self.joint_transform(i, float(q[i]))
        return t

    def joint_positions(self, q: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray]]:
        """各关节坐标系原点在世界（基座）下的位置与旋转。"""
        frames: List[Tuple[np.ndarray, np.ndarray]] = []
        t = np.eye(4, dtype=float)
        for i in range(self.chain.dof):
            t = t @ self.joint_transform(i, float(q[i]))
            frames.append((t[:3, 3].copy(), t[:3, :3].copy()))
        return frames
