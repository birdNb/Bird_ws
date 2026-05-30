#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
5-DOF 数值逆解（阻尼最小二乘）。

5 自由度无法同时精确满足 6 维位姿，支持：
  - position: 仅位置
  - position_tool_z: 位置 + 末端 z 轴（5 约束，推荐）
  - position_orientation: 位置 + 完整姿态（软约束）
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from .forward_kinematics import ForwardKinematics
from .transforms import (
    clamp_vector,
    rotation_error,
    split_pose,
    tool_z_error,
)
from .urdf_parser import ArmChain


class IKTaskMode(str, Enum):
    POSITION = "position"
    POSITION_TOOL_Z = "position_tool_z"
    POSITION_ORIENTATION = "position_orientation"


@dataclass
class IKConfig:
    max_iterations: int = 120
    position_tolerance_m: float = 2e-3
    orientation_tolerance_rad: float = 0.05
    damping: float = 0.08
    step_scale: float = 0.85
    position_weight: float = 1.0
    orientation_weight: float = 0.35
    jacobian_delta: float = 1e-4


@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    message: str


class NumericalIK:
    def __init__(
        self,
        fk: ForwardKinematics,
        chain: ArmChain,
        config: Optional[IKConfig] = None,
    ):
        self.fk = fk
        self.chain = chain
        self.cfg = config or IKConfig()
        self.q_lo, self.q_hi = chain.limits()

    def solve(
        self,
        target_T: np.ndarray,
        q_seed: Optional[np.ndarray] = None,
        mode: IKTaskMode = IKTaskMode.POSITION_TOOL_Z,
    ) -> IKResult:
        n = self.chain.dof
        q = (
            np.zeros(n, dtype=float)
            if q_seed is None
            else np.asarray(q_seed, dtype=float).copy()
        )
        q = clamp_vector(q, self.q_lo, self.q_hi)
        p_tgt, r_tgt = split_pose(target_T)

        best_q = q.copy()
        best_pos = float("inf")
        best_ori = float("inf")

        for it in range(1, self.cfg.max_iterations + 1):
            t_cur = self.fk.compute(q)
            p_cur, r_cur = split_pose(t_cur)
            e_task, pos_err, ori_err = self._task_error(
                p_cur, r_cur, p_tgt, r_tgt, mode,
            )

            pos_ok = pos_err < self.cfg.position_tolerance_m
            ori_ok = (
                mode == IKTaskMode.POSITION
                or ori_err < self.cfg.orientation_tolerance_rad
            )
            if pos_ok and ori_ok:
                return IKResult(
                    success=True,
                    q=q,
                    iterations=it,
                    position_error_m=pos_err,
                    orientation_error_rad=ori_err,
                    message="converged",
                )

            j = self._numeric_jacobian(q, p_tgt, r_tgt, mode)
            jj_t = j @ j.T
            lam2 = self.cfg.damping ** 2
            try:
                # e = x_des - x_cur，J = d(x)/dq → x_new ≈ x + J*dq，令 J*dq ≈ e
                dq = j.T @ np.linalg.solve(
                    jj_t + lam2 * np.eye(jj_t.shape[0]), e_task,
                )
            except np.linalg.LinAlgError:
                return IKResult(
                    success=False,
                    q=best_q,
                    iterations=it,
                    position_error_m=best_pos,
                    orientation_error_rad=best_ori,
                    message="singular_jacobian",
                )
            best_step_q = q.copy()
            best_step_pos = pos_err
            for scale in (1.0, 0.5, 0.25):
                q_try = clamp_vector(
                    q + self.cfg.step_scale * scale * dq,
                    self.q_lo,
                    self.q_hi,
                )
                t_try = self.fk.compute(q_try)
                p_try, r_try = split_pose(t_try)
                _, pe_try, _ = self._task_error(
                    p_try, r_try, p_tgt, r_tgt, mode,
                )
                if pe_try < best_step_pos:
                    best_step_pos = pe_try
                    best_step_q = q_try
            q = best_step_q
            t_new = self.fk.compute(q)
            p_new, r_new = split_pose(t_new)
            _, pos_new, ori_new = self._task_error(
                p_new, r_new, p_tgt, r_tgt, mode,
            )
            if pos_new < best_pos:
                best_pos = pos_new
                best_ori = ori_new
                best_q = q.copy()

        return IKResult(
            success=False,
            q=best_q,
            iterations=self.cfg.max_iterations,
            position_error_m=best_pos,
            orientation_error_rad=best_ori,
            message="max_iterations",
        )

    def _task_error(
        self,
        p_cur: np.ndarray,
        r_cur: np.ndarray,
        p_tgt: np.ndarray,
        r_tgt: np.ndarray,
        mode: IKTaskMode,
    ) -> tuple[np.ndarray, float, float]:
        e_pos = (p_tgt - p_cur) * self.cfg.position_weight
        pos_err = float(np.linalg.norm(e_pos))
        if mode == IKTaskMode.POSITION:
            return e_pos, pos_err, 0.0
        if mode == IKTaskMode.POSITION_TOOL_Z:
            e_ori = tool_z_error(r_cur, r_tgt) * self.cfg.orientation_weight
            ori_err = float(np.linalg.norm(e_ori))
            return np.concatenate([e_pos, e_ori]), pos_err, ori_err
        e_ori = rotation_error(r_cur, r_tgt) * self.cfg.orientation_weight
        ori_err = float(np.linalg.norm(e_ori))
        return np.concatenate([e_pos, e_ori]), pos_err, ori_err

    def _numeric_jacobian(
        self,
        q: np.ndarray,
        p_tgt: np.ndarray,
        r_tgt: np.ndarray,
        mode: IKTaskMode,
    ) -> np.ndarray:
        """几何雅可比 d(末端位姿)/dq；任务误差 e 满足 e_dot ≈ -J * dq。"""
        n = self.chain.dof
        t0 = self.fk.compute(q)
        p0, r0 = split_pose(t0)
        h = self.cfg.jacobian_delta
        if mode == IKTaskMode.POSITION:
            m = 3
        else:
            # position(3) + tool_z(3) 或 rotation(3)
            m = 6
        j = np.zeros((m, n), dtype=float)
        for i in range(n):
            dq = np.zeros(n, dtype=float)
            dq[i] = h
            t1 = self.fk.compute(q + dq)
            p1, r1 = split_pose(t1)
            j[0:3, i] = (p1 - p0) / h
            if mode == IKTaskMode.POSITION_TOOL_Z:
                j[3:6, i] = (tool_z_error(r1, r0)) / h
            elif mode == IKTaskMode.POSITION_ORIENTATION:
                j[3:6, i] = (rotation_error(r1, r0)) / h
        if mode == IKTaskMode.POSITION:
            j *= self.cfg.position_weight
        elif mode == IKTaskMode.POSITION_TOOL_Z:
            j[0:3] *= self.cfg.position_weight
            j[3:6] *= self.cfg.orientation_weight
        else:
            j[0:3] *= self.cfg.position_weight
            j[3:6] *= self.cfg.orientation_weight
        return j
