import math
from dataclasses import dataclass
from itertools import product

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


@dataclass
class LegIK:
    hip_offset: tuple[float, float]
    shoulder_length: float
    wrist_length: float

    def find_angles(self, hip_to_foot_vec: NDArray[np.float64], isleft: bool):
        x, y, z = hip_to_foot_vec
        off0, off1 = self.hip_offset
        if isleft:
            y = -y
        upper, lower = self.shoulder_length, self.wrist_length

        h1 = math.sqrt(off0**2 + off1**2)
        h2 = math.sqrt(z**2 + y**2)
        alpha_0 = math.atan(y / z)
        alpha_1 = math.atan(off1 / off0) if off0 > 0 else math.radians(90)
        alpha_2 = math.atan(off0 / off1)
        alpha_3 = math.asin(np.clip(h1 * math.sin(alpha_2 + math.radians(90)) / h2, -1, 1))
        alpha_4 = math.radians(180) - (alpha_3 + alpha_2 + math.radians(90))
        alpha_5 = alpha_1 - alpha_4
        theta_h = alpha_0 - alpha_5

        if not isleft:
            theta_h *= -1

        r0 = h1 * math.sin(alpha_4) / math.sin(alpha_3)
        h = math.sqrt(r0**2 + x**2)
        phi = math.asin(x / h)
        theta_s = (
            math.acos(np.clip((h**2 + upper**2 - lower**2) / (2 * h * upper), -1, 1))
            - phi
        )
        # TODO: Figure out if this should be negative
        theta_w = -math.acos(
            np.clip(-(lower**2 + upper**2 - h**2) / (2 * lower * upper), -1, 1)
        )

        return theta_h, theta_s, theta_w


class QuadropedIK:
    def __init__(
        self,
        length: float,
        width: float,
        height: float,
        hip_offset: tuple[float, float],
        shoulder_length: float,
        wrist_length: float,
        rotation: Rotation = Rotation.identity(),
        translation: NDArray[np.floating] = np.zeros((1, 3)),
    ) -> None:
        self.body_points = np.array(
            [[x * length / 2, y * width / 2, 0] for x, y in product([1, -1], [1, -1])]
        )

        self.foot_points = np.array(
            [
                [x * length / 2, y * (width / 2 + hip_offset[1]), -height]
                for x, y in product([1, -1], [1, -1])
            ]
        )

        self.isleft = [True, False, True, False]

        self.leg_ik = LegIK(hip_offset, shoulder_length, wrist_length)
        self.rotation = rotation
        self.translation = translation

    def find_angles(
        self,
        foot_points: NDArray = None,
    ):
        if foot_points is None:
            foot_points = self.foot_points

        body_points = self.body_points
        # body_points = rotation.apply(self.body_points) + translation
        foot_points = self.rotation.apply(foot_points) + self.translation
        # foot_points = self.foot_points
        hip_to_foot_vecs = foot_points - body_points
        joint_angles = [
            self.leg_ik.find_angles(htf, isleft)
            for htf, isleft in zip(hip_to_foot_vecs, self.isleft)
        ]

        return np.array(joint_angles)
