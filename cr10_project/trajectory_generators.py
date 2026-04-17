#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class CartesianPose:
    x: float
    y: float
    z: float
    wrist_yaw: float = 0.0

    @property
    def point(self) -> Point3:
        return (self.x, self.y, self.z)


def interpolate_scalar(start: float, end: float, steps: int) -> List[float]:
    if steps < 2:
        raise ValueError("At least two interpolation steps are required.")

    return [start + (end - start) * i / (steps - 1) for i in range(steps)]


def interpolate_line(start: Point3, end: Point3, steps: int = 50) -> List[Point3]:
    xs = interpolate_scalar(start[0], end[0], steps)
    ys = interpolate_scalar(start[1], end[1], steps)
    zs = interpolate_scalar(start[2], end[2], steps)
    return list(zip(xs, ys, zs))


def interpolate_pose(start: CartesianPose, end: CartesianPose, steps: int = 25) -> List[CartesianPose]:
    xs = interpolate_scalar(start.x, end.x, steps)
    ys = interpolate_scalar(start.y, end.y, steps)
    zs = interpolate_scalar(start.z, end.z, steps)
    yaws = interpolate_scalar(start.wrist_yaw, end.wrist_yaw, steps)
    return [
        CartesianPose(x=x, y=y, z=z, wrist_yaw=yaw)
        for x, y, z, yaw in zip(xs, ys, zs, yaws)
    ]


def build_polyline(points: Sequence[Point3], steps_per_segment: int = 25) -> List[Point3]:
    if not points:
        return []
    if len(points) == 1:
        return [tuple(points[0])]  # type: ignore[arg-type]

    polyline: List[Point3] = []
    for index in range(len(points) - 1):
        segment = interpolate_line(points[index], points[index + 1], steps_per_segment)
        if polyline:
            polyline.extend(segment[1:])
        else:
            polyline.extend(segment)
    return polyline


def circle_parallel_to_ground(
    center: Point3,
    radius: float,
    steps: int = 80,
    *,
    start_angle: float = 0.0,
    sweep_angle: float = 2.0 * math.pi,
) -> List[Point3]:
    if radius <= 0.0:
        raise ValueError("Circle radius must be positive.")
    if steps < 3:
        raise ValueError("At least three points are required to describe a circle.")

    return [
        (
            center[0] + radius * math.cos(start_angle + sweep_angle * i / (steps - 1)),
            center[1] + radius * math.sin(start_angle + sweep_angle * i / (steps - 1)),
            center[2],
        )
        for i in range(steps)
    ]


def offset_point(point: Point3, dz: float = 0.0, dx: float = 0.0, dy: float = 0.0) -> Point3:
    return (point[0] + dx, point[1] + dy, point[2] + dz)


def mirror_point(point: Sequence[float]) -> Point3:
    return (-float(point[0]), float(point[1]), float(point[2]))


def mirror_yaw(alpha: float) -> float:
    mirrored = math.pi - alpha
    return math.atan2(math.sin(mirrored), math.cos(mirrored))
