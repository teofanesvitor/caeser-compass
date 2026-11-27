"""Minimum Curvature trajectory calculations.

Azimuth convention: 0 degrees is North and increases clockwise.
"""
from __future__ import annotations

import numpy as np


def normalize_angle_deg(value: float) -> float:
    return float(value % 360.0)


def target_reached(point: dict, target: dict, tolerance: float = 1e-3) -> bool:
    if target is None:
        return False
    return all(
        abs(float(point[key]) - float(target[target_key])) <= tolerance
        for key, target_key in (
            ("verticalDepth", "tvd"), ("ns", "ns"), ("ew", "ew")
        )
    )


class WellTrajectory:
    """Compute a survey using the Minimum Curvature method."""

    def __init__(
        self, md, inc, azi, *, tvd_offset=0.0, north_offset=0.0,
        east_offset=0.0, vs_azimuth=None
    ):
        if not (len(md) == len(inc) == len(azi)):
            raise ValueError("MD, INC e AZI devem possuir o mesmo tamanho.")
        if len(md) < 2:
            raise ValueError("A trajetória precisa de pelo menos dois stations.")

        self.md = np.asarray(md, dtype=float)
        self.inc = np.radians(np.asarray(inc, dtype=float))
        self.azi = np.radians(np.asarray(azi, dtype=float))
        if np.any(np.diff(self.md) <= 0):
            raise ValueError("MD deve ser estritamente crescente.")

        self._vs_azimuth_input = vs_azimuth
        self._compute(tvd_offset, north_offset, east_offset)

    def _compute(self, tvd_offset, north_offset, east_offset):
        n = len(self.md)
        self.north = np.zeros(n)
        self.east = np.zeros(n)
        self.tvd = np.zeros(n)
        self.north[0], self.east[0], self.tvd[0] = (
            float(north_offset), float(east_offset), float(tvd_offset)
        )
        self.dls = np.zeros(n)
        self.build = np.zeros(n)
        self.turn = np.zeros(n)
        self.toolface = np.zeros(n)

        for i in range(1, n):
            dmd = self.md[i] - self.md[i - 1]
            i1, i2 = self.inc[i - 1], self.inc[i]
            a1, a2 = self.azi[i - 1], self.azi[i]
            dogleg = np.arccos(np.clip(
                np.cos(i1) * np.cos(i2)
                + np.sin(i1) * np.sin(i2) * np.cos(a2 - a1), -1.0, 1.0
            ))
            rf = 1.0 if abs(dogleg) < 1e-10 else 2.0 / dogleg * np.tan(dogleg / 2.0)
            self.north[i] = self.north[i-1] + 0.5*dmd*(np.sin(i1)*np.cos(a1)+np.sin(i2)*np.cos(a2))*rf
            self.east[i] = self.east[i-1] + 0.5*dmd*(np.sin(i1)*np.sin(a1)+np.sin(i2)*np.sin(a2))*rf
            self.tvd[i] = self.tvd[i-1] + 0.5*dmd*(np.cos(i1)+np.cos(i2))*rf
            self.dls[i] = np.degrees(dogleg) * 30.0 / dmd
            self.build[i] = np.degrees(i2-i1) * 30.0 / dmd

            vertical_start = abs(i1) < 1e-10
            vertical_end = abs(i2) < 1e-10
            delta_azi = (np.degrees(a2-a1) + 180.0) % 360.0 - 180.0
            self.turn[i] = 0.0 if vertical_start or vertical_end else delta_azi * 30.0 / dmd

            if vertical_start:
                self.toolface[i] = normalize_angle_deg(np.degrees(a2))
            elif dogleg < 1e-3:
                self.toolface[i] = 0.0
            else:
                den = np.sin(i1) * np.sin(dogleg)
                if abs(den) < 1e-10:
                    self.toolface[i] = 0.0
                else:
                    arg = np.clip((np.cos(i1)*np.cos(dogleg)-np.cos(i2))/den, -1.0, 1.0)
                    tf = np.degrees(np.arccos(arg))
                    self.toolface[i] = normalize_angle_deg(tf) if vertical_end else np.sign(self.turn[i])*tf

        self.horizontal_disp = np.hypot(self.north, self.east)
        self.traj_azimuth = np.arctan2(self.east, self.north)
        self.vs_azimuth = (
            np.radians(self._vs_azimuth_input) if self._vs_azimuth_input is not None
            else np.arctan2(self.east[-1]-self.east[0], self.north[-1]-self.north[0])
        )
        self.vs = self.north*np.cos(self.vs_azimuth) + self.east*np.sin(self.vs_azimuth)

    def results(self):
        return {
            "MD": self.md.copy(),
            "Inclination": np.degrees(self.inc),
            "Azimuth": np.degrees(self.azi),
            "North": self.north.copy(),
            "East": self.east.copy(),
            "TVD": self.tvd.copy(),
            "Horizontal Displacement": self.horizontal_disp.copy(),
            "Trajectory Azimuth (deg)": np.degrees(self.traj_azimuth),
            "VS Azimuth (deg)": float(np.degrees(self.vs_azimuth)),
            "Vertical Section": self.vs.copy(),
            "DLS (deg/30m)": self.dls.copy(),
            "Build (deg/30m)": self.build.copy(),
            "Turn (deg/30m)": self.turn.copy(),
            "Toolface (deg)": self.toolface.copy(),
        }

def spatial_angle(theta1: float, phi1: float, theta2: float, phi2: float) -> float:
    """Angular separation between two wellbore directions in radians."""
    value = (
        np.cos(theta1) * np.cos(theta2)
        + np.sin(theta1) * np.sin(theta2) * np.cos(phi2 - phi1)
    )
    return float(np.arccos(np.clip(value, -1.0, 1.0)))


def gamma_factor(length: float, dogleg: float) -> float:
    """Minimum Curvature gamma factor: half-length multiplied by RF."""
    if abs(dogleg) < 1e-12:
        return float(length) / 2.0
    return float(length) / dogleg * np.tan(dogleg / 2.0)


def deg2rad(value: float) -> float:
    return float(np.radians(value))


def rad2deg(value: float) -> float:
    return float(np.degrees(value))


def normalize_angle_rad(value: float) -> float:
    return float(value % (2.0 * np.pi))
