"""Common interface for trajectory construction methods."""
from __future__ import annotations

from typing import Protocol


class TrajectoryMethod(Protocol):
    name: str

    def solve(self, last_data: dict, target: dict, **parameters) -> list[dict]:
        """Return trajectory points in the shared point schema."""
        ...
