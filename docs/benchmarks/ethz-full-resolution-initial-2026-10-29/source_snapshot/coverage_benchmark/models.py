from dataclasses import dataclass, field
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class PlanningInput:
    map_id: str
    free: np.ndarray
    navigable: np.ndarray
    resolution: float
    start: tuple[float, float]
    tool_width: float
    clearance: float
    source_root: Path
    runtime_root: Path


@dataclass
class PlanningResult:
    path: list[list[float]] = field(default_factory=list)
    success: bool = False
    message: str = ''
    details: dict = field(default_factory=dict)
