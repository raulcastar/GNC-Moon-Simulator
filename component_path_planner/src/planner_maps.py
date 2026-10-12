"""The planner's maps: loaded from CSV and turned into cost, obstacles and regions."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class PlannerMaps:
    elevation: np.ndarray       # metres, row = y, col = x
    cost: np.ndarray            # travel cost per cell, inf on obstacles
    obstacles: np.ndarray       # 1 where impassable
    free_regions: np.ndarray    # label of the 4-connected drivable region per cell (0 = none)


def load_map(path: str) -> np.ndarray:
    """A space-separated map file as a 2D array."""
    with open(path, "rb") as handle:
        return np.loadtxt(handle, delimiter=" ", skiprows=0)


def build_maps(planner, elevation: np.ndarray, soil: np.ndarray, resolution: float,
               slope_threshold: float) -> PlannerMaps:
    """Cost and obstacle maps (Global_Path_Planner.getCostMap) and the drivable regions."""
    cost, obstacles, _ = planner.getCostMap(elevation, soil, resolution, slope_threshold)
    # Drivable regions, 4-connected like the planner's search.
    _, regions = cv2.connectedComponents((obstacles != 1).astype(np.uint8), connectivity=4)
    return PlannerMaps(elevation, cost, obstacles, regions)


def load_planner_maps(planner, elevation_path: str, soil_path: str, resolution: float,
                      slope_threshold: float) -> PlannerMaps:
    return build_maps(planner, load_map(elevation_path), load_map(soil_path),
                      resolution, slope_threshold)
