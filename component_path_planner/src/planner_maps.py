"""
The global planner's maps, from the elevation and soil maps. No ROS.

    slope       from the elevation's gradient (degrees)
    obstacles   1 where impassable: slope over the threshold, or no-go soil (0)
    cost        travel cost per cell: rises with slope (SLOPE_COST), impassable
                cells cost 2 x the threshold; smoothed over 3 x 3 cells
    regions     label of the 4-connected drivable region of each cell (0 = none)
"""

from dataclasses import dataclass

import cv2
import numpy as np

# Travel cost at these slopes (degrees), linear in between; SLOPE_COST_MAX at
# and beyond the last slope.
SLOPE_COST_DEG = np.array([0.0, 5.0, 10.0, 15.0, 20.0])
SLOPE_COST = np.array([10.0, 12.0, 15.0, 20.0, 40.0])
SLOPE_COST_MAX = 50.0


@dataclass
class PlannerMaps:
    elevation: np.ndarray       # metres, row = y, col = x
    cost: np.ndarray            # travel cost per cell (finite everywhere)
    obstacles: np.ndarray       # 1 where impassable
    free_regions: np.ndarray    # label of the 4-connected drivable region per cell (0 = none)


def load_map(path: str) -> np.ndarray:
    """A space-separated map file as a 2D array."""
    with open(path, "rb") as handle:
        return np.loadtxt(handle, delimiter=" ", skiprows=0)


def slope_deg(elevation: np.ndarray, resolution: float) -> np.ndarray:
    """Slope of every cell (degrees) from the elevation's gradient; 0 where undefined."""
    dx, dy = np.gradient(elevation, resolution, resolution)
    slope = (180 / np.pi) * np.arctan(np.sqrt(dx ** 2 + dy ** 2))
    slope[np.isnan(slope)] = 0
    return slope


def slope_cost(slope: np.ndarray) -> np.ndarray:
    """Travel cost of each slope: linear between the SLOPE_COST points, SLOPE_COST_MAX beyond."""
    i = np.searchsorted(SLOPE_COST_DEG, slope, side='left')
    i = np.minimum(i, len(SLOPE_COST_DEG) - 1)
    x0, x1 = SLOPE_COST_DEG[i - 1], SLOPE_COST_DEG[i]
    y0, y1 = SLOPE_COST[i - 1], SLOPE_COST[i]
    fraction = (slope - x0) / (x1 - x0)
    cost = fraction * (y1 - y0) + y0
    return np.where(slope >= SLOPE_COST_DEG[-1], SLOPE_COST_MAX, cost)


def build_maps(elevation: np.ndarray, soil: np.ndarray, resolution: float,
               slope_threshold: float) -> PlannerMaps:
    """The cost and obstacle maps and the drivable regions."""
    slope = slope_deg(elevation, resolution)
    obstacles = np.zeros_like(elevation)
    obstacles[slope > slope_threshold] = 1
    obstacles[soil == 0] = 1

    cost = slope_cost(slope)
    cost[obstacles == 1] = 2 * slope_threshold
    cost = cv2.blur(cost, (3, 3))

    # Drivable regions, 4-connected like the search.
    _, regions = cv2.connectedComponents((obstacles != 1).astype(np.uint8), connectivity=4)
    return PlannerMaps(elevation, cost, obstacles, regions)


def load_planner_maps(elevation_path: str, soil_path: str, resolution: float,
                      slope_threshold: float) -> PlannerMaps:
    return build_maps(load_map(elevation_path), load_map(soil_path), resolution, slope_threshold)
