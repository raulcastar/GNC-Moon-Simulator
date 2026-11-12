"""Steering fields: four angles [FL, BL, FR, BR] in rad, sent as SFL, SBL, SFR, SBR."""
import math

import pytest

from src.component_rover_leo_raph import STEERING_LIMIT, steering_angles, steering_fields


def test_default_is_straight():
    assert steering_fields([0.0, 0.0, 0.0, 0.0]) == {"SFL": 0.0, "SBL": 0.0, "SFR": 0.0, "SBR": 0.0}


def test_fields_follow_the_wheel_order():
    assert steering_fields([0.1, -0.1, 0.2, -0.2]) == {"SFL": 0.1, "SBL": -0.1, "SFR": 0.2, "SBR": -0.2}


def test_angles_are_clamped_to_the_joint_limit():
    assert steering_angles([2.0, -2.0, 0.3, 0]) == pytest.approx([STEERING_LIMIT, -STEERING_LIMIT, 0.3, 0.0])


@pytest.mark.parametrize("value", [[0.1, 0.2, 0.3], [0.0] * 5, [0.0, math.nan, 0.0, 0.0],
                                   [0.0, math.inf, 0.0, 0.0], ["left", 0, 0, 0], None, 0.5])
def test_anything_but_four_finite_numbers_is_rejected(value):
    assert steering_angles(value) is None
