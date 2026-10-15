"""headlights_intensity commands: clamped to 0..1, non-numbers ignored."""
import math

import pytest

from src.component_rover_leo_raph import HEADLIGHT_ZERO_ON_WIRE, headlight_level, headlight_wire_value


@pytest.mark.parametrize("value, level", [
    (0.0, 0.0), (0.35, 0.35), (1.0, 1.0),
    (-0.2, 0.0), (3.0, 1.0),                 # clamped
    ("0.5", 0.5),                             # a number given as text
])
def test_levels_are_clamped_to_the_unit_range(value, level):
    assert headlight_level(value) == pytest.approx(level)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, None, "bright"])
def test_non_numbers_are_ignored(value):
    assert headlight_level(value) is None


@pytest.mark.parametrize("level, wire", [(1.0, 1.0), (0.35, 0.35), (0.123456, 0.1235),
                                         (0.0, HEADLIGHT_ZERO_ON_WIRE), (0.00001, HEADLIGHT_ZERO_ON_WIRE)])
def test_zero_never_goes_out_as_an_exact_zero(level, wire):
    """Unreal's JSON 'Get Field' treats an exact 0 as missing, so 0 % is sent as a tiny value."""
    assert headlight_wire_value(level) == pytest.approx(wire)
    assert headlight_wire_value(level) != 0
