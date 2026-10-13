"""
The optional capture time in Unreal's telemetry ('t_ns': simulation time in ns
since the Unix epoch, from UnrealSimClock's Get Sim Time Ns String), which
odometry is stamped with instead of its arrival time.
"""
from src.component_rover_leo_raph import stamp_ns_from_telemetry

NS = 1791816533694094831          # 19 digits: more than a double holds exactly


def test_text_keeps_every_nanosecond():
    assert stamp_ns_from_telemetry({'t_ns': str(NS)}) == NS
    assert stamp_ns_from_telemetry({'t_ns': f' {NS} '}) == NS


def test_an_integer_works_too():
    assert stamp_ns_from_telemetry({'t_ns': NS}) == NS


def test_a_number_with_a_decimal_point_is_rounded():
    assert stamp_ns_from_telemetry({'t_ns': '1791816533694094000.0'}) == 1791816533694094000


def test_missing_or_unreadable_means_stamp_on_arrival():
    assert stamp_ns_from_telemetry({}) is None
    assert stamp_ns_from_telemetry({'t_ns': ''}) is None
    assert stamp_ns_from_telemetry({'t_ns': 'soon'}) is None
    assert stamp_ns_from_telemetry({'t_ns': 0}) is None
    assert stamp_ns_from_telemetry({'t_ns': True}) is None
