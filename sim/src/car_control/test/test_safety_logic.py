import math

import pytest

from car_control.safety_logic import SafetyMonitor, min_valid_range


# ---- min_valid_range ------------------------------------------------------

def test_min_range_picks_the_closest_valid_value():
    assert min_valid_range([1.2, 0.4, 2.0], 0.05, 4.0) == pytest.approx(0.4)


def test_min_range_ignores_inf_nan_and_out_of_range():
    ranges = [math.inf, math.nan, 0.01, 5.0, 1.5]
    assert min_valid_range(ranges, 0.05, 4.0) == pytest.approx(1.5)


def test_min_range_is_inf_when_nothing_valid():
    assert min_valid_range([math.inf, math.inf], 0.05, 4.0) == math.inf


# ---- SafetyMonitor ---------------------------------------------------------

def test_obstacle_closer_than_stop_distance_blocks():
    m = SafetyMonitor(stop_distance=0.3)
    m.on_scan(0.25, now=10.0)
    blocked, reason = m.evaluate(now=10.1)
    assert blocked
    assert 'obstacle at 0.25 m' in reason


def test_obstacle_further_away_is_clear():
    m = SafetyMonitor(stop_distance=0.3)
    m.on_scan(0.5, now=10.0)
    assert m.evaluate(now=10.1) == (False, '')


def test_no_scan_ever_received_blocks():
    m = SafetyMonitor()
    blocked, reason = m.evaluate(now=1.0)
    assert blocked
    assert reason == 'no sensor data'


def test_stale_sensor_data_blocks():
    m = SafetyMonitor(max_scan_age=0.5)
    m.on_scan(2.0, now=10.0)          # path clear ...
    assert m.evaluate(now=10.4)[0] is False
    blocked, reason = m.evaluate(now=10.6)   # ... but then the sensor goes silent
    assert blocked
    assert reason == 'no sensor data'


def test_blocked_removes_forward_motion_only():
    assert SafetyMonitor.filter_command(0.15, 0.4, blocked=True) == (0.0, 0.4)


def test_blocked_still_allows_reversing():
    assert SafetyMonitor.filter_command(-0.1, 0.0, blocked=True) == (-0.1, 0.0)


def test_not_blocked_passes_command_unchanged():
    assert SafetyMonitor.filter_command(0.15, -0.3, blocked=False) == (0.15, -0.3)
