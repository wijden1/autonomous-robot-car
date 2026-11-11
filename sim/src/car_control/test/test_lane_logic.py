import pytest

from car_control.lane_logic import (DRIVE, SLOW, STOP, LaneSteeringController,
                                    SignStateMachine)


# ---- PD lane steering -----------------------------------------------------

def test_zero_error_gives_zero_steering():
    c = LaneSteeringController(kp=1.5, kd=0.2)
    assert c.update(0.0, dt=0.05) == pytest.approx(0.0)


def test_line_on_the_right_steers_right():
    c = LaneSteeringController(kp=1.5, kd=0.0)
    assert c.update(0.4, dt=0.05) == pytest.approx(-0.6)   # -(1.5 * 0.4)


def test_line_on_the_left_steers_left():
    c = LaneSteeringController(kp=1.5, kd=0.0)
    assert c.update(-0.4, dt=0.05) > 0.0


def test_d_term_reacts_to_change_of_error():
    c = LaneSteeringController(kp=0.0, kd=0.2)
    c.update(0.0, dt=0.05)
    # error jumps by 0.1 in 50 ms -> derivative 2.0 -> turn -(0.2 * 2.0)
    assert c.update(0.1, dt=0.05) == pytest.approx(-0.4)


def test_steering_is_limited():
    c = LaneSteeringController(kp=10.0, kd=0.0, max_turn=1.5)
    assert c.update(1.0, dt=0.05) == pytest.approx(-1.5)
    assert c.update(-1.0, dt=0.05) == pytest.approx(1.5)


def test_reset_clears_previous_error():
    c = LaneSteeringController(kp=0.0, kd=0.2)
    c.update(0.5, dt=0.05)
    c.reset()
    assert c.update(0.0, dt=0.05) == pytest.approx(0.0)


# ---- Sign state machine ---------------------------------------------------

def test_starts_in_drive_at_full_speed():
    sm = SignStateMachine()
    assert sm.state == DRIVE
    assert sm.speed_factor == 1.0


def test_stop_sign_stops_then_drives_again_after_wait_time():
    sm = SignStateMachine(stop_time=3.0)
    assert sm.on_sign('stop', now=10.0) == (DRIVE, STOP)
    assert sm.speed_factor == 0.0
    assert sm.update(now=12.9) is None            # still waiting
    assert sm.state == STOP
    assert sm.update(now=13.1) == (STOP, DRIVE)   # wait time is over


def test_same_stop_sign_is_ignored_right_after_leaving():
    sm = SignStateMachine(stop_time=3.0, ignore_stop_time=4.0)
    sm.on_sign('stop', now=10.0)
    sm.update(now=13.1)                           # back to DRIVE
    assert sm.on_sign('stop', now=14.0) is None   # camera still sees it
    assert sm.state == DRIVE
    assert sm.on_sign('stop', now=17.5) == (DRIVE, STOP)   # a new stop sign later


def test_slow_sign_halves_speed_for_slow_time():
    sm = SignStateMachine(slow_time=5.0)
    assert sm.on_sign('slow', now=20.0) == (DRIVE, SLOW)
    assert sm.speed_factor == 0.5
    assert sm.update(now=25.1) == (SLOW, DRIVE)
    assert sm.speed_factor == 1.0


def test_slow_sign_triggers_only_once():
    """Regression test for a bug found in the demo: SLOW -> DRIVE -> SLOW again."""
    sm = SignStateMachine(slow_time=5.0, ignore_slow_time=5.0)
    sm.on_sign('slow', now=20.0)
    sm.update(now=25.1)
    assert sm.on_sign('slow', now=25.2) is None
    assert sm.state == DRIVE


def test_stop_sign_has_priority_over_slow_zone():
    sm = SignStateMachine()
    sm.on_sign('slow', now=20.0)
    assert sm.on_sign('stop', now=21.0) == (SLOW, STOP)


def test_no_sign_changes_nothing():
    sm = SignStateMachine()
    assert sm.on_sign('none', now=5.0) is None
    assert sm.state == DRIVE
