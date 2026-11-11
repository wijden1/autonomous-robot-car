"""Lane steering and sign state machine, without ROS (so they can be unit-tested)."""

DRIVE, SLOW, STOP = 'DRIVE', 'SLOW', 'STOP'


class LaneSteeringController:
    """PD controller: lane offset (-1 left .. +1 right) -> turn rate (rad/s).

    A line on the right (positive offset) gives a negative turn rate,
    which turns the car to the right, towards the line.
    """

    def __init__(self, kp=1.5, kd=0.2, max_turn=1.5):
        self.kp = kp
        self.kd = kd
        self.max_turn = max_turn
        self.prev_error = 0.0

    def reset(self):
        self.prev_error = 0.0

    def update(self, offset, dt):
        derivative = (offset - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = offset
        turn = -(self.kp * offset + self.kd * derivative)
        return max(-self.max_turn, min(self.max_turn, turn))


class SignStateMachine:
    """DRIVE -> STOP (wait) -> DRIVE at stop signs, DRIVE -> SLOW -> DRIVE at slow signs.

    After leaving STOP or SLOW, the same sign is ignored for a while,
    because the camera still sees it while the car drives past.
    """

    def __init__(self, stop_time=3.0, slow_time=5.0, ignore_stop_time=4.0, ignore_slow_time=5.0):
        self.stop_time = stop_time
        self.slow_time = slow_time
        self.ignore_stop_time = ignore_stop_time
        self.ignore_slow_time = ignore_slow_time
        self.state = DRIVE
        self.state_until = 0.0
        self.ignore_stop_until = 0.0
        self.ignore_slow_until = 0.0

    def _set(self, new_state, now, duration=0.0):
        old = self.state
        self.state = new_state
        self.state_until = now + duration
        return (old, new_state) if old != new_state else None

    def on_sign(self, sign, now):
        """Feed a detected sign ('stop', 'slow' or 'none'). Returns (old, new) on a change."""
        if sign == 'stop' and self.state != STOP and now > self.ignore_stop_until:
            return self._set(STOP, now, self.stop_time)
        if sign == 'slow' and self.state == DRIVE and now > self.ignore_slow_until:
            return self._set(SLOW, now, self.slow_time)
        return None

    def update(self, now):
        """Call regularly: ends STOP/SLOW when their time is up. Returns (old, new) on a change."""
        if self.state == STOP and now > self.state_until:
            self.ignore_stop_until = now + self.ignore_stop_time
            return self._set(DRIVE, now)
        if self.state == SLOW and now > self.state_until:
            self.ignore_slow_until = now + self.ignore_slow_time
            return self._set(DRIVE, now)
        return None

    @property
    def speed_factor(self):
        return {DRIVE: 1.0, SLOW: 0.5, STOP: 0.0}[self.state]
