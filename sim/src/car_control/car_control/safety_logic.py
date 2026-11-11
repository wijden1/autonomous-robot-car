"""Safety rules of the car, without ROS (so they can be unit-tested)."""
import math


def min_valid_range(ranges, range_min, range_max):
    """Smallest valid distance of a laser scan; inf if nothing valid was measured."""
    valid = [r for r in ranges
             if not math.isnan(r) and range_min < r < range_max]
    return min(valid) if valid else math.inf


class SafetyMonitor:
    """Decides whether forward driving must be blocked.

    Blocked when an obstacle is closer than stop_distance, or when the
    distance sensor has been silent for longer than max_scan_age (fail-safe:
    'no data' never means 'all clear').
    """

    def __init__(self, stop_distance=0.3, max_scan_age=0.5):
        self.stop_distance = stop_distance
        self.max_scan_age = max_scan_age
        self.min_range = math.inf
        self.last_scan_time = None

    def on_scan(self, min_range, now):
        self.min_range = min_range
        self.last_scan_time = now

    def evaluate(self, now):
        """Return (blocked, reason). reason is '' when the path is clear."""
        if self.last_scan_time is None or now - self.last_scan_time >= self.max_scan_age:
            return True, 'no sensor data'
        if self.min_range < self.stop_distance:
            return True, f'obstacle at {self.min_range:.2f} m'
        return False, ''

    @staticmethod
    def filter_command(linear_x, angular_z, blocked):
        """When blocked, only forward motion is removed: reversing and turning stay allowed."""
        if blocked and linear_x > 0.0:
            linear_x = 0.0
        return linear_x, angular_z
