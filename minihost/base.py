"""Omni base and lift: turning what the operator wants into servo velocities.

Numbers for the alohamini1 (from the upstream model_specs): three omni wheels
on a 0.125 m radius, wheel radius 0.05 m, servo ids 8 (left), 9 (back),
10 (right) on the left arm's bus. Lift is servo 11, a lead screw at 84 mm per
revolution, also run in velocity mode. All of these are parameters here
because alohamini2 differs and because I have not measured any of them.

Both wheels and lift run the servo in velocity mode (Operating_Mode = 1):
you send a signed speed in ticks per second (4096 ticks = one revolution) and
the servo holds that speed. Sending zero stops it.
"""
import math

WHEEL_IDS = (8, 9, 10)
LIFT_ID = 11
TICKS_PER_REV = 4096


class OmniBase:
    def __init__(self, wheel_radius=0.05, base_radius=0.125, max_ticks_s=3000):
        self.wheel_radius = wheel_radius
        self.base_radius = base_radius
        self.max = max_ticks_s
        # Wheel mounting angles. Upstream uses 240, 0, 120 degrees minus 90,
        # i.e. the back wheel points along -y. I kept their convention so the
        # existing keyboard mapping (x forward, y left, theta ccw) still holds.
        self.angles = [math.radians(a - 90) for a in (240, 0, 120)]

    def wheel_speeds(self, vx, vy, wz):
        """Body velocities (m/s, m/s, rad/s) -> {id: ticks/s} for the three wheels.

        For an omni wheel at angle a and distance R from the centre, the rim
        speed it must produce is  v = vx*cos(a) + vy*sin(a) + wz*R. Divide by
        wheel radius for rad/s, then into ticks/s. Upstream negates x and y;
        I kept that too, so forward on the keyboard is forward on the robot
        (assumption until driven).
        """
        speeds = {}
        for i, a in zip(WHEEL_IDS, self.angles):
            rim = -vx * math.cos(a) + -vy * math.sin(a) + wz * self.base_radius
            speeds[i] = rim / self.wheel_radius / (2 * math.pi) * TICKS_PER_REV
        # if any wheel is asked for more than it can do, slow all of them by the
        # same factor so the base still goes in the commanded direction
        peak = max(abs(v) for v in speeds.values())
        if peak > self.max:
            speeds = {i: v * self.max / peak for i, v in speeds.items()}
        return {i: int(round(v)) for i, v in speeds.items()}


class Lift:
    """Height is tracked by unwrapping the servo's 0..4095 position across turns.

    The servo only knows its angle, not how many turns it has done, so we
    count turns ourselves from consecutive reads: if the angle jumped by more
    than half a revolution, it wrapped. This only works if we read faster than
    the lift does half a turn, which at these speeds is easy.

    Height is relative to wherever the lift was when the host started. Homing
    (driving down until it stalls) is not done here yet; the soft limits are
    therefore relative too and should be set conservatively until it is.
    """

    def __init__(self, lead_mm_per_rev=84.0, soft_min_mm=-50.0, soft_max_mm=50.0, max_mm_s=100.0):
        self.mm_per_tick = lead_mm_per_rev / TICKS_PER_REV
        self.soft_min = soft_min_mm
        self.soft_max = soft_max_mm
        self.max_mm_s = max_mm_s
        self.last_ticks = None
        self.start_ticks = None
        self.turns = 0

    def update(self, pos_ticks):
        """Call with every fresh Present_Position of the lift servo."""
        if self.start_ticks is None:
            self.start_ticks = pos_ticks
        if self.last_ticks is not None:
            d = pos_ticks - self.last_ticks
            if d > TICKS_PER_REV / 2:
                self.turns -= 1
            elif d < -TICKS_PER_REV / 2:
                self.turns += 1
        self.last_ticks = pos_ticks

    @property
    def height_mm(self):
        if self.last_ticks is None:
            return 0.0
        return (self.turns * TICKS_PER_REV + self.last_ticks - self.start_ticks) * self.mm_per_tick

    def velocity_ticks(self, mm_s):
        """Requested lift speed (mm/s) -> servo ticks/s, zeroed at the soft limits."""
        mm_s = max(-self.max_mm_s, min(self.max_mm_s, mm_s))
        h = self.height_mm
        if (h >= self.soft_max and mm_s > 0) or (h <= self.soft_min and mm_s < 0):
            return 0
        return int(round(mm_s / self.mm_per_tick))
