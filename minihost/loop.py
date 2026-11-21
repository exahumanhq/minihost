"""Fixed-rate loop pacing, and the once-a-second summary.

The idea: decide the next tick time in advance and sleep until it, instead of
sleeping a fixed amount after the work. With sleep(dt)-after-work (what the
original ALOHA script does) the real period is dt plus however long the work
took, so 50 Hz quietly becomes 45. With a busy-wait (GELLO's Rate class) the
period is exact but a core sits at 100 percent, which on a Pi 4 is a quarter of
the machine. Sleeping until an absolute deadline gives the exact period and
costs nothing, and if a tick runs late we know by how much.

time.monotonic everywhere. time.time can jump when NTP adjusts the clock, and
a control loop that measures intervals with it will eventually see a negative
or huge dt.
"""
import time


class Ticker:
    def __init__(self, hz):
        self.dt = 1.0 / hz
        self.next = time.monotonic()
        self.late = 0            # ticks that started after their deadline
        self.worst_late_ms = 0.0
        self.count = 0

    def wait(self):
        """Sleep until the next deadline, then advance it. Returns how late we were (s)."""
        now = time.monotonic()
        late = now - self.next
        if late > 0:
            self.late += 1
            self.worst_late_ms = max(self.worst_late_ms, late * 1e3)
            # we missed the deadline; don't try to catch up by running ticks
            # back to back, just rebase. Catching up would burst the servos.
            self.next = now
        else:
            time.sleep(-late)
        self.next += self.dt
        self.count += 1
        return max(late, 0.0)


class Summary:
    """Collects numbers during the second and prints one line at the end of it.

    Printing every tick floods the terminal and costs more than the tick. One
    line a second is enough to see if the loop is healthy, which is the point.
    """

    def __init__(self, period_s=1.0):
        self.period = period_s
        self.t0 = time.monotonic()
        self.sums = {}
        self.n = 0

    def add(self, **vals):
        for k, v in vals.items():
            self.sums[k] = self.sums.get(k, 0.0) + v
        self.n += 1

    def maybe_print(self, prefix=""):
        now = time.monotonic()
        if now - self.t0 < self.period:
            return
        hz = self.n / (now - self.t0)
        fields = " ".join(f"{k}={v / max(self.n, 1):.1f}" for k, v in self.sums.items())
        print(f"{prefix}hz={hz:.1f} {fields}", flush=True)
        self.t0, self.sums, self.n = now, {}, 0
