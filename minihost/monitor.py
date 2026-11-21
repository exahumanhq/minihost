"""The robot-side "I need help" signal, in servo ticks.

Same two rules as the simulation bench, but on raw register values:

  stall: the goal we are sending is far from where the joint is, and it has
         stayed that way for a dwell time. A healthy servo closes a position
         error in well under a second; one that sits on a big error is pushing
         against something or has given up.
  load:  Present_Load is high on some joint for a dwell time. The bench showed
         load alone misses out-of-reach commands (no force when a servo pins at
         its limit) and stall alone misses a joint that is winning against a
         drag, so both are kept.

Thresholds are guesses in ticks until measure_bus.py tells us what a resting
load and a normal tracking error look like on this arm. They are attributes
so host.py can be run with different values without editing this file.

This only raises flags. Deciding what to do about them (stop the base, drop
to delayed feedback on the operator screen) is the operator's or a later
version's job. False alarm rate has to be known on hardware first.
"""


class Monitor:
    err_ticks = 60       # about 5 degrees
    stall_s = 0.6
    load_thr = 500       # Present_Load is 0..1023 magnitude
    load_s = 0.6

    def __init__(self, dt):
        self.dt = dt
        self.stall_t = 0.0
        self.load_t = 0.0

    def update(self, goal, pos, load):
        err = max(abs(g - p) for g, p in zip(goal, pos))
        peak_load = max(abs(l) for l in load)
        self.stall_t = self.stall_t + self.dt if err > self.err_ticks else 0.0
        self.load_t = self.load_t + self.dt if peak_load > self.load_thr else 0.0
        return {"stall": self.stall_t >= self.stall_s, "load": self.load_t >= self.load_s,
                "err_ticks": err, "peak_load": peak_load}
