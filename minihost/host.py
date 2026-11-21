"""The process that runs on the robot. One fixed-rate loop that:

  1. takes the newest command off the UDP socket (or notices there isn't one)
  2. reads the latest servo state the bus thread has cached
  3. decides what the arms, wheels and lift should do, including the watchdog
  4. hands goals to the bus thread
  5. sends one state packet back, with the monitor flags
  6. sleeps until the next deadline

Nothing in here blocks on hardware or network. The bus has its own thread
(bus.py), the socket is non-blocking (link.py), and video is a separate
process entirely (camera.py) so a slow frame can never delay a joint command.

    python -m minihost.host --left /dev/serial/by-id/... --right /dev/serial/by-id/...

Run tools/measure_bus.py first. Start with --no-torque to watch the loop and
the numbers before anything moves.

The loop body lives in `Host.tick()` so the tests can drive it one tick at a
time against the twin or the servo emulator; `main()` only parses arguments
and calls `Host.run()`.
"""
import argparse
import time

from .base import LIFT_ID, WHEEL_IDS, Lift, OmniBase
from .bus import Bus, BusThread
from .link import CMD_SIZE, CMD_ARMS_ACTIVE, ST_LOAD, ST_LOW_VOLT, ST_STALL, ST_WATCHDOG, Udp, pack_state, unpack_cmd
from .loop import Summary, Ticker
from .monitor import Monitor

ARM_IDS = (1, 2, 3, 4, 5, 6)


class Host:
    """Everything between "ports are open" and "torque off, ports closed".

    `left` carries the left arm (1-6) plus wheels (8-10) and lift (11); `right`
    the right arm (1-6). Both are Bus or SimBus. Construction does the one-time
    setup (voltage check, limits, velocity mode, torque with caps, bus threads,
    first read); `tick()` is one loop iteration; `run()` paces it; `close()` is
    the shutdown path everything relies on.
    """

    def __init__(self, left, right, hz=50, watchdog_ms=300, max_step=80, min_volt=11.0, no_torque=False,
                 udp_port=5600, summary=None):
        self.left, self.right = left, right
        self.hz, self.watchdog_ms, self.max_step, self.min_volt, self.no_torque = hz, watchdog_ms, max_step, min_volt, no_torque
        arms = [(left, 0), (right, 6)]          # (bus, offset into the 12-long arm arrays)

        self.volt = left.voltage()
        print(f"follower bus voltage {self.volt:.1f} V")
        if self.volt < min_volt and not no_torque:
            raise SystemExit(f"bus voltage {self.volt:.1f} V is below {min_volt}; turn the 12 V supply on")
        self.limits = {}
        for bus, off in arms:
            for i, (lo, hi) in bus.limits().items():
                if i in ARM_IDS:
                    self.limits[off + i - 1] = (lo, hi)

        if not no_torque:
            left.set_velocity_mode(list(WHEEL_IDS) + [LIFT_ID])
            for bus, _ in arms:
                bus.prepare_follower()

        self.lt, self.rt = BusThread(left), BusThread(right)
        self.lt.start(); self.rt.start()
        # wait for the first read from each bus so we know where the arms are
        deadline = time.monotonic() + 5.0
        while self.lt.latest is None or self.rt.latest is None:
            if time.monotonic() > deadline:
                self.close()
                raise SystemExit("no answer from the servos within 5 s; check power, ports and ids")
            time.sleep(0.01)

        self.base = OmniBase()
        self.lift = Lift()
        self.monitor = Monitor(dt=1 / hz)
        self.net = Udp(bind_port=udp_port)
        self.summary = summary or Summary()

        # the goal we are currently sending the arms; starts at where they are, so
        # enabling torque holds position instead of jumping
        self.goal = [0] * 12
        for bus_t, off in ((self.lt, 0), (self.rt, 6)):
            for i in ARM_IDS:
                self.goal[off + i - 1] = bus_t.latest[i]["pos"]
        self.last_cmd_t = None
        self.last_cmd = None
        self.seq = 0
        self.ticks = 0
        self.flags = 0
        print(f"listening on udp {self.net.port}, loop {hz:.0f} Hz, watchdog {watchdog_ms:.0f} ms")

    def tick(self, late=0.0, now=None):
        """One loop iteration. Returns the state flags it sent (or would send)."""
        now = time.monotonic() if now is None else now
        lt, rt, a_max_step = self.lt, self.rt, self.max_step

        # 1. newest command, if any
        raw = self.net.recv(CMD_SIZE)
        if raw is not None:
            self.last_cmd = unpack_cmd(raw)
            self.last_cmd_t = now

        # 2. latest servo state
        pos = [0] * 12
        load = [0] * 12
        for bus_t, off in ((lt, 0), (rt, 6)):
            for i in ARM_IDS:
                pos[off + i - 1] = bus_t.latest[i]["pos"]
                load[off + i - 1] = bus_t.latest[i]["load"]
        self.lift.update(lt.latest[LIFT_ID]["pos"])
        volt = lt.latest[ARM_IDS[0]]["volt"]
        self.pos, self.load = pos, load

        # 3. decide
        flags = 0
        alive = self.last_cmd_t is not None and (now - self.last_cmd_t) * 1e3 < self.watchdog_ms
        if alive and self.last_cmd["flags"] & CMD_ARMS_ACTIVE:
            # Move each goal toward the operator's target by at most max_step
            # ticks per tick, and never outside the servo's own limits. The
            # step clamp is what makes a sudden leader jump a smooth follower
            # move instead of a snap.
            for k in range(12):
                v = self.last_cmd["arm"][k]
                if k in (5, 11):
                    # gripper arrives as 0..4095 fraction of the leader's range (see
                    # operator/teleop.py); stretch it onto this follower gripper's range
                    lo, hi = self.limits[k]
                    v = lo + (hi - lo) * v / 4095
                tgt = max(self.limits[k][0], min(self.limits[k][1], v))
                d = tgt - self.goal[k]
                self.goal[k] += max(-a_max_step, min(a_max_step, d))
            wheels = self.base.wheel_speeds(self.last_cmd["vx"], self.last_cmd["vy"], self.last_cmd["wz"])
            lift_v = self.lift.velocity_ticks(self.last_cmd["lift"])
        else:
            # Watchdog: no fresh command. Arms hold their current goal,
            # base and lift stop. Holding (rather than going limp) is what
            # every system read does; a limp arm drops what it is carrying.
            if self.last_cmd_t is not None:
                flags |= ST_WATCHDOG
            wheels = {i: 0 for i in WHEEL_IDS}
            lift_v = 0

        m = self.monitor.update(self.goal, pos, load)
        flags |= (ST_STALL if m["stall"] else 0) | (ST_LOAD if m["load"] else 0)
        if volt < self.min_volt:
            flags |= ST_LOW_VOLT
        self.flags = flags
        self.wheels, self.lift_v = wheels, lift_v

        # 4. hand to the bus threads (they write after their next read)
        if not self.no_torque:
            lt.set_goals({i: self.goal[i - 1] for i in ARM_IDS})
            rt.set_goals({i: self.goal[6 + i - 1] for i in ARM_IDS})
            lt.set_velocities({**wheels, LIFT_ID: lift_v})

        # 5. tell the operator
        if self.net.peer:
            echo_seq = self.last_cmd["seq"] if self.last_cmd else 0
            echo_t = self.last_cmd["t"] if self.last_cmd else 0.0
            self.net.send(pack_state(self.seq, echo_seq, echo_t, pos, load, volt, flags))
            self.seq += 1

        # 6. numbers, once a second
        self.summary.add(late_ms=late * 1e3, bus_l_ms=lt.read_ms, bus_r_ms=rt.read_ms,
                         bus_err=lt.errors + rt.errors, dead_motors=len(lt.missing) + len(rt.missing),
                         dropped=self.net.dropped, wd=int(bool(flags & ST_WATCHDOG)), stall=int(bool(flags & ST_STALL)))
        self.summary.maybe_print(f"[host] lift={self.lift.height_mm:.0f}mm v={volt:.1f} ")
        self.ticks += 1
        return flags

    def run(self):
        tick = Ticker(self.hz)
        try:
            while True:
                late = tick.wait()
                self.tick(late)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self):
        """Torque off no matter how we got here. Order: stop the threads first so
        a queued goal can't be written after torque is off."""
        for t in (getattr(self, "lt", None), getattr(self, "rt", None)):
            if t is not None:
                t.stop()
        if not self.no_torque:
            self.left.sync_write_velocity({**{i: 0 for i in WHEEL_IDS}, LIFT_ID: 0})
            self.left.torque_off(); self.right.torque_off()
        self.left.close(); self.right.close()
        if getattr(self, "net", None) is not None:
            self.net.sock.close()
        print("torque off, ports closed")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", help="serial port of the left arm board (also wheels and lift)")
    ap.add_argument("--right", help="serial port of the right arm board")
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--hz", type=float, default=50)
    ap.add_argument("--watchdog-ms", type=float, default=300)
    ap.add_argument("--max-step", type=int, default=80, help="max ticks a joint goal may move per tick")
    ap.add_argument("--min-volt", type=float, default=11.0)
    ap.add_argument("--no-torque", action="store_true", help="run everything but never enable torque")
    ap.add_argument("--sim", action="store_true", help="drive the MuJoCo twin instead of serial ports")
    a = ap.parse_args()

    if not a.sim and not (a.left and a.right):
        raise SystemExit("need --left and --right serial ports, or --sim")
    if a.sim:
        from .sim_bus import SimBus as BusCls
        a.left, a.right = "left", "right"
    else:
        BusCls = Bus
    # The left board carries the left arm (1-6) plus wheels (8-10) and lift (11).
    left = BusCls(a.left, list(ARM_IDS) + list(WHEEL_IDS) + [LIFT_ID])
    right = BusCls(a.right, list(ARM_IDS))
    Host(left, right, hz=a.hz, watchdog_ms=a.watchdog_ms, max_step=a.max_step, min_volt=a.min_volt,
         no_torque=a.no_torque, udp_port=a.port).run()


if __name__ == "__main__":
    main()
