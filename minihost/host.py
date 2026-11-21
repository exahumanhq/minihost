"""The process that runs on the robot. One fixed-rate loop that:

  1. takes the newest command off the UDP socket (or notices there isn't one)
  2. reads the latest servo state the bus thread has cached
  3. decides what the arms, wheels and lift should do, including the watchdog
  4. hands goals to the bus thread
  5. sends one state packet back, with the monitor flags
  6. sleeps until the next deadline

Nothing in here blocks on hardware or network. The bus has its own thread
(bus.py), the socket is non-blocking (link.py), and video is a separate
process entirely (camera.py, not written yet) so a slow frame can never
delay a joint command.

    python -m minihost.host --left /dev/serial/by-id/... --right /dev/serial/by-id/...

Run tools/measure_bus.py first. Start with --no-torque to watch the loop and
the numbers before anything moves.
"""
import argparse
import time

from .base import LIFT_ID, WHEEL_IDS, Lift, OmniBase
from .bus import Bus, BusThread
from .link import CMD_SIZE, CMD_ARMS_ACTIVE, ST_LOAD, ST_LOW_VOLT, ST_STALL, ST_WATCHDOG, Udp, pack_state, unpack_cmd
from .loop import Summary, Ticker
from .monitor import Monitor

ARM_IDS = (1, 2, 3, 4, 5, 6)


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
    arms = [(left, 0), (right, 6)]          # (bus, offset into the 12-long arm arrays)

    volt = left.voltage()
    print(f"follower bus voltage {volt:.1f} V")
    if volt < a.min_volt and not a.no_torque:
        raise SystemExit(f"bus voltage {volt:.1f} V is below {a.min_volt}; turn the 12 V supply on")
    limits = {}
    for bus, off in arms:
        for i, (lo, hi) in bus.limits().items():
            if i in ARM_IDS:
                limits[off + i - 1] = (lo, hi)

    if not a.no_torque:
        left.set_velocity_mode(list(WHEEL_IDS) + [LIFT_ID])
        for bus, _ in arms:
            bus.prepare_follower()

    lt, rt = BusThread(left), BusThread(right)
    lt.start(); rt.start()
    # wait for the first read from each bus so we know where the arms are
    while lt.latest is None or rt.latest is None:
        time.sleep(0.01)

    base = OmniBase()
    lift = Lift()
    monitor = Monitor(dt=1 / a.hz)
    net = Udp(bind_port=a.port)
    tick = Ticker(a.hz)
    summary = Summary()

    # the goal we are currently sending the arms; starts at where they are, so
    # enabling torque holds position instead of jumping
    goal = [0] * 12
    for bus_t, off in ((lt, 0), (rt, 6)):
        for i in ARM_IDS:
            goal[off + i - 1] = bus_t.latest[i]["pos"]
    last_cmd_t = None
    last_cmd = None
    seq = 0
    print(f"listening on udp {a.port}, loop {a.hz:.0f} Hz, watchdog {a.watchdog_ms:.0f} ms")

    try:
        while True:
            late = tick.wait()
            now = time.monotonic()

            # 1. newest command, if any
            raw = net.recv(CMD_SIZE)
            if raw is not None:
                last_cmd = unpack_cmd(raw)
                last_cmd_t = now

            # 2. latest servo state
            pos = [0] * 12
            load = [0] * 12
            for bus_t, off in ((lt, 0), (rt, 6)):
                for i in ARM_IDS:
                    pos[off + i - 1] = bus_t.latest[i]["pos"]
                    load[off + i - 1] = bus_t.latest[i]["load"]
            lift.update(lt.latest[LIFT_ID]["pos"])
            volt = lt.latest[ARM_IDS[0]]["volt"]

            # 3. decide
            flags = 0
            alive = last_cmd_t is not None and (now - last_cmd_t) * 1e3 < a.watchdog_ms
            if alive and last_cmd["flags"] & CMD_ARMS_ACTIVE:
                # Move each goal toward the operator's target by at most max_step
                # ticks per tick, and never outside the servo's own limits. The
                # step clamp is what makes a sudden leader jump a smooth follower
                # move instead of a snap.
                for k in range(12):
                    v = last_cmd["arm"][k]
                    if k in (5, 11):
                        # gripper arrives as 0..4095 fraction of the leader's range (see
                        # operator/teleop.py); stretch it onto this follower gripper's range
                        lo, hi = limits[k]
                        v = lo + (hi - lo) * v / 4095
                    tgt = max(limits[k][0], min(limits[k][1], v))
                    d = tgt - goal[k]
                    goal[k] += max(-a.max_step, min(a.max_step, d))
                wheels = base.wheel_speeds(last_cmd["vx"], last_cmd["vy"], last_cmd["wz"])
                lift_v = lift.velocity_ticks(last_cmd["lift"])
            else:
                # Watchdog: no fresh command. Arms hold their current goal,
                # base and lift stop. Holding (rather than going limp) is what
                # every system read does; a limp arm drops what it is carrying.
                if last_cmd_t is not None:
                    flags |= ST_WATCHDOG
                wheels = {i: 0 for i in WHEEL_IDS}
                lift_v = 0

            m = monitor.update(goal, pos, load)
            flags |= (ST_STALL if m["stall"] else 0) | (ST_LOAD if m["load"] else 0)
            if volt < a.min_volt:
                flags |= ST_LOW_VOLT

            # 4. hand to the bus threads (they write after their next read)
            if not a.no_torque:
                lt.set_goals({i: goal[i - 1] for i in ARM_IDS})
                rt.set_goals({i: goal[6 + i - 1] for i in ARM_IDS})
                lt.set_velocities({**wheels, LIFT_ID: lift_v})

            # 5. tell the operator
            if net.peer:
                echo_seq = last_cmd["seq"] if last_cmd else 0
                echo_t = last_cmd["t"] if last_cmd else 0.0
                net.send(pack_state(seq, echo_seq, echo_t, pos, load, volt, flags))
                seq += 1

            # 6. numbers, once a second
            summary.add(late_ms=late * 1e3, bus_l_ms=lt.read_ms, bus_r_ms=rt.read_ms,
                        bus_err=lt.errors + rt.errors, dead_motors=len(lt.missing) + len(rt.missing), dropped=net.dropped, wd=int(bool(flags & ST_WATCHDOG)),
                        stall=int(bool(flags & ST_STALL)))
            summary.maybe_print(f"[host] lift={lift.height_mm:.0f}mm v={volt:.1f} ")
    except KeyboardInterrupt:
        pass
    finally:
        # torque off no matter how we got here. Order: stop the threads first so
        # a queued goal can't be written after torque is off.
        lt.stop(); rt.stop()
        if not a.no_torque:
            left.sync_write_velocity({**{i: 0 for i in WHEEL_IDS}, LIFT_ID: 0})
            left.torque_off(); right.torque_off()
        left.close(); right.close()
        print("torque off, ports closed")


if __name__ == "__main__":
    main()
