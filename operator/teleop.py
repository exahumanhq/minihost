"""The operator's side: two leader arms, a keyboard, and a window of numbers.

    python operator/teleop.py --host <pi-ip> --left /dev/cu.usbmodem... --right /dev/cu.usbmodem...
    python operator/teleop.py --host 127.0.0.1 --fake-leader      # test against `minihost.host --sim`

Each tick (same 50 Hz as the host) it reads the leaders' Present_Position,
reads which keys are down, packs one command datagram and sends it. Whatever
state packet has arrived since last tick is shown. The leaders are read with
the same Bus class as the followers; torque is never enabled on them.

Keys: W/S forward/back, A/D left/right, Q/E rotate, R/F lift up/down.
      SPACE holds arms (followers freeze where they are), SPACE again resumes.
      ESC quits. Keys are velocities: release and the base stops.

The window shows the one number this whole project is about, round trip time,
computed from the timestamp the host echoes back. Also: how long since the
last state packet (if that grows, the link or the host is gone), the monitor
flags, and leader vs follower ticks per joint so a drifting joint is visible.

pygame is used because it gives key-down and key-up events (a terminal only
gives key presses, which can't do "release to stop") and a window to draw in,
and it runs the same on macOS and Linux. It is the only dependency here
besides the servo SDK.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from minihost.link import (CMD_ARMS_ACTIVE, ST_LOAD, ST_LOW_VOLT, ST_STALL, ST_WATCHDOG, STATE_SIZE, Udp,
                           pack_cmd, unpack_state)
from minihost.loop import Ticker

ARM_IDS = (1, 2, 3, 4, 5, 6)
JOINTS = ["pan", "lift", "elbow", "wflex", "wroll", "grip"]


class FakeLeader:
    """Slow sine on every joint, for testing the loop without arms in hand."""

    def __init__(self):
        self.t0 = time.monotonic()

    def sync_read(self):
        t = time.monotonic() - self.t0
        return {i: {"pos": int(2048 + 300 * math.sin(0.5 * t + i))} for i in ARM_IDS}

    def limits(self):
        return {i: (500, 3500) for i in ARM_IDS}

    def close(self):
        pass


def gripper_fraction(ticks, lo, hi):
    """Map a gripper's raw ticks onto 0..4095 across ITS OWN calibrated range.

    The two grippers on this robot were calibrated to different ranges (leader
    1686..2945 and the follower to something else, per the so101 README), so
    sending raw leader ticks would never fully open or close the follower, or
    would drive it into its stop. Every other joint is 2048-centred on both
    arms and goes across raw. The host maps the fraction back onto the
    follower gripper's range.
    """
    f = (ticks - lo) / max(1, hi - lo)
    return int(round(4095 * min(1.0, max(0.0, f))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--left")
    ap.add_argument("--right")
    ap.add_argument("--fake-leader", action="store_true")
    ap.add_argument("--hz", type=float, default=50)
    ap.add_argument("--speed", type=float, default=0.2, help="base m/s per key")
    ap.add_argument("--turn", type=float, default=0.8, help="base rad/s per key")
    ap.add_argument("--lift-speed", type=float, default=30.0, help="lift mm/s per key")
    ap.add_argument("--headless", action="store_true", help="no window; prints a line a second (for tests)")
    ap.add_argument("--seconds", type=float, default=0, help="quit after this long (0 = run until ESC)")
    a = ap.parse_args()

    if a.fake_leader:
        left = right = FakeLeader()
    else:
        if not (a.left and a.right):
            raise SystemExit("need --left and --right leader ports, or --fake-leader")
        from minihost.bus import Bus
        left, right = Bus(a.left, ARM_IDS), Bus(a.right, ARM_IDS)

    if a.headless:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((720, 420))
    pygame.display.set_caption("minihost operator")
    font = pygame.font.SysFont("monospace", 16)

    # the leaders' own calibrated gripper ranges, read once from the servo EEPROM
    glim = {"l": left.limits()[6], "r": right.limits()[6]}
    print(f"leader gripper ranges: left {glim['l']}, right {glim['r']}")

    net = Udp()
    addr = (a.host, a.port)
    tick = Ticker(a.hz)
    seq = 0
    arms_active = True
    state = None
    state_t = None
    rtt_ms = float("nan")
    t_start = time.monotonic()
    last_print = t_start

    try:
        while True:
            tick.wait()
            now = time.monotonic()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                    return
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                    arms_active = not arms_active
            k = pygame.key.get_pressed()
            vx = a.speed * (k[pygame.K_w] - k[pygame.K_s])
            vy = a.speed * (k[pygame.K_a] - k[pygame.K_d])
            wz = a.turn * (k[pygame.K_q] - k[pygame.K_e])
            lift = a.lift_speed * (k[pygame.K_r] - k[pygame.K_f])

            # read leaders: 6 ticks each, left then right
            lp, rp = left.sync_read(), right.sync_read()
            arm = [lp[i]["pos"] for i in ARM_IDS] + [rp[i]["pos"] for i in ARM_IDS]
            arm[5] = gripper_fraction(arm[5], *glim["l"])      # grippers go across as a fraction, see above
            arm[11] = gripper_fraction(arm[11], *glim["r"])

            net.send(pack_cmd(seq, arm, vx, vy, wz, lift, CMD_ARMS_ACTIVE if arms_active else 0), addr)
            seq += 1

            raw = net.recv(STATE_SIZE)
            if raw is not None:
                state = unpack_state(raw)
                state_t = now
                # the host echoed the monotonic time we stamped on the command it
                # last acted on; both ends of that are our own clock, so this is
                # a true round trip and needs no clock sync
                if state["echo_seq"] > 0:
                    rtt_ms = (now - state["echo_t"]) * 1e3

            # ---- draw
            screen.fill((20, 20, 20))
            y = 10
            def line(txt, color=(220, 220, 220)):
                nonlocal y
                screen.blit(font.render(txt, True, color), (10, y))
                y += 20
            age = (now - state_t) * 1e3 if state_t else float("nan")
            line(f"rtt {rtt_ms:6.1f} ms    state age {age:6.1f} ms    cmd seq {seq}", (255, 255, 120))
            if state:
                f = state["flags"]
                flags = " ".join(n for b, n in ((ST_WATCHDOG, "WATCHDOG"), (ST_STALL, "STALL"), (ST_LOAD, "LOAD"), (ST_LOW_VOLT, "LOW_VOLT")) if f & b)
                line(f"volt {state['volt']:.1f}  flags: {flags or '-'}", (255, 120, 120) if f else (120, 255, 120))
                line("")
                line(f"{'joint':8s} {'L lead':>7s} {'L foll':>7s} {'load':>5s}   {'R lead':>7s} {'R foll':>7s} {'load':>5s}")
                for j, name in enumerate(JOINTS):
                    line(f"{name:8s} {arm[j]:7d} {state['pos'][j]:7d} {state['load'][j]:5d}   "
                         f"{arm[6 + j]:7d} {state['pos'][6 + j]:7d} {state['load'][6 + j]:5d}")
            else:
                line("no state from host yet", (255, 120, 120))
            line("")
            line(f"base vx {vx:+.2f} vy {vy:+.2f} wz {wz:+.2f}   lift {lift:+.0f} mm/s   arms {'ACTIVE' if arms_active else 'HELD'}")
            line("WASD move  QE turn  RF lift  SPACE hold arms  ESC quit", (140, 140, 140))
            pygame.display.flip()

            if a.headless and now - last_print >= 1.0:
                print(f"[op] rtt={rtt_ms:.1f}ms age={age:.1f}ms flags={state['flags'] if state else '-'} "
                      f"L0 lead={arm[0]} foll={state['pos'][0] if state else '-'}", flush=True)
                last_print = now
            if a.seconds and now - t_start > a.seconds:
                return
    finally:
        left.close(); right.close()
        pygame.quit()


if __name__ == "__main__":
    main()
