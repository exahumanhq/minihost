"""A Bus that is really the MuJoCo twin. Same methods as bus.Bus, so host.py
runs unchanged with `--sim`. Lets the whole loop (UDP, watchdog, step clamp,
monitor, lift unwrapping) be exercised with servos that have inertia, limits
and contact, before anything touches hardware.

Tick convention: the STS3215 reports 0..4095 per revolution, 2048 at the
middle. I map 2048 to joint angle 0 and 4096 ticks to 2*pi, which is what the
servo does physically. Whether the real arms are calibrated so that 2048 is
the MJCF's zero pose is an assumption; Skyler's notes say the arms were
calibrated in-servo to the SO-101 "2048 = middle" convention, which is the
same thing if the MJCF zero is the middle of each joint's range. Check on
hardware by reading positions with measure_bus.py in a known pose.

Wheels and lift: the twin's base is modelled but I only drive the lift here
(it is a position actuator in the MJCF, so lift velocity is integrated into a
height target). Wheels are accepted and ignored.
"""
import math
import os
import threading
import time

import mujoco

from .base import LIFT_ID, TICKS_PER_REV, WHEEL_IDS

XML = os.path.join(os.path.dirname(__file__), "..", "vendor", "exahuman_twin", "mujoco", "alohamini_twin.xml")
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)


def rad_to_ticks(r):
    return int(round(2048 + r * TICKS_PER_RAD))


def ticks_to_rad(t):
    return (t - 2048) / TICKS_PER_RAD


class _World:
    """One MuJoCo model shared by both arms' SimBus. Stepped by whichever bus
    asks first in a given period; the lock keeps the two bus threads from
    stepping at once."""

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(XML)
        self.d = mujoco.MjData(self.m)
        self.lock = threading.Lock()
        self.last_step = time.monotonic()
        self.lift_act = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_pos")
        self.d.ctrl[self.lift_act] = 0.3
        self.lift_v = 0.0   # m/s, integrated into the lift position target
        mujoco.mj_forward(self.m, self.d)

    def advance(self):
        """Step physics forward to wall-clock now (so the sim runs in real time)."""
        with self.lock:
            now = time.monotonic()
            n = int((now - self.last_step) / self.m.opt.timestep)
            if n <= 0:
                return
            self.d.ctrl[self.lift_act] = min(0.6, max(0.0, self.d.ctrl[self.lift_act] + self.lift_v * n * self.m.opt.timestep))
            for _ in range(min(n, 200)):   # cap so a long pause doesn't freeze us catching up
                mujoco.mj_step(self.m, self.d)
            self.last_step = now


_world = None


def world():
    global _world
    if _world is None:
        _world = _World()
    return _world


class SimBus:
    def __init__(self, port, ids, baud=0):
        """`port` is "left" or "right" here, picking which arm of the twin."""
        self.arm = port
        self.ids = list(ids)
        self.w = world()
        m = self.w.m
        self.act = {i: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{self.arm}_{JOINTS[i - 1]}") for i in self.ids if i <= 6}
        jnt = {i: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{self.arm}_{JOINTS[i - 1]}") for i in self.ids if i <= 6}
        self.qpos = {i: m.jnt_qposadr[j] for i, j in jnt.items()}
        self.qvel = {i: m.jnt_dofadr[j] for i, j in jnt.items()}
        self.force_max = {i: m.actuator_forcerange[a, 1] for i, a in self.act.items()}
        self.torque = False
        self.lift_ticks = 2048.0   # fake lift encoder, wraps like the real one
        self.polled = list(self.ids)   # BusThread drops silent motors through set_polled; mirror Bus

    def set_polled(self, ids):
        self.polled = list(ids)

    def voltage(self):
        return 12.0

    def limits(self):
        m = self.w.m
        out = {}
        for i in self.ids:
            if i in self.act:
                lo, hi = m.actuator_ctrlrange[self.act[i]]
                out[i] = (rad_to_ticks(lo), rad_to_ticks(hi))
            else:
                out[i] = (0, 4095)
        return out

    def set_velocity_mode(self, ids):
        pass

    def prepare_follower(self, **kw):
        self.torque = True

    def torque_off(self):
        self.torque = False

    def close(self):
        pass

    def sync_read(self):
        self.w.advance()
        d = self.w.d
        out = {}
        for i in self.ids:
            if i in self.act:
                load = d.actuator_force[self.act[i]] / self.force_max[i] * 1023   # scale to the 0..1023 register
                out[i] = {"pos": rad_to_ticks(d.qpos[self.qpos[i]]), "vel": int(d.qvel[self.qvel[i]] * TICKS_PER_RAD),
                          "load": int(load), "volt": 12.0, "temp": 35}
            elif i == LIFT_ID:
                # lift height in metres -> lead screw ticks (84 mm/rev), wrapped to 0..4095
                ticks = d.ctrl[self.w.lift_act] * 1000 / 84.0 * TICKS_PER_REV
                out[i] = {"pos": int(ticks) % TICKS_PER_REV, "vel": 0, "load": 0, "volt": 12.0, "temp": 35}
            else:
                out[i] = {"pos": 0, "vel": 0, "load": 0, "volt": 12.0, "temp": 35}
        return out

    def sync_write_goal(self, goals):
        if not self.torque:
            return
        for i, g in goals.items():
            if i in self.act:
                self.w.d.ctrl[self.act[i]] = ticks_to_rad(g)

    def sync_write_velocity(self, vels):
        if LIFT_ID in vels:
            # ticks/s on an 84 mm lead -> m/s
            self.w.lift_v = vels[LIFT_ID] / TICKS_PER_REV * 84.0 / 1000
        # wheels ignored in sim for now

    # for tests: push on the right gripper, like the bench's obstacle
    def push_tip(self, force_z):
        m, d = self.w.m, self.w.d
        site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{self.arm}_gripperframe")
        d.xfrc_applied[m.site_bodyid[site], 2] = force_z
