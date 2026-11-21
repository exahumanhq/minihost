import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from minihost.base import WHEEL_IDS, Lift, OmniBase
from minihost.bus import from_sign_mag, to_sign_mag
from minihost.link import CMD_SIZE, STATE_SIZE, Udp, pack_cmd, pack_state, unpack_cmd, unpack_state
from minihost.monitor import Monitor


def test_sign_magnitude_roundtrip():
    for v in (0, 5, -5, 1023, -1023):
        assert from_sign_mag(to_sign_mag(v, 10), 10) == v
    for v in (0, 3000, -3000):
        assert from_sign_mag(to_sign_mag(v, 15), 15) == v


def test_cmd_and_state_pack_roundtrip():
    c = unpack_cmd(pack_cmd(7, list(range(2000, 2012)), 0.1, -0.2, 0.3, 25.0, 1))
    assert c["seq"] == 7 and c["arm"] == list(range(2000, 2012)) and c["flags"] == 1
    assert abs(c["vx"] - 0.1) < 1e-6
    s = unpack_state(pack_state(3, 7, c["t"], [2048] * 12, [-100] * 12, 12.1, 2))
    assert s["echo_seq"] == 7 and s["load"] == [-100] * 12 and s["flags"] == 2


def test_udp_newest_wins_loopback():
    host = Udp(bind_port=5699)
    op = Udp()
    for seq in range(5):
        op.send(pack_cmd(seq, [2048] * 12, 0, 0, 0, 0, 1), ("127.0.0.1", 5699))
    time.sleep(0.02)
    c = unpack_cmd(host.recv(CMD_SIZE))
    assert c["seq"] == 4 and host.dropped == 4
    host.send(pack_state(0, c["seq"], c["t"], [2048] * 12, [0] * 12, 12.0, 0))
    time.sleep(0.02)
    s = unpack_state(op.recv(STATE_SIZE))
    rtt_ms = (time.monotonic() - s["echo_t"]) * 1e3
    assert s["echo_seq"] == 4 and 0 < rtt_ms < 200


def test_omni_base_pure_rotation_spins_all_wheels_same_way():
    b = OmniBase()
    w = b.wheel_speeds(0, 0, 1.0)
    assert len(set(math.copysign(1, v) for v in w.values())) == 1
    assert b.wheel_speeds(0, 0, 0) == {i: 0 for i in WHEEL_IDS}


def test_omni_base_scales_down_not_clips():
    b = OmniBase(max_ticks_s=100)
    w = b.wheel_speeds(5.0, 0, 0)
    assert max(abs(v) for v in w.values()) == 100


def test_lift_unwraps_and_respects_soft_limits():
    l = Lift(lead_mm_per_rev=84.0, soft_min_mm=-10, soft_max_mm=10)
    for t in (4000, 4090, 50, 150):      # crosses 4095 -> 0 going up
        l.update(t)
    assert l.turns == 1
    assert abs(l.height_mm - (4096 + 150 - 4000) * 84 / 4096) < 1e-6   # relative to the first reading
    l2 = Lift(soft_max_mm=0.0)
    l2.update(100)
    assert l2.velocity_ticks(50) == 0 and l2.velocity_ticks(-50) < 0


def test_monitor_dwell():
    m = Monitor(dt=0.1)
    for _ in range(5):
        r = m.update([2100] * 12, [2000] * 12, [0] * 12)
    assert not r["stall"]
    r = m.update([2100] * 12, [2000] * 12, [0] * 12)
    assert r["stall"]
    assert not m.update([2000] * 12, [2000] * 12, [0] * 12)["stall"]


def test_sim_bus_tracks_goal_and_shows_load_when_pushed():
    from minihost.sim_bus import SimBus
    b = SimBus("right", [1, 2, 3, 4, 5, 6])
    b.prepare_follower()
    b.sync_write_goal({i: 2048 for i in range(1, 7)})
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        s = b.sync_read()
        time.sleep(0.005)
    assert all(abs(s[i]["pos"] - 2048) < 20 for i in range(1, 7))
    b.push_tip(40.0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        s = b.sync_read()
        time.sleep(0.005)
    assert max(abs(s[i]["load"]) for i in range(1, 7)) > 100


def test_real_bus_over_pty_emulator_reads_writes_and_survives_a_dead_motor():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
    from feetech_sim import open_board
    from minihost.bus import Bus, BusThread
    path, board = open_board("right", [1, 2, 3, 4, 5, 6])
    b = Bus(path, [1, 2, 3, 4, 5, 6])
    assert b.voltage() == 12.0
    b.prepare_follower(gap_s=0.0)
    assert board.mem[3]["Torque_Enable"] == 1 and board.mem[3]["Acceleration"] == 30
    board.missing.add(4)            # cable falls out after start-up
    t = BusThread(b)
    t.start()
    time.sleep(0.4)
    assert t.missing == [4] and sorted(t.latest) == [1, 2, 3, 5, 6]
    assert t.read_ms < 5            # once dropped, reads are fast again
    board.missing.clear()
    time.sleep(2.5)
    assert t.missing == []
    t.stop()
    b.torque_off()
    assert board.mem[1]["Torque_Enable"] == 0
    b.close()


def test_gripper_fraction():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "operator"))
    from teleop import gripper_fraction
    assert gripper_fraction(1686, 1686, 2945) == 0
    assert gripper_fraction(2945, 1686, 2945) == 4095
    assert gripper_fraction(100, 1686, 2945) == 0      # clamps below the range
