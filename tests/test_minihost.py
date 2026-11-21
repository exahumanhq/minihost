import math
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "operator"))
from minihost.base import LIFT_ID, WHEEL_IDS, Lift, OmniBase
from minihost.bus import from_sign_mag, to_sign_mag
from minihost.link import (CMD_ARMS_ACTIVE, CMD_SIZE, ST_WATCHDOG, STATE_SIZE, Udp, pack_cmd, pack_state,
                           unpack_cmd, unpack_state)
from minihost.loop import Summary, percentile
from minihost.monitor import Monitor

ARM_IDS = [1, 2, 3, 4, 5, 6]
LEFT_IDS = ARM_IDS + list(WHEEL_IDS) + [LIFT_ID]
HAS_PTY = hasattr(os, "openpty")


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
    host = Udp(bind_port=0)
    op = Udp()
    for seq in range(5):
        op.send(pack_cmd(seq, [2048] * 12, 0, 0, 0, 0, 1), ("127.0.0.1", host.port))
    time.sleep(0.02)
    c = unpack_cmd(host.recv(CMD_SIZE))
    assert c["seq"] == 4 and host.dropped == 4
    host.send(pack_state(0, c["seq"], c["t"], [2048] * 12, [0] * 12, 12.0, 0))
    time.sleep(0.02)
    s = unpack_state(op.recv(STATE_SIZE))
    rtt_ms = (time.monotonic() - s["echo_t"]) * 1e3
    assert s["echo_seq"] == 4 and 0 <= rtt_ms < 200


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


def test_summary_percentiles_show_the_stall_a_mean_hides():
    assert percentile([], 0.5) == 0.0 and percentile([3, 1, 2], 0.5) == 2 and percentile(list(range(100)), 0.99) == 98
    s = Summary()
    for _ in range(49):
        s.add(bus_l_ms=0.7, wd=0)
    s.add(bus_l_ms=34.9, wd=1)          # one silent-motor read in the second
    st = s.stats()
    assert abs(st["bus_l_ms"]["mean"] - 1.384) < 0.01     # the mean barely moves
    assert st["bus_l_ms"]["p99"] == 34.9 and st["bus_l_ms"]["max"] == 34.9
    line = s.format(now=s.t0 + 1.0)
    assert "bus_l_ms=1.4|34.9|34.9" in line and "wd=0.0" in line and line.startswith("hz=50.0")


def test_sim_bus_tracks_goal_and_shows_load_when_pushed():
    from minihost.sim_bus import SimBus
    b = SimBus("right", ARM_IDS)
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


def _dead_motor_scenario(bus, board):
    """Shared by the pty and loopback variants: same Bus, same SDK, different wire."""
    from minihost.bus import BusThread
    assert bus.voltage() == 12.0
    bus.prepare_follower(gap_s=0.0)
    assert board.mem[3]["Torque_Enable"] == 1 and board.mem[3]["Acceleration"] == 30
    board.missing.add(4)            # cable falls out after start-up
    bus.sync_read()                 # a good read first, so stale buffers would exist
    t0 = time.perf_counter()
    data = bus.sync_read()
    stall_ms = (time.perf_counter() - t0) * 1e3
    # The SDK stops at the silent motor, so 5 and 6 are lost in this read too. They must be
    # ABSENT, not reported with last read's bytes (the stale-buffer bug sync_read now clears).
    assert sorted(data) == [1, 2, 3]
    t = BusThread(bus)
    t.start()
    time.sleep(0.4)
    assert t.missing == [4] and sorted(t.latest) == [1, 2, 3, 5, 6]
    assert t.read_ms < 5            # once dropped, reads are fast again
    board.missing.clear()
    time.sleep(2.5)
    assert t.missing == []
    t.stop()
    bus.torque_off()
    assert board.mem[1]["Torque_Enable"] == 0
    bus.close()
    return stall_ms


@pytest.mark.skipif(not HAS_PTY, reason="pty needs Linux or macOS")
def test_real_bus_over_pty_emulator_reads_writes_and_survives_a_dead_motor():
    from feetech_sim import open_board
    from minihost.bus import Bus
    path, board = open_board("right", ARM_IDS)
    _dead_motor_scenario(Bus(path, ARM_IDS), board)


def test_real_bus_over_loopback_emulator_any_os():
    """Same Bus and SDK code as the pty test, with LoopbackPort standing in for the serial device.
    The SDK's own timeout formula applies, so the silent-motor stall is reproduced: 2 x 16 ms + 2 ms."""
    from feetech_sim import loopback_board
    from minihost.bus import Bus
    port, board = loopback_board("right", ARM_IDS, latency_ms=16.0)
    stall_ms = _dead_motor_scenario(Bus("loopback", ARM_IDS, port_handler=port), board)
    assert 30 < stall_ms < 80, stall_ms   # the ~34 ms the docs talk about


def test_gripper_fraction():
    from teleop import gripper_fraction
    assert gripper_fraction(1686, 1686, 2945) == 0
    assert gripper_fraction(2945, 1686, 2945) == 4095
    assert gripper_fraction(100, 1686, 2945) == 0      # clamps below the range


# ---- host loop ---------------------------------------------------------------------------------

def _send_cmd(host, arm, vx=0.0, vy=0.0, wz=0.0, lift=0.0, flags=CMD_ARMS_ACTIVE, seq=1):
    op = Udp()
    op.send(pack_cmd(seq, arm, vx, vy, wz, lift, flags), ("127.0.0.1", host.net.port))
    time.sleep(0.02)
    return op


def _settle(host):
    """Give the bus threads a couple of iterations to write the queued goals."""
    time.sleep(0.05)


def test_host_loop_on_twin_clamps_steps_maps_gripper_and_watches_the_dog():
    from minihost.host import Host
    from minihost.sim_bus import SimBus
    left, right = SimBus("left", LEFT_IDS), SimBus("right", ARM_IDS)
    host = Host(left, right, hz=50, watchdog_ms=100, max_step=80, udp_port=0)
    try:
        start = list(host.goal)
        lo, hi = host.limits[5]
        arm = [start[k] + 500 for k in range(5)] + [4095] + [start[6 + k] - 500 for k in range(5)] + [0]
        op = _send_cmd(host, arm, vx=0.1, lift=20.0)
        flags = host.tick()
        assert flags & ST_WATCHDOG == 0
        assert host.goal[0] == start[0] + 80 and host.goal[6] == start[6] - 80       # step clamp
        assert host.goal[5] == min(hi, start[5] + 80) and host.goal[5] <= hi        # gripper fraction 4095 -> follower max
        assert host.wheels[8] != 0 and host.lift_v > 0
        for _ in range(12):
            host.tick()
        assert all(lo_k <= g <= hi_k for g, (lo_k, hi_k) in zip(host.goal, [host.limits[k] for k in range(12)]))
        assert host.goal[0] == min(host.limits[0][1], start[0] + 500)                # reached the target or its limit
        _settle(host)
        assert abs(left.w.d.ctrl[left.act[1]] - (host.goal[0] - 2048) * 2 * math.pi / 4096) < 1e-6   # twin got the goal
        st = unpack_state(op.recv(STATE_SIZE))
        assert st["echo_seq"] == 1 and (time.monotonic() - st["echo_t"]) * 1e3 < 2000
        # watchdog: 100 ms of silence stops base and lift and holds the arms
        time.sleep(0.15)
        held = list(host.goal)
        flags = host.tick()
        assert flags & ST_WATCHDOG and host.wheels == {i: 0 for i in WHEEL_IDS} and host.lift_v == 0
        assert host.goal == held
        # commands resume: the flag clears and goals move again
        _send_cmd(host, [2048] * 12, seq=2)
        assert host.tick() & ST_WATCHDOG == 0
    finally:
        host.close()
    assert not left.torque and not right.torque                                      # torque off on close


def test_host_no_torque_never_hands_goals_to_the_bus():
    from minihost.host import Host
    from minihost.sim_bus import SimBus
    left, right = SimBus("left", LEFT_IDS), SimBus("right", ARM_IDS)
    host = Host(left, right, no_torque=True, udp_port=0)
    try:
        _send_cmd(host, [2600] * 12, vx=0.2)
        for _ in range(3):
            host.tick()
        assert host.goal[0] > 2048                      # the loop still computes goals
        assert host.lt._goals is None and host.lt._vels is None   # but never hands them to the bus
        assert not left.torque and not right.torque
    finally:
        host.close()


def test_host_loop_on_emulated_boards_with_real_bus_and_sdk():
    """The whole host over the real Bus + scservo_sdk, with the twin behind LoopbackPorts."""
    from feetech_sim import loopback_board
    from minihost.bus import Bus
    from minihost.host import Host
    lport, lboard = loopback_board("left", LEFT_IDS, latency_ms=2.0)
    rport, rboard = loopback_board("right", ARM_IDS, latency_ms=2.0)
    left = Bus("left", LEFT_IDS, port_handler=lport)
    right = Bus("right", ARM_IDS, port_handler=rport)
    host = Host(left, right, hz=50, udp_port=0)
    try:
        assert lboard.mem[8]["Operating_Mode"] == 1 and lboard.mem[LIFT_ID]["Operating_Mode"] == 1
        assert all(lboard.mem[i]["Torque_Enable"] == 1 and lboard.mem[i]["Acceleration"] == 30 for i in ARM_IDS)
        start = list(host.goal)
        _send_cmd(host, [start[k] + 200 for k in range(12)], wz=0.5)
        for _ in range(3):
            host.tick()
            time.sleep(0.02)
        _settle(host)
        assert lboard.goal[1] == host.goal[0] and rboard.goal[1] == host.goal[6]      # goals reached the boards
        assert lboard.mem[8]["Goal_Velocity"] != 0                                     # wheels got a velocity
        instrs = {inst for _, inst, _ in lboard.log}
        assert {0x82, 0x83} <= instrs                                                  # sync read and sync write used
        assert host.summary.stats()["bus_l_ms"]["max"] < 20
    finally:
        host.close()
    assert all(lboard.mem[i]["Torque_Enable"] == 0 for i in ARM_IDS)
    assert lboard.mem[8]["Goal_Velocity"] == 0
