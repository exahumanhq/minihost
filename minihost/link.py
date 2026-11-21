"""The network link: one UDP datagram per tick in each direction.

Why UDP and not TCP or ZMQ: a teleop command is only useful for the next
20 ms. If a packet is lost, the next one supersedes it anyway, so there is
nothing to retransmit. TCP would stall the whole stream to redeliver a stale
command. ZMQ's CONFLATE option gets the same newest-wins behaviour, but UDP
gives it with no dependency and a fixed, inspectable packet layout.

Why struct and not JSON: fixed layout, about 60 bytes, parsed in microseconds,
and nothing in it can be executed on receive (unlike pickle).

Every packet carries the sender's monotonic clock and a sequence number. The
state packet echoes the last command's timestamp back, so the operator can
compute round trip time as (now - echoed) without the two clocks agreeing.
That number is the whole reason this link exists: latency measured, not felt.
"""
import socket
import struct
import time

N_ARM = 12   # 6 joints per arm, left then right

# command: seq u32, t_send f64, 12 arm goals u16, vx vy wz f32, lift_mm_s f32, flags u8
CMD_FMT = "<Id12H4fB"
# state: seq u32, t_send f64, echo_seq u32, echo_t f64, 12 pos u16, 12 load i16, volt f32, flags u8
STATE_FMT = "<IdId12H12hfB"
CMD_SIZE = struct.calcsize(CMD_FMT)
STATE_SIZE = struct.calcsize(STATE_FMT)

# command flags
CMD_ARMS_ACTIVE = 1   # operator is holding the leaders; if clear the host holds the arms still
# state flags (the monitor's "I need help" bits live here)
ST_WATCHDOG = 1
ST_STALL = 2
ST_LOAD = 4
ST_LOW_VOLT = 8


def pack_cmd(seq, arm_goals, vx, vy, wz, lift_mm_s, flags):
    return struct.pack(CMD_FMT, seq, time.monotonic(), *arm_goals, vx, vy, wz, lift_mm_s, flags)


def unpack_cmd(b):
    v = struct.unpack(CMD_FMT, b)
    return {"seq": v[0], "t": v[1], "arm": list(v[2:14]), "vx": v[14], "vy": v[15], "wz": v[16],
            "lift": v[17], "flags": v[18]}


def pack_state(seq, echo_seq, echo_t, pos, load, volt, flags):
    return struct.pack(STATE_FMT, seq, time.monotonic(), echo_seq, echo_t, *pos, *load, volt, flags)


def unpack_state(b):
    v = struct.unpack(STATE_FMT, b)
    return {"seq": v[0], "t": v[1], "echo_seq": v[2], "echo_t": v[3], "pos": list(v[4:16]),
            "load": list(v[16:28]), "volt": v[28], "flags": v[29]}


class Udp:
    """Non-blocking UDP socket. recv() drains everything waiting and returns only
    the last datagram, which is the newest-wins rule in one place."""

    def __init__(self, bind_port=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if bind_port is not None:
            self.sock.bind(("0.0.0.0", bind_port))
        self.sock.setblocking(False)
        self.peer = None       # who last talked to us; the host replies there
        self.dropped = 0       # datagrams skipped because a newer one was waiting

    def send(self, data, addr=None):
        addr = addr or self.peer
        if addr:
            self.sock.sendto(data, addr)

    def recv(self, size):
        last = None
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                return last
            if len(data) != size:
                continue           # not ours; ignore rather than crash
            if last is not None:
                self.dropped += 1
            last, self.peer = data, addr
