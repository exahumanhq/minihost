"""A fake Feetech servo board backed by the MuJoCo twin, reachable two ways.

    python tools/feetech_sim.py --arm right --ids 1,2,3,4,5,6
    -> (Linux/macOS) prints a /dev/pts/N path. Point Bus (or tools/measure_bus.py,
       or minihost.host --left/--right) at that path as if it were a USB board.

    LoopbackPort(FakeBoard(...))
    -> (any OS, in-process) a drop-in for the SDK's PortHandler. Pass it to
       Bus(..., port_handler=LoopbackPort(board)) and the real Bus and the real
       scservo_sdk run against the fake board with no serial device at all. This
       is how the tests run on Windows, where there is no pty.

Why this exists: `--sim` swaps Bus out entirely, so bus.py and the SDK never
run in the sim. This keeps the real Bus and the real scservo_sdk in the loop
and fakes the thing on the other end of the wire. It catches framing,
threading and timeout mistakes in bus.py. It cannot catch a wrong register
address or sign bit, because it was written from the same table as bus.py
(the working so101 scripts drove these arms with that table, which is the
actual evidence those are right).

Protocol, as read from scservo_sdk/protocol_packet_handler.py:
  request : FF FF ID LEN INSTR PARAMS... CHK      LEN = len(PARAMS)+2
  status  : FF FF ID LEN ERR   PARAMS... CHK
  CHK = ~(ID + LEN + INSTR/ERR + sum(PARAMS)) & 0xFF
  2-byte values little-endian (SCS_END = 0, what PacketHandler(0) sets).
  SYNC_READ (0x82) to id 0xFE: params = addr, len, id1, id2...; each servo
  answers in turn with its own status packet.
  SYNC_WRITE (0x83) to 0xFE: params = addr, len, then id + len bytes per servo;
  no reply.

--missing-id N (or board.missing.add(N)) makes servo N never answer, to
reproduce the "one unplugged motor makes every read wait ~34 ms" behaviour of
the SDK's timeout.
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from minihost.bus import REG, to_sign_mag
from minihost.sim_bus import SimBus

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 1, 2, 3, 0x82, 0x83
BROADCAST = 0xFE


def checksum(body):
    return (~sum(body)) & 0xFF


def status_packet(sid, params, err=0):
    body = [sid, len(params) + 2, err] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


class FakeBoard:
    def __init__(self, sim, ids, missing=()):
        self.sim = sim
        self.ids = ids
        self.missing = set(missing)
        # EEPROM/RAM that the twin doesn't model, kept per servo
        self.mem = {i: {"Torque_Enable": 0, "Acceleration": 0, "Goal_Velocity": 0, "Lock": 1, "Operating_Mode": 0,
                        "Min_Position_Limit": 0, "Max_Position_Limit": 4095} for i in ids}
        lim = sim.limits()
        for i in ids:
            self.mem[i]["Min_Position_Limit"], self.mem[i]["Max_Position_Limit"] = lim[i]
        self.goal = {i: 2048 for i in ids}
        self.vel = {}
        self.log = []   # (id, instr, params) of every request handled, for tests

    def registers(self, i):
        """Current value of every register we know, as a flat {addr: byte}."""
        s = self.sim.sync_read()[i]
        vals = dict(self.mem[i])
        vals["Goal_Position"] = self.goal[i]
        vals["Present_Position"] = s["pos"]
        vals["Present_Velocity"] = to_sign_mag(s["vel"], 15)
        vals["Present_Load"] = to_sign_mag(s["load"], 10)
        vals["Present_Voltage"] = int(s["volt"] * 10)
        vals["Present_Temperature"] = s["temp"]
        out = {}
        for name, (addr, n) in REG.items():
            v = int(vals.get(name, 0))
            for k in range(n):
                out[addr + k] = (v >> (8 * k)) & 0xFF
        return out

    def read(self, i, addr, n):
        r = self.registers(i)
        return [r.get(addr + k, 0) for k in range(n)]

    def write(self, i, addr, data):
        for name, (a, n) in REG.items():
            if a == addr:
                v = sum(b << (8 * k) for k, b in enumerate(data[:n]))
                if name == "Goal_Position":
                    self.goal[i] = v
                    if self.mem[i]["Torque_Enable"]:
                        self.sim.sync_write_goal({i: v})
                elif name == "Goal_Velocity":
                    self.mem[i][name] = v
                    if self.mem[i]["Operating_Mode"] == 1:
                        sign = -1 if v & 0x8000 else 1
                        self.sim.sync_write_velocity({i: sign * (v & 0x7FFF)})
                elif name == "Torque_Enable":
                    self.mem[i][name] = v
                    if v:
                        self.sim.prepare_follower()
                else:
                    self.mem[i][name] = v
                return

    def handle(self, sid, instr, params):
        """Return the bytes to send back for one request packet."""
        self.log.append((sid, instr, list(params)))
        if instr == SYNC_WRITE:
            addr, n = params[0], params[1]
            for k in range(2, len(params), n + 1):
                i = params[k]
                if i in self.mem:
                    self.write(i, addr, params[k + 1:k + 1 + n])
            return b""
        if instr == SYNC_READ:
            addr, n = params[0], params[1]
            out = b""
            for i in params[2:]:
                if i in self.mem and i not in self.missing:
                    out += status_packet(i, self.read(i, addr, n))
            return out
        if sid not in self.mem or sid in self.missing:
            return b""
        if instr == PING:
            return status_packet(sid, [])
        if instr == READ:
            return status_packet(sid, self.read(sid, params[0], params[1]))
        if instr == WRITE:
            self.write(sid, params[0], params[1:])
            return status_packet(sid, [])
        return b""


class Framer:
    """Cuts a byte stream into request frames and hands each to the board.
    Shared by the pty server and LoopbackPort so both parse identically."""

    def __init__(self, board, verbose=False):
        self.board = board
        self.verbose = verbose
        self.buf = b""

    def feed(self, chunk):
        """Return the concatenated replies for every complete frame in chunk."""
        self.buf += chunk
        out = b""
        while True:
            h = self.buf.find(b"\xff\xff")
            if h < 0:
                self.buf = b""
                return out
            if len(self.buf) < h + 4:
                self.buf = self.buf[h:]
                return out
            sid, ln = self.buf[h + 2], self.buf[h + 3]
            end = h + 4 + ln
            if len(self.buf) < end:
                self.buf = self.buf[h:]
                return out
            pkt = self.buf[h:end]
            self.buf = self.buf[end:]
            body = list(pkt[2:-1])
            if checksum(body) != pkt[-1]:
                if self.verbose:
                    print("bad checksum", pkt.hex(), flush=True)
                continue
            instr, params = pkt[4], list(pkt[5:-1])
            reply = self.board.handle(sid, instr, params)
            if self.verbose:
                print(f"id={sid} instr={instr:#x} params={params} -> {len(reply)}B", flush=True)
            out += reply


class LoopbackPort:
    """Stands in for scservo_sdk's PortHandler, in process, on any OS.

    Implements exactly the methods protocol_packet_handler and the group
    read/write classes call: is_using, clearPort, writePort, readPort,
    setPacketTimeout, isPacketTimeout, plus open/close/baud for Bus. The
    timeout formula is the SDK's own (tx time per byte + 2 x LATENCY_TIMER +
    2 ms), so a silent motor costs the same ~34 ms here as on hardware. Pass a
    smaller latency_ms to make tests faster; the behaviour is the same.
    """

    def __init__(self, board, latency_ms=16.0, baud=1_000_000, verbose=False):
        self.framer = Framer(board, verbose)
        self.rx = b""
        self.is_using = False
        self.latency_ms = latency_ms
        self.tx_time_per_byte = 1000.0 / baud * 10.0
        self.packet_timeout = 0.0
        self.packet_start = 0.0
        self.is_open = False

    # -- what Bus calls
    def openPort(self):
        self.is_open = True
        return True

    def closePort(self):
        self.is_open = False

    def setBaudRate(self, baud):
        self.tx_time_per_byte = 1000.0 / baud * 10.0
        return True

    def getPortName(self):
        return "loopback"

    # -- what the packet handler calls
    def clearPort(self):
        self.rx = b""

    def writePort(self, packet):
        self.rx += self.framer.feed(bytes(packet))
        return len(packet)

    def readPort(self, length):
        chunk, self.rx = self.rx[:length], self.rx[length:]
        return chunk

    def getBytesAvailable(self):
        return len(self.rx)

    def setPacketTimeout(self, packet_length):
        self.packet_start = time.perf_counter()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.latency_ms * 2.0) + 2.0

    def setPacketTimeoutMillis(self, msec):
        self.packet_start = time.perf_counter()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if (time.perf_counter() - self.packet_start) * 1e3 > self.packet_timeout:
            self.packet_timeout = 0
            return True
        return False


def serve(board, fd, verbose=False):
    framer = Framer(board, verbose)
    while True:
        try:
            chunk = os.read(fd, 4096)
        except OSError:
            return
        reply = framer.feed(chunk)
        if reply:
            os.write(fd, reply)


def open_board(arm, ids, missing=(), verbose=False):
    """Create the pty, start serving on a thread, return the slave path. POSIX only."""
    master, slave = os.openpty()
    # raw mode on the slave so nothing echoes or translates bytes
    import tty
    tty.setraw(slave)
    board = FakeBoard(SimBus(arm, ids), ids, missing)
    threading.Thread(target=serve, args=(board, master, verbose), daemon=True).start()
    return os.ttyname(slave), board


def loopback_board(arm, ids, missing=(), latency_ms=16.0):
    """FakeBoard plus a LoopbackPort for it: `Bus(arm, ids, port_handler=port)`."""
    board = FakeBoard(SimBus(arm, ids), ids, missing)
    return LoopbackPort(board, latency_ms=latency_ms), board


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="right", choices=["left", "right"])
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--missing-id", type=int, default=None)
    ap.add_argument("-v", action="store_true")
    a = ap.parse_args()
    if not hasattr(os, "openpty"):
        raise SystemExit("pty mode needs Linux or macOS; on Windows use LoopbackPort in-process (see tests)")
    ids = [int(x) for x in a.ids.split(",")]
    path, _ = open_board(a.arm, ids, [a.missing_id] if a.missing_id else (), a.v)
    print(path, flush=True)
    while True:
        time.sleep(1)
