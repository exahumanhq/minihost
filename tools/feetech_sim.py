"""A fake Feetech servo board on a pseudo-terminal, backed by the MuJoCo twin.

    python tools/feetech_sim.py --arm right --ids 1,2,3,4,5,6
    -> prints a /dev/pts/N path. Point Bus (or tools/measure_bus.py, or
       minihost.host --left/--right) at that path as if it were a USB board.

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

--missing-id N makes servo N never answer, to reproduce the "one unplugged
motor makes every read wait ~34 ms" behaviour of the SDK's timeout.
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


def serve(board, fd, verbose=False):
    buf = b""
    while True:
        try:
            chunk = os.read(fd, 4096)
        except OSError:
            return
        buf += chunk
        while True:
            h = buf.find(b"\xff\xff")
            if h < 0:
                buf = b""
                break
            if len(buf) < h + 4:
                buf = buf[h:]
                break
            sid, ln = buf[h + 2], buf[h + 3]
            end = h + 4 + ln
            if len(buf) < end:
                buf = buf[h:]
                break
            pkt = buf[h:end]
            buf = buf[end:]
            body = list(pkt[2:-1])
            if checksum(body) != pkt[-1]:
                if verbose:
                    print("bad checksum", pkt.hex(), flush=True)
                continue
            instr, params = pkt[4], list(pkt[5:-1])
            reply = board.handle(sid, instr, params)
            if verbose:
                print(f"id={sid} instr={instr:#x} params={params} -> {len(reply)}B", flush=True)
            if reply:
                os.write(fd, reply)


def open_board(arm, ids, missing=(), verbose=False):
    """Create the pty, start serving on a thread, return the slave path."""
    master, slave = os.openpty()
    # raw mode on the slave so nothing echoes or translates bytes
    import termios, tty
    tty.setraw(slave)
    board = FakeBoard(SimBus(arm, ids), ids, missing)
    threading.Thread(target=serve, args=(board, master, verbose), daemon=True).start()
    return os.ttyname(slave), board


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="right", choices=["left", "right"])
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--missing-id", type=int, default=None)
    ap.add_argument("-v", action="store_true")
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]
    path, _ = open_board(a.arm, ids, [a.missing_id] if a.missing_id else (), a.v)
    print(path, flush=True)
    while True:
        time.sleep(1)
