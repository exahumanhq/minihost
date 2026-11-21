"""One Feetech serial bus with a group of STS3215 servos on it.

This is the only file that talks to servos. Everything else asks it for the
latest positions or hands it goals. I am using the official feetech-servo-sdk
(scservo_sdk) for the packet layer because writing the checksum and framing by
hand is easy to get subtly wrong and I can't test on hardware from here. The
SDK's GroupSyncRead and GroupSyncWrite are the two calls that matter: one
packet out, N replies back, for N motors. That is what keeps bus time low.

Register addresses are from LeRobot's Feetech table for the STS3215 (see
README references). I have not checked them against the Feetech datasheet
myself, so if a read looks wrong, that is the first place to look.

A note on the SDK that cost people time before: port_handler.LATENCY_TIMER is
16 ms, and the packet timeout is 2x that plus transfer time. On a bus where a
motor never answers, every sync read waits ~34 ms before giving up. That is
why a missing motor makes the whole loop crawl. measure_bus.py prints the
timeout so you can see it.
"""
import threading
import time

from scservo_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler

# STS3215 control table: name -> (address, bytes). EEPROM ones need Lock=0 to change.
REG = {
    "Min_Position_Limit": (9, 2),
    "Operating_Mode": (33, 1),
    "Max_Position_Limit": (11, 2),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Velocity": (46, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
}

# The STS protocol packs signed 2-byte values as sign-magnitude: one bit is the
# sign, the rest the size. Which bit differs per register (LeRobot's encoding
# table): velocity and goal velocity use bit 15, load uses bit 10 because load
# is only 0..1023. Verify on hardware with measure_bus.py by pushing a joint
# both ways and watching the sign of 'load'.
def from_sign_mag(v, bit):
    mask = (1 << bit) - 1
    return -(v & mask) if v & (1 << bit) else v


def to_sign_mag(v, bit):
    return (abs(int(v)) & ((1 << bit) - 1)) | ((1 << bit) if v < 0 else 0)


class Bus:
    """Open one serial port, know a list of motor ids, do sync reads and writes.

    Positions are raw ticks (0..4095, 2048 = middle). No calibration here on
    purpose; Skyler's arms are calibrated inside the servos (Homing_Offset and
    Min/Max_Position_Limit are set), so raw ticks already line up leader to
    follower. If that ever changes, the mapping belongs in the host, not here.
    """

    def __init__(self, port, ids, baud=1_000_000):
        self.ids = list(ids)
        self.port = PortHandler(port)
        if not self.port.openPort():
            raise RuntimeError(f"could not open {port}")
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"could not set baud {baud} on {port}")
        # protocol_end=0 is what LeRobot passes for STS3215. It picks byte order.
        self.ph = PacketHandler(0)
        # one reader for the 8-byte block Present_Position..Present_Load (56..63 is
        # position, velocity, load, voltage, temperature; reading them all at once
        # costs the same as reading position alone, one packet per motor)
        self.reader = GroupSyncRead(self.port, self.ph, REG["Present_Position"][0], 8)
        self.polled = list(self.ids)
        for i in self.ids:
            self.reader.addParam(i)
        self.writer = GroupSyncWrite(self.port, self.ph, REG["Goal_Position"][0], 2)
        self.vel_writer = GroupSyncWrite(self.port, self.ph, REG["Goal_Velocity"][0], 2)
        self.lock = threading.Lock()   # the SDK is not thread safe; one call at a time

    def set_polled(self, ids):
        """Change which motors the sync read asks for. Used to drop a motor
        that has stopped answering: the SDK waits its full timeout for a silent
        motor and, because replies come in id order, also discards every reply
        after it while waiting. One dead motor otherwise costs ~34 ms and the
        data of every motor behind it, every single read."""
        self.reader.clearParam()
        for i in ids:
            self.reader.addParam(i)
        self.polled = list(ids)

    # ---- single-motor reads and writes (setup only, slow: one packet per call)
    def read(self, name, i):
        addr, n = REG[name]
        with self.lock:
            if n == 1:
                v, res, err = self.ph.read1ByteTxRx(self.port, i, addr)
            else:
                v, res, err = self.ph.read2ByteTxRx(self.port, i, addr)
        if res != COMM_SUCCESS:
            raise IOError(f"read {name} id {i}: {self.ph.getTxRxResult(res)}")
        return v

    def write(self, name, i, value):
        addr, n = REG[name]
        with self.lock:
            if n == 1:
                res, err = self.ph.write1ByteTxRx(self.port, i, addr, int(value))
            else:
                res, err = self.ph.write2ByteTxRx(self.port, i, addr, int(value))
        if res != COMM_SUCCESS:
            raise IOError(f"write {name} id {i}: {self.ph.getTxRxResult(res)}")

    # ---- the two calls the control loop uses
    def sync_read(self):
        """Return {id: dict(pos, vel, load, volt, temp)} for every motor that answered.

        One bus transaction. If a motor doesn't answer, the SDK reports a
        timeout for the whole call, but the motors that did answer are still
        in its buffer, so we keep those and leave the silent one out. The
        caller sees which ids are missing by their absence. Raising on the
        whole read (what I first wrote) would have thrown away five good
        motors because of one bad cable.
        """
        with self.lock:
            res = self.reader.txRxPacket()
        out = {}
        g = self.reader.getData
        for i in self.polled:
            if not self.reader.isAvailable(i, 56, 8):
                continue
            out[i] = {
                "pos": g(i, 56, 2),
                "vel": from_sign_mag(g(i, 58, 2), 15),
                "load": from_sign_mag(g(i, 60, 2), 10),
                "volt": g(i, 62, 1) / 10.0,   # register is in 0.1 V
                "temp": g(i, 63, 1),
            }
        if not out:
            raise IOError(f"sync read: {self.ph.getTxRxResult(res)}")
        return out

    def sync_write_goal(self, goals):
        """goals: {id: ticks}. One packet for all motors."""
        self.writer.clearParam()
        for i, g in goals.items():
            g = int(g)
            self.writer.addParam(i, [g & 0xFF, (g >> 8) & 0xFF])
        with self.lock:
            res = self.writer.txPacket()
        if res != COMM_SUCCESS:
            raise IOError(f"sync write: {self.ph.getTxRxResult(res)}")

    def sync_write_velocity(self, vels):
        """vels: {id: signed ticks/s}. For motors in velocity mode (wheels, lift)."""
        self.vel_writer.clearParam()
        for i, v in vels.items():
            raw = to_sign_mag(v, 15)
            self.vel_writer.addParam(i, [raw & 0xFF, (raw >> 8) & 0xFF])
        with self.lock:
            res = self.vel_writer.txPacket()
        if res != COMM_SUCCESS:
            raise IOError(f"sync write velocity: {self.ph.getTxRxResult(res)}")

    def set_velocity_mode(self, ids):
        """Operating_Mode 1 = constant speed mode (0 = position). Needed for wheels and lift."""
        for i in ids:
            self.write("Operating_Mode", i, 1)

    # ---- safety setup, done once before tracking starts
    def prepare_follower(self, accel=30, speed=1500, gap_s=0.15):
        """Cap acceleration and speed, then enable torque one motor at a time.

        Why one at a time with a gap: six servos pulling in-rush current together
        browned out the 12 V supply on this robot (Skyler's notes, 2026-09-12).
        Why caps before torque: a servo with torque on and no caps will go to
        its goal as fast as it physically can.
        """
        for i in self.ids:
            self.write("Acceleration", i, accel)
            self.write("Goal_Velocity", i, speed)
        for i in self.ids:
            self.write("Torque_Enable", i, 1)
            time.sleep(gap_s)

    def torque_off(self):
        for i in self.ids:
            try:
                self.write("Torque_Enable", i, 0)
            except IOError:
                pass   # we are shutting down; try every motor regardless

    def limits(self):
        """{id: (min_ticks, max_ticks)} from the servos' own EEPROM limits."""
        return {i: (self.read("Min_Position_Limit", i), self.read("Max_Position_Limit", i)) for i in self.ids}

    def voltage(self):
        return self.read("Present_Voltage", self.ids[0]) / 10.0

    def close(self):
        self.port.closePort()


class BusThread:
    """Keeps `latest` fresh by sync-reading in a loop on its own thread.

    The control loop reads `latest` (a dict) and never waits for the serial
    port. Goals go the other way: the loop calls `set_goals` and the thread
    writes them right after its next read. So each thread iteration is one
    read and at most one write, which is the minimum the bus can do anyway.

    GELLO does this with Dynamixels. The reason is simple: a sync read blocks
    for however long the bus takes (unknown on this hardware until measured),
    and the control loop should tick at a fixed rate regardless.
    """

    def __init__(self, bus, max_hz=200):
        self.bus = bus
        self.min_period = 1.0 / max_hz   # don't poll faster than this even if the bus could
        self.latest = None          # {id: {...}} from the most recent sync read
        self.latest_t = 0.0         # time.monotonic() of that read
        self._goals = None
        self._vels = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.read_ms = 0.0          # last read duration, for the once-a-second summary
        self.errors = 0
        self.missing = []           # ids currently dropped from polling because they stopped answering
        self.misses = {}
        self.retry_at = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True, name="bus")

    def start(self):
        self.thread.start()

    def stop(self):
        self._stop.set()
        self.thread.join(timeout=1.0)

    def set_goals(self, goals):
        with self._lock:
            self._goals = dict(goals)

    def set_velocities(self, vels):
        with self._lock:
            self._vels = dict(vels)

    def _run(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                data = self.bus.sync_read()
                if self.latest is None:
                    self.latest = data
                else:
                    self.latest = {**self.latest, **data}   # motors that didn't answer keep their last value
                self.latest_t = time.monotonic()
                for i in self.bus.polled:
                    self.misses[i] = 0 if i in data else self.misses.get(i, 0) + 1
                # Drop only the FIRST silent motor: the ones behind it in id order
                # were probably fine and only lost because the SDK discarded their
                # replies while waiting for it. They get read normally next time.
                dead = [i for i in self.bus.polled if self.misses.get(i, 0) >= 3][:1]
                if dead:
                    self.bus.set_polled([i for i in self.bus.polled if i not in dead])
                    for i in dead:
                        del self.misses[i]
                    self.retry_at = time.monotonic() + 2.0
                self.missing = [i for i in self.bus.ids if i not in self.bus.polled]
                if self.missing and time.monotonic() > self.retry_at:
                    # give dropped motors another chance now and then; a cable may be back
                    self.bus.set_polled(self.bus.ids)
                    self.misses = {}
                    self.retry_at = time.monotonic() + 2.0
            except IOError:
                self.errors += 1
            self.read_ms = (time.monotonic() - t0) * 1e3
            with self._lock:
                goals, self._goals = self._goals, None
                vels, self._vels = self._vels, None
            try:
                if goals:
                    self.bus.sync_write_goal(goals)
                if vels:
                    self.bus.sync_write_velocity(vels)
            except IOError:
                self.errors += 1
            # On real hardware the serial read blocks and this loop is paced by
            # the bus. The sleep is for the case where reads come back fast (or
            # a fake bus in tests): without it this thread spins, and in Python
            # a spinning thread starves the control loop of the interpreter.
            time.sleep(max(0.0, self.min_period - (time.monotonic() - t0)))
