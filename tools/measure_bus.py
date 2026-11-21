"""First thing to run on the robot. Answers the question the README leaves open:
how long does one sync read of N servos take on this bus, and what does a
missing motor cost.

    python tools/measure_bus.py /dev/serial/by-id/usb-...  --ids 1,2,3,4,5,6

Prints, for a few hundred reads: mean, median, worst, and the SDK's own packet
timeout so you can see how a non-answering motor would stall the loop. Then
reads voltage, limits and load once so we learn whether Present_Load is a real
signal on these servos (nothing I read ever uses it).

Never enables torque. Safe to run on the leader or the follower.
"""
import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from minihost.bus import Bus
from scservo_sdk import port_handler

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--ids", default="1,2,3,4,5,6")
ap.add_argument("--n", type=int, default=300)
args = ap.parse_args()
ids = [int(x) for x in args.ids.split(",")]

bus = Bus(args.port, ids)
print(f"port {args.port}, ids {ids}")
print(f"SDK LATENCY_TIMER = {port_handler.LATENCY_TIMER} ms, so a motor that never answers "
      f"costs about {2 * port_handler.LATENCY_TIMER + 2} ms per read before the SDK gives up")

# warm up: the first read after opening the port is often slow
for _ in range(5):
    bus.sync_read()

times = []
for _ in range(args.n):
    t0 = time.perf_counter()
    bus.sync_read()
    times.append((time.perf_counter() - t0) * 1e3)
print(f"sync read of {len(ids)} motors, {args.n} samples: "
      f"mean {statistics.mean(times):.2f} ms, median {statistics.median(times):.2f} ms, "
      f"worst {max(times):.2f} ms")
print(f"-> at that rate the bus alone allows about {1000 / statistics.mean(times):.0f} reads/s; "
      f"a 50 Hz loop needs one read and one write per 20 ms")

# one-off reads that tell us about the servos
data = bus.sync_read()
print(f"voltage {bus.voltage():.1f} V (follower should be ~12, leader on USB ~5)")
lim = bus.limits()
for i in ids:
    d = data[i]
    print(f"id {i}: pos {d['pos']:4d}  vel {d['vel']:5d}  load {d['load']:5d}  temp {d['temp']}C  "
          f"limits {lim[i][0]}..{lim[i][1]}")
print("hold a joint still, then push on it gently and re-run: if 'load' moves, the failure monitor has its signal")
bus.close()
