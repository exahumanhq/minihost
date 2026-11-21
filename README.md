# minihost

A small teleop host for the AlohaMini (two SO-101 follower arms, omni base, lift, cameras) meant to run on a Raspberry Pi 4 without pinning it. The operator side holds two SO-101 leader arms and a keyboard on a laptop. This README is the design doc: what the system has to do, why each piece is there, what to avoid, and where each piece ends up in the code once written. References are at the bottom.

Status: everything written for a first hardware session. Host, camera server and operator UI exist; the host and operator have run end to end against the MuJoCo twin (`--sim` / `--fake-leader`) at 50 Hz with 20 ms round trip over loopback. Nothing has run on hardware yet.

## What teleop here means

The robot has 12 arm joints (6 per arm), 3 base velocities (x, y, rotation) and a lift height. A person moves two leader arms and presses keys; the follower arms copy the leader joints, the base and lift follow the keys; the person watches camera video. All of that goes over a network link that may have real latency. Everything below follows from wanting that loop to be fast, safe and measurable on a weak computer.

## What is needed, and why

### Servo bus

- Read all follower joint positions in one sync read per bus, not one read per motor. One packet out, one back per bus; per-motor reads multiply bus time by six.
- Write all goal positions in one sync write per bus, same reason.
- Run the bus on its own thread at whatever rate the bus can sustain, and have the control loop read the latest cached values. The bus is the slow blocking thing; the loop should never wait on it. GELLO does exactly this with a background thread filling a shared array.
- Read Present_Position every cycle; read voltage and load less often (every 10 to 50 cycles). Voltage and load are needed for safety and the failure monitor, not for tracking.
- Set Acceleration and Goal_Velocity caps on every follower servo before enabling torque. Cheap servos will snap to a far goal at full speed otherwise.
- Enable torque one servo at a time with a short gap. Turning all six on at once browned out the 12 V supply on this hardware (Skyler's notes, 2026-09-12).
- Refuse to enable torque if the follower bus voltage is low. Under-volted servos cannot hold and the arm slumps.
- Soft start: before tracking, interpolate from where the follower is to where the leader is over about a second. Every system read does this (ALOHA move_arms, GELLO linspace ramp, Skyler's --ramp).
- Clamp each goal to the servo's own Min/Max_Position_Limit and to a per-cycle step limit. The leader can be moved faster than the follower should go.
- On any exit, including an exception, disable follower torque. Skyler's scripts do this in a finally block.

### Base and lift

- Base takes velocities, not positions. Keys map to velocity; releasing keys sends zero.
- Lift takes a height target; keys nudge it. Clamp to soft limits.
- Watchdog: if no command has arrived for a set time, send zero base velocity and hold the arms where they are. LeKiwi uses 1 s; this is probably too long for a base moving at walking pace, so make it a parameter and start lower.

### Network

- One datagram per tick each way. Commands are small (16 numbers). Newest wins; a late command is discarded, never queued. AlohaMini gets this with ZMQ CONFLATE; plain UDP gives it for free.
- Every command and every state packet carries the sender's monotonic timestamp and a sequence number. Without that, latency is a feeling, not a number. The whole point of the earlier bench was that the link delay is the thing to measure.
- Commands are JSON or a fixed struct, never pickle. Pickle executes what it receives.
- State goes back at the control rate without video in it. Video is a separate stream (below). LeKiwi puts JPEGs in the same message as state, so a slow frame delays joint feedback.
- Watchdog on the operator side too: if state stops arriving, show it, do not keep sending.

### Video

- Ask the camera for MJPG, never YUYV. On a Pi 4 all USB 2.0 devices share one 480 Mbit/s bus; raw 640x480x30 is about 147 Mbit/s per camera, MJPG about 9. The current AlohaMini config does not set this and the Pi guide tells you to add it by hand.
- Pass MJPG frames through to the network as they come off the camera. Do not decode to BGR and re-encode to JPEG on the Pi; that is pure CPU for nothing. This is the single biggest saving available.
- Only capture and send when an operator is connected. LeKiwi encodes every tick even with no client.
- Frame rate and camera count are runtime parameters. Start with one camera at 15 fps and go up once the loop is stable.
- For a remote operator (not same Wi-Fi), WebRTC is the right transport and both Stretch and TeleVision use it. It is a dependency worth taking, but second. First version: MJPG frames over UDP or a plain HTTP multipart stream, which any browser can show.

### Control loop timing

- Fixed rate, scheduled by absolute deadline: next = next + dt; sleep until next. Not sleep(dt) after the work (ALOHA; rate drifts) and not a busy-wait (GELLO's Rate class; pins a core, which is the 116 percent you saw). Not from any of the repos read; standard control-loop practice.
- Log every missed deadline with how late it was. Print a one-line summary once a second: achieved Hz, bus read ms, bus write ms, late ticks. AlohaMini's --profile_timing does something like this and it is the only way to know the loop is healthy.
- Use time.monotonic, never time.time, for anything that measures intervals.

### Process and dependencies

- No training stack in the control process. The current host imports lerobot, which imports torch at module load. Torch does nothing for teleop and costs hundreds of MB.
- Dependencies: pyserial (or the Feetech SDK), numpy, opencv only if a camera path needs it, nothing else on the Pi. Install with plain pip into a venv; conda on a Pi 4 is slow and large.
- One process. Threads for the servo bus and for each camera; the control loop is the main thread. Multiprocessing is not needed at this size.
- Run it by hand while developing. A systemd unit with Restart=always is fine later, but only after there is a way to see why it died (journal plus the once-a-second summary).

### Safety and failure monitor (from the bench)

- Robot-side stall detection: commanded minus measured position large while velocity is near zero for more than a dwell time. Flag it in the state packet.
- Load at rest: Present_Load high while the joint is still. Same flag.
- These run on the Pi next to the servos, so they see fresh state even when the operator is 300 ms away. The bench showed this decides 1 to 2 s before a remote human would.
- The flags are advisory in the first version: show them on the operator screen. Stopping or backing off automatically comes later, once false alarm rates are known on hardware.

### Operator side

- Read the two leader arms with the same bus code as the follower (same servos, same registers). Torque off on leaders.
- Keyboard for base and lift. Keys down means velocity; keys up means zero.
- Show: video, joint plot or numbers, measured round-trip latency, missed-tick count, monitor flags. The latency number is the one people will look at.
- Later: the MuJoCo twin as a predictive display, and the twin-versus-real residual next to it.

## What to avoid, in one place

- Per-motor serial reads. Six times the bus time.
- Encoding JPEG from raw frames on the Pi. Let the camera do it.
- Video and joint state in the same message.
- Queued commands. Stale commands move the arm to where the operator was.
- Pickle, or any format that executes on receive.
- sleep(dt) pacing, or busy-wait pacing.
- time.time for intervals.
- Torque on all servos at once.
- Torque on without acceleration and velocity caps set.
- Importing a framework whose import graph reaches torch.
- Always-on service before the process can tell you why it died.
- Hard-coded /dev paths. Use /dev/serial/by-id or udev names; both repos learned this.
- Trusting what any of the reference repos says about rates on a Pi. None of them measured it. Measure here.

## Open questions, stated as assumptions until checked

- Feetech bus: how long a 6-motor sync read takes at 1 Mbaud. The guide says the 50 Hz loop is bus-bound; nobody has published a number for this hardware. First thing to measure.
- Whether the SO-101 servos expose a usable Present_Load, and how noisy it is. Skyler's scripts never read it.
- Whether the Pi's USB cameras actually deliver MJPG at 640x480x30 (check with v4l2-ctl --list-formats-ext).
- Pi model and RAM. Behaviour at 2 GB and 4 GB differs.
- Whether dropping LeRobot compatibility is acceptable. This host cannot write LeRobot datasets; if ACT training is wanted later, a recorder is extra work.

## Where to find it

Fills in as files are written. Tests: `python -m pytest -q tests` (10 tests, no hardware needed). `WALKTHROUGH.md` explains every function and compares it to the old setup; `CHECKLIST.md` is the bring-up order.

Try it without a robot, three terminals:

```bash
python -m minihost.host --sim
python -m minihost.camera --test
python operator/teleop.py --host 127.0.0.1 --fake-leader
```

| Need | File | Notes |
|---|---|---|
| Feetech sync read/write, torque, caps | `minihost/bus.py` | `Bus` class. Uses feetech-servo-sdk. Register table at the top. Not run on hardware. |
| Background bus thread with cached state | `minihost/bus.py` | `BusThread`. Control loop reads `.latest`, calls `.set_goals()`. |
| Base and lift mapping | `minihost/base.py` | `OmniBase.wheel_speeds` (body velocity to ticks/s, scales down instead of clipping), `Lift` (unwraps turns, relative height, soft limits). Numbers are alohamini1 and unmeasured. |
| UDP command and state packets, timestamps, sequence | `minihost/link.py` | fixed `struct` layouts, `Udp.recv` drains and keeps the newest. State echoes the command timestamp so the operator computes RTT. |
| MJPG camera passthrough | `minihost/camera.py` | ffmpeg `-c:v copy` to an HTTP multipart stream any browser shows. Starts ffmpeg on first client, stops on last. `--test` uses a test pattern. Separate process from host.py by design. |
| Fixed-rate loop, deadline log, once-a-second summary | `minihost/loop.py`, `minihost/host.py` | `Ticker` and `Summary` in loop.py. `host.py` is the robot process: `python -m minihost.host --left ... --right ...`, start with `--no-torque`. Dry-run with a fake bus holds 51 Hz with 0 late ticks. |
| Stall and load monitor | `minihost/monitor.py` | two dwell-timed rules in ticks; flags go out in the state packet. Thresholds are guesses until measured. |
| Operator: leader arms, keyboard, display | `operator/teleop.py` | pygame window: RTT, state age, flags, leader vs follower ticks. WASD/QE base, RF lift, SPACE holds arms. `--fake-leader` for testing. Reuses `Bus` for the leaders, torque never enabled. |
| MuJoCo twin as a fake bus | `minihost/sim_bus.py` | `SimBus` has the same methods as `Bus`; `python -m minihost.host --sim` runs the real loop against it. Tick/radian mapping is an assumption noted in the file. |
| Fake servo board on a pty (real Bus + SDK, no hardware) | `tools/feetech_sim.py` | speaks the Feetech protocol, backed by the twin; `--missing-id` reproduces the silent-motor stall. |
| Bus timing measurement | `tools/measure_bus.py` | Run this first on the robot. Never enables torque. Prints sync-read time, SDK timeout, voltage, limits, load. |

## References

Code read directly (clone dates 2026-11-14):

- tonyzhaozh/aloha, `aloha_scripts/one_side_teleop.py`, `constants.py`
- MarkFzp/mobile-aloha, `aloha_scripts/record_episodes.py`
- wuphilipp/gello_software, `gello/env.py`, `gello/dynamixel/driver.py`, `gello/zmq_core/robot_node.py`, `experiments/run_env.py`
- huggingface/lerobot, `src/lerobot/robots/lekiwi/lekiwi_host.py`, `src/lerobot/cameras/opencv/`, `src/lerobot/motors/motors_bus.py`
- liyiteng/lerobot_alohamini, `src/lerobot/robots/alohamini/alohamini_host.py`, `config_alohamini.py`
- hello-robot/stretch_web_teleop, README and `nodes/`
- OpenTeleVision/TeleVision, `teleop/TeleVision.py`
- Ben369369/Exahuman, `hybrid_twin/`
- skylerlchan/Robotics, `Aloha Mini/so101/teleop.py`, `arm.py`, `pi4/`, `PI4_FULL_TELEOP_GUIDE.md`

Papers and docs, from memory, not re-read for this document:

- Sheridan and Ferrell, 1963 and 1965, remote manipulation under transmission delay (move-and-wait)
- Zhao, Kumar, Levine, Finn, ALOHA / ACT, RSS 2023
- Wu et al., GELLO, 2023
- Feetech STS3215 register map (Present_Position, Present_Load, Acceleration, Goal_Velocity, Torque_Enable); check against the datasheet before relying on addresses
- Raspberry Pi 4 USB: one shared USB 2.0 bus behind the VL805 controller (Raspberry Pi documentation)
