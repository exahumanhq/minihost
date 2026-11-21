# Walkthrough, function by function

For each function: what it does, why we need it, what the old setup did
(Skyler's `Robotics/Aloha Mini` scripts plus the `lerobot_alohamini` host it
ran on the Pi), and whether this is better, worse, or just different. "Old"
means code I read in those repos; where I am going from the guide's prose
rather than code I say so.

Tested means: run here against the MuJoCo twin and the pty servo emulator.
Nothing has touched hardware.

---

## minihost/bus.py

### `from_sign_mag(v, bit)`, `to_sign_mag(v, bit)`
- Does: converts between the servo's sign-magnitude integers and Python ints. Velocity uses bit 15, load uses bit 10.
- Why: Present_Velocity, Present_Load and Goal_Velocity are signed on the wire this way, not two's complement. Get it wrong and a backwards wheel reads as a huge positive number.
- Old: LeRobot's Feetech driver has an encoding table doing the same per register, applied inside its `sync_read`.
- Verdict: same facts, less machinery. I copied the bit positions from their table; the pty emulator uses the same ones, so this is only confirmed on hardware.

### `Bus.__init__(port, ids, baud)`
- Does: opens the serial port at 1 Mbaud, builds one GroupSyncRead over the 8-byte block 56..63 (position, velocity, load, voltage, temperature) and two GroupSyncWrites (goal position, goal velocity).
- Why: one read packet returns everything the loop and the monitor need. Reading position alone costs the same bus time as reading all five.
- Old: LeRobot's `MotorsBus` builds a GroupSyncRead per register name on demand and caches it; the AlohaMini host called `sync_read("Present_Position")` then separately read voltage and load on a slower schedule, each its own transaction.
- Verdict: better for bus time (one transaction instead of up to three). Worse for flexibility (fixed block, STS3215 only). Tested through the pty: mean 0.7 ms per read with six servos, which is the emulator's speed not the bus's.

### `Bus.set_polled(ids)`
- Does: changes which motors the sync read asks for.
- Why: see `BusThread._run`. A silent motor costs ~34 ms every read and the SDK discards the replies of every motor behind it in id order while waiting. Confirmed on the emulator: motor 4 silent made 5 and 6 vanish and the read take 34.9 ms.
- Old: nothing. LeRobot raises and the host either crashes or, with its `safety_supervisor`, keeps retrying at 34 ms a read.
- Verdict: better. This is the fix for one of the ways the old host "was finicky".

### `Bus.read(name, i)`, `Bus.write(name, i, value)`
- Does: single-register read and write by name.
- Why: setup only (limits, torque, caps). Slow: one packet per call.
- Old: same shape in LeRobot (`read("Torque_Enable", motor)`).
- Verdict: same.

### `Bus.sync_read()`
- Does: one transaction, returns `{id: {pos, vel, load, volt, temp}}` for every motor that answered. Motors that didn't answer are simply absent. Raises only if nobody answered.
- Why: partial data is better than no data. My first version raised on any timeout and threw away five good motors for one bad cable.
- Old: LeRobot raises `ConnectionError` on any failed read. The AlohaMini host catches it and counts it toward a "safety supervisor" that eventually halts.
- Verdict: better. Tested: missing motor leaves the others in the dict.

### `Bus.sync_write_goal(goals)`, `Bus.sync_write_velocity(vels)`
- Does: one packet with every motor's goal position (or signed velocity).
- Why: twelve single writes would cost twelve bus transactions.
- Old: LeRobot `sync_write("Goal_Position", {...})`, same idea.
- Verdict: same. Tested through the pty: goals land on the emulator and the twin moves there.

### `Bus.set_velocity_mode(ids)`
- Does: writes Operating_Mode = 1 on wheels and lift.
- Why: wheels and the lead-screw lift are driven by speed, not position.
- Old: AlohaMini host does the same on connect (`OperatingMode.VELOCITY`).
- Verdict: same.

### `Bus.prepare_follower(accel, speed, gap_s)`
- Does: writes Acceleration and Goal_Velocity caps on every motor, then Torque_Enable one motor at a time with a 150 ms gap.
- Why: caps first so a servo with torque on and a far goal cannot snap. One at a time because six servos' in-rush current browned out the 12 V supply on this robot (Skyler's 2026-09-12 note in `so101/`).
- Old: Skyler's `teleop.py` does exactly this (he learned it the hard way). The lerobot host enables torque on all motors in one sync write and relies on `max_relative_target` to limit per-step motion instead of servo-side caps.
- Verdict: kept Skyler's version, which is the one that ran on this power supply. Tested: emulator sees accel=30 then torque=1 per motor.

### `Bus.torque_off()`
- Does: Torque_Enable 0 on every motor, ignoring errors.
- Why: it runs in the host's `finally`; a failed write to one motor must not stop the others going limp.
- Old: both old scripts disable torque on exit; Skyler's in a `finally`, lerobot's in `disconnect()`.
- Verdict: same. Tested via the Ctrl-C run: all torque registers 0 afterwards.

### `Bus.limits()`, `Bus.voltage()`
- Does: reads each servo's EEPROM Min/Max_Position_Limit; reads bus voltage from the first motor.
- Why: the servos are calibrated in-EEPROM on this robot (so101 README), so their own limits are the truth. Voltage decides whether torque is allowed at all.
- Old: Skyler's `arm.py` reads the same limits and intersects with a soft clamp; his `teleop.py` refuses to start under-volted. The lerobot host uses calibration JSON files instead of the servo's EEPROM.
- Verdict: Skyler's approach, kept. The so101 README is explicit that `lerobot-calibrate` would overwrite the in-servo calibration; nothing here writes EEPROM.

### `BusThread`
- Does: a thread that sync-reads in a loop, keeps `latest` (merged so a silent motor keeps its last value), writes queued goals and velocities right after each read, drops a motor after three consecutive misses and re-probes every 2 s, paces itself to at most 200 Hz.
- Why: the control loop must never block on serial. The bus is the slow unknown; the loop ticks at 50 Hz regardless. The drop-and-retry is the direct answer to the 34 ms stall.
- Old: lerobot host reads the bus inline in the control loop; a slow read is a late tick. GELLO (not on this robot) uses a bus thread like this one.
- Verdict: better. Tested: motor 4 goes silent, within ~60 ms it is dropped and reads are back under 1 ms; plug it back and within 2.5 s it is polled again. Without the pacing sleep the thread starved the control loop to 33 Hz in the pure-Python fake; with it, 50 Hz.

---

## minihost/loop.py

### `Ticker.wait()`
- Does: sleeps until an absolute deadline, advances it by dt, counts late ticks and the worst lateness. After a stall it rebases rather than running ticks back to back.
- Why: `sleep(dt)` after the work drifts (ALOHA); busy-waiting pins a core (GELLO's Rate). The old host used `sleep(max(0, dt - elapsed))`, which is fine until a tick runs long and then quietly loses the time. Rebasing on a miss avoids a burst of commands to the servos.
- Old: `busy_wait(max(0, 1/hz - elapsed))` in the AlohaMini host, which on their build was a busy loop.
- Verdict: better on CPU, same accuracy. Tested: 100 ticks at 200 Hz in 0.495 s with 0 to 1 late.

### `Summary.add(**vals)`, `Summary.maybe_print(prefix)`
- Does: averages whatever it is given over a second and prints one line.
- Why: the only way to know the loop is healthy on a Pi is a number per second you can read in `top`'s other pane.
- Old: `--profile_timing` in the AlohaMini host prints a similar per-second breakdown; off by default.
- Verdict: same idea, always on.

---

## minihost/base.py

### `OmniBase.wheel_speeds(vx, vy, wz)`
- Does: body velocity to ticks/s for wheels 8, 9, 10 using the three-omni-wheel formula; if any wheel exceeds its max, scales all three down together.
- Why: scaling together keeps the commanded direction. Clipping one wheel would make the base curve.
- Old: AlohaMini `_body_to_wheel_raw` is the same kinematics (I kept their angles and their sign flips); it clips per wheel against `max_raw`.
- Verdict: same kinematics, better saturation. Direction signs unverified; if W drives backwards, flip the signs here.

### `Lift.update(pos_ticks)`, `Lift.height_mm`, `Lift.velocity_ticks(mm_s)`
- Does: counts lead-screw turns from consecutive 0..4095 readings, reports height relative to the first reading, converts mm/s to ticks/s and zeros it at soft limits.
- Why: the servo only knows its angle. Something has to count turns.
- Old: AlohaMini `lift_axis.py` does the same unwrapping plus homing (drive down until stall), absolute height, and servo-side limits. It also does the "slam on start" homing move the guide warns about.
- Verdict: worse for now: no homing, relative height, conservative soft limits. Deliberate, so nothing slams on start. Homing is a later addition.

---

## minihost/link.py

### `pack_cmd` / `unpack_cmd`, `pack_state` / `unpack_state`
- Does: fixed `struct` layouts, about 60 bytes each, with sender timestamp and sequence; state echoes the command's timestamp.
- Why: RTT measured on one clock with no sync. Nothing executable on the wire.
- Old: JSON dicts over ZMQ. The old command carried no timestamp, so latency was never measured, only felt. State and JPEG frames went in the same message.
- Verdict: better for latency measurement and for not coupling video to control. Worse for extensibility (adding a field means editing both ends).

### `Udp.send`, `Udp.recv(size)`
- Does: non-blocking UDP; `recv` drains the socket and returns only the newest datagram, counting what it skipped.
- Why: a late command must be dropped, never queued. The old client's observation poll blocked up to 200 ms.
- Old: ZMQ PUSH/PULL with CONFLATE on commands (newest wins, same effect) and ROUTER/DEALER with a credit scheme for observations.
- Verdict: same behaviour on commands, far less code, one fewer dependency. Tested: five commands sent, newest received, `dropped == 4`.

---

## minihost/monitor.py

### `Monitor.update(goal, pos, load)`
- Does: two dwell-timed rules, stall (goal far from position for 0.6 s) and load (Present_Load high for 0.6 s), returning flags.
- Why: the robot-side "I need help" signal from the bench. Runs on fresh state next to the servos, 1 to 2 s ahead of a remote human on the simulated numbers.
- Old: nothing like it. The AlohaMini host's `safety_supervisor` watches bus errors and voltage, not task progress.
- Verdict: new. Thresholds are guesses until the first hardware session.

---

## minihost/host.py

### `main()`
- Does: opens both buses (or the twin with `--sim`), checks voltage, reads limits, sets wheel/lift velocity mode, enables torque with caps, starts the bus threads, then the 50 Hz loop: newest command, cached state, step-clamped goals inside the servo limits with the gripper fraction stretched onto the follower's range, watchdog that holds arms and stops base and lift, monitor flags, one state packet back, one summary line a second. `finally`: stop threads, zero velocities, torque off, close ports.
- Why: this is the whole robot-side program. No camera, no torch, no framework.
- Old: `alohamini_host.py`: same loop shape but with camera capture and JPEG encode inside it, torch imported through lerobot, a 100-minute self-exit, `Restart=always` in systemd, 1 s watchdog, no command timestamps.
- Verdict: better where it counts (nothing slow in the loop, nothing imported for no reason, latency measured, dead motors survived). Worse: no dataset recording, no LeRobot compatibility, no homing. Tested end to end on two pty boards with the real SDK: 50 Hz, 0 late ticks, RTT 20 ms over loopback, watchdog flag within one summary line of the operator quitting, torque registers 0 after Ctrl-C.

---

## minihost/sim_bus.py

### `SimBus` (same methods as `Bus`), `_World.advance()`
- Does: the MuJoCo twin pretending to be a servo board: goals in ticks become actuator targets in radians, joint angles come back as ticks, actuator force scaled to the 0..1023 load register, lift height faked as a wrapping encoder. Physics steps in real time whenever read.
- Why: lets `host.py --sim` run unchanged with servos that have inertia and limits.
- Old: nothing. The old setup had no way to run the host without the robot.
- Verdict: new. Its tick/radian zero (2048 = MJCF zero pose) is an assumption; the arms being calibrated to "2048 = middle" is confirmed by the so101 README, whether the MJCF zero is that middle is not.

---

## minihost/camera.py

### `Camera.attach/detach`, `Camera._reader`, `Camera.wait_frame`, `make_handler`
- Does: on first HTTP client, starts `ffmpeg -f v4l2 -input_format mjpeg ... -c:v copy -f mjpeg -`, splits the byte stream on JPEG SOI/EOI markers, keeps the newest frame; clients wait on a condition variable for a new frame and get it as a multipart/x-mixed-replace stream. Last client leaving stops ffmpeg.
- Why: the Pi should never decode or encode a frame. The old host decoded MJPG to BGR with OpenCV and re-encoded to JPEG at quality 90, per frame, per camera, every tick, with or without a client (on LeKiwi; AlohaMini at least only encoded on request). That is where most of the 116 percent went.
- Old: `cv2.VideoCapture` in a thread per camera at 30 fps, YUYV by default (the guide tells you to add `fourcc="MJPG"` yourself), `cv2.imencode` in a thread pool, frames inside the ZMQ observation message.
- Verdict: better on CPU by construction, and video can no longer delay a joint command. Worse: the stream is HTTP over TCP, so a bad link stalls video (not control); WebRTC is the later step. Tested with ffmpeg's test pattern through a raw socket; not with a camera.

---

## operator/teleop.py

### `FakeLeader`, `gripper_fraction(ticks, lo, hi)`, `main()`
- Does: reads both leader arms with the same `Bus` (torque never on), reads keys, converts grippers to a 0..4095 fraction of each leader's own calibrated range, sends one command datagram per tick, shows RTT, state age, flags, leader vs follower ticks.
- Why: the gripper fraction exists because the two grippers were calibrated to different ranges (so101 README: leader 1686..2945); raw ticks across would never fully close or would hit a stop. Every other joint is 2048-centred on both arms and goes across raw, as Skyler's `teleop.py` does.
- Old: Skyler's `teleop.py` maps raw ticks and handles the gripper range specially too, both arms on one laptop, no network. The lerobot client reads leaders through calibration JSONs and shows Rerun plots.
- Verdict: Skyler's mapping, over a link, with the latency number on screen. Worse: no camera in the window (open the camera URL in a browser next to it). Tested headless with `--fake-leader`; the drawing code has not been seen on a display.

---

## tools/measure_bus.py
- Does: timed sync reads, prints mean/median/worst, the SDK's timeout, voltage, limits, and load per motor. Never enables torque.
- Why: first thing to run on hardware. Gives the bus number nobody has published, and tells us whether `load` is a signal.
- Old: nothing equivalent; the guide asserts the loop is bus-bound without a figure.
- Verdict: new.

## tools/feetech_sim.py
- Does: a fake servo board on a pty speaking the Feetech protocol (ping, read, write, sync read, sync write, checksum, little-endian), backed by `SimBus`. `--missing-id` makes one servo silent.
- Why: so `bus.py` and the real SDK run without hardware. It found the partial-read and dead-motor behaviour.
- Old: nothing.
- Verdict: new. It cannot validate register addresses (written from the same table), only plumbing.

---

## What the old repo did that this does not (yet)
- LeRobot dataset recording and ACT training compatibility.
- Lift homing and absolute height.
- Five cameras; this has one `camera.py` process per camera.
- Rerun-based operator view with camera feeds inline.
- A systemd unit. Deliberately not yet.
