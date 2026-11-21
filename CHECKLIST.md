# Bring-up checklist

Three checkpoints. Each one has to pass before the next one is worth starting,
because every later failure gets blamed on the layer below otherwise.

## Checkpoint 1: software only, no robot (done 2026-11-21, re-run after the pty emulator was added)

Everything here runs on a laptop. If any of it fails, the problem is in this
repo, not the hardware.

- [x] `pip install -r requirements.txt` in a fresh venv. Pulls feetech-servo-sdk, numpy, mujoco, pygame.
- [x] `python -m pytest -q tests` passes. 15 tests: sign-magnitude round trip, packet pack/unpack, UDP newest-wins over loopback, wheel kinematics, lift unwrap and soft limits, monitor dwell, summary percentiles, twin tracks a goal and shows load when pushed, the real `Bus` and SDK against the servo emulator twice (pty on Linux/macOS, in-process `LoopbackPort` on any OS: reads, torque with caps, a motor going silent costs the SDK's ~34 ms once and is dropped within three reads and recovered within 2.5 s), gripper range fraction, and the host loop itself three ways (on the twin: step clamp, gripper mapping, limits, watchdog trip and recovery, torque off on close; `--no-torque` hands nothing to the bus; over the real Bus + SDK on two emulated boards: velocity mode, caps, goals and wheel speeds land on the boards). On Windows 14 run and the pty one skips.
- [x] Two bugs the new tests found (2026-11-21): `--sim` crashed the bus thread after its first read because `SimBus` had no `polled`, so the twin never moved; and the SDK keeps each motor's bytes from the previous sync read, so motors behind a silent one were reported with stale positions instead of missing. Both fixed in `sim_bus.py` and `Bus.sync_read`.
- [x] `python -m minihost.host --sim` holds 50 Hz with `late_ms=0.0` for several seconds and exits with "torque off, ports closed" on Ctrl-C.
- [x] `python operator/teleop.py --host 127.0.0.1 --fake-leader --headless --seconds 3` against the sim host prints `rtt` about 20 ms (one host tick) and follower ticks within a few of the leader.
- [x] Watchdog: kill the operator while the sim host runs; the host's `wd=` column goes to 1 within one summary line.
- [x] `python -m minihost.camera --test`, then open `http://127.0.0.1:8080/` in a browser: test pattern moves. Host log shows "started ffmpeg for first client" and "stopped ffmpeg" when the tab closes. (Verified here with a raw socket, not a browser: frames arrive with the multipart boundary; the first takes about a second because ffmpeg is starting.)
- [x] Real `Bus` + real scservo_sdk against `tools/feetech_sim.py` on two pty boards, driving `host.py` with no `--sim`: 50 Hz, 0 late ticks, RTT 20 ms, torque registers set with caps on start and cleared on Ctrl-C, wheel and lift servos put in velocity mode.
- [x] Reproduced the SDK's silent-motor stall on the emulator: one unanswering servo makes a sync read take 34.9 ms and loses every servo behind it in id order. `BusThread` now drops that motor after three misses and re-probes every 2 s.
- [x] The pygame window opens on a machine with a display. Not verifiable here (no display); the headless path runs, so the only untested lines are the ones that draw.

Known limits of this checkpoint: the emulator was written from the same
register table as `bus.py`, so it confirms plumbing, not addresses. It says
nothing about real bus timing, Present_Load behaviour, wheel signs or Pi CPU.

Settled by the old scripts having run on this robot, so checkpoint 2 only
confirms them: register table and 1 Mbaud (Skyler's `so101/teleop.py` drove
these arms through the same LeRobot table); 2048 = middle pose with
calibration stored in the servos (so101 README, three places); EEPROM limits
are real; wheels 8/9/10 and lift 11 on the left board. Not settled, only
claimed: that the cameras do MJPG at 640x480 comes from the guide's prose, and
the upstream camera config never set a fourcc, so it stays a checkpoint-2
check (`v4l2-ctl --list-formats-ext`). Do not run `lerobot-calibrate` on these
arms: it overwrites the in-servo calibration (so101 README).

## Checkpoint 2: each piece of hardware alone, nothing moves

On the Pi for the follower side, on the laptop for the leaders. Torque is
never enabled in this checkpoint.

- [ ] Pi: `ls /dev/serial/by-id/` shows two boards; note which is left (arm + wheels + lift) and which is right.
- [ ] Pi: `python tools/measure_bus.py <left-port> --ids 1,2,3,4,5,6,8,9,10,11`. Record mean and worst sync-read time. The 50 Hz loop needs read plus write under 20 ms with margin.
- [ ] Pi: same on the right port with ids 1 to 6.
- [ ] Both boards report voltage near 12 V with the supply on. If it reads near 5 V the 12 V supply is off and torque must stay off.
- [ ] Confirm: hold each arm at its middle pose, positions read near 2048. Expected from the so101 README; if not, something rewrote the calibration.
- [ ] Record each gripper's EEPROM limits (leader was 1686..2945 per the README). The operator maps the leader gripper onto the follower's range from these.
- [ ] Push gently on a joint, re-run `measure_bus.py`: `load` changes magnitude and sign with direction. If it never moves, the load rule in the monitor has no signal on this hardware and `load` can be dropped.
- [ ] Confirm: each joint's `limits` from EEPROM are narrower than 0..4095 (expected; the so101 scripts rely on them).
- [ ] Laptop: `measure_bus.py` on both leader ports. Positions change as you move the arm; voltage will read low (USB powered), which is fine for leaders.
- [ ] Confirm: `v4l2-ctl --list-formats-ext -d /dev/video0` lists MJPG at 640x480 (expected; the guide's MJPG fix worked on these cameras).
- [ ] Pi: `python -m minihost.camera --dev /dev/video0 --fps 15`, open from the laptop browser. `top` on the Pi while streaming: ffmpeg should be a few percent, not tens.
- [ ] Pi: `python -m minihost.host --no-torque --left ... --right ...`. Summary line shows `hz=50`, `bus_l_ms` and `bus_r_ms` matching measure_bus, `late_ms` near 0. Run `top` alongside: the host process should be well under one core.
- [ ] Laptop: `operator/teleop.py --host <pi> --left ... --right ...` against that no-torque host. Window shows leader ticks moving, follower ticks static, RTT under 50 ms on lab Wi-Fi. Record the RTT; it is the first real latency number.

## Checkpoint 3: torque on, one thing at a time

Someone stands by the power switch. Arms start folded in a known pose.

- [ ] Unplug one follower servo's cable with the host running `--no-torque`: `dead_motors` goes to 1 in the summary, `bus_*_ms` stays low, the other joints keep updating on the operator window. Plug back: clears within a few seconds.
- [ ] Right arm only: start host with torque, left leader unplugged (host holds left where it is). Move the right leader slowly. Follower follows, no oscillation, no grinding at joint limits. If a joint runs the wrong way, the tick mapping for that joint is mirrored.
- [ ] Step clamp: jerk the leader. Follower moves at the clamped speed, not a snap. Adjust `--max-step`.
- [ ] Watchdog with torque: pull the laptop's network. Arms hold, base and lift stop, operator window shows state age climbing. Reconnect; control resumes without a jump (the host's goal is still where the arm is).
- [ ] Left arm the same way. Then both.
- [ ] Base: W moves forward. If not, flip the sign convention in `OmniBase.wheel_speeds`. Then A/D, then Q/E. Check all three wheels turn for pure rotation.
- [ ] Lift: R raises, F lowers, and it stops at the soft limits. Set `soft_min`/`soft_max` to the real travel once homed.
- [ ] Monitor: hold a follower joint by hand while the leader moves. STALL flag appears on the operator window after the dwell time and clears when released. Note false alarms during normal motion; raise `err_ticks` or `stall_s` if any.
- [ ] Load flag: lean the gripper on the table. LOAD appears, clears when lifted.
- [ ] Camera and control together over lab Wi-Fi for ten minutes. Host `late_ms` stays near 0, `dropped` stays low, Pi stays under one core and well under full memory. Compare with the old host's 116 percent.
- [ ] Ctrl-C the host mid-motion: torque off, arms go limp onto the table, ports close. This is the shutdown path everything relies on.

After checkpoint 3 the system is at the point the simulation bench started
from: a real link with a measured delay, and a monitor that can be tested
against real stuck-on-the-bin failures.
