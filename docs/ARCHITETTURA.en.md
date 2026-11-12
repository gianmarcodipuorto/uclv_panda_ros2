<p align="right">
  <a href="ARCHITETTURA.md"><img src="https://img.shields.io/badge/Italiano-8c959f?style=for-the-badge" alt="Italiano"></a>
  <a href="ARCHITETTURA.en.md"><img src="https://img.shields.io/badge/English-0969da?style=for-the-badge" alt="English"></a>
</p>

# How the bridges work

This page is for those who change the code. To use the driver, the [README](../README.en.md) is enough.

## The path of a command

```
client ──topic──▶ subscription ──lock-free queue──▶ 1 kHz loop (libfranka thread) ──▶ robot
                                                     │  jitter buffer: plays the samples back
                                                     │    according to header.stamp
                                                     │  tracker: makes the command acceptable
                                                     │    to the robot
/joint_states, /panda/robot_state ◀── publisher ◀────┘  loop state, lock-free
```

Each bridge uses three threads:

- **Robot thread.** With the bridge active it runs `robot.control()`, which libfranka raises to SCHED_FIFO 99. With the bridge inactive it reads the state with `readOnce()` at `publish_freq`.
- **Publisher thread.** It publishes state and status at `publish_freq` and writes the loop events to the log.
- **Executor.** A MultiThreadedExecutor with 4 threads: lifecycle, commands, actions and services, recovery.

The 1 kHz callback has no allocations, locks, logging or ROS calls. Commands and state go through lock-free queues and non-blocking boxes.

## State machine and trackers

All bridges share the same state machine (holding, moving, stopping, finished), in `pipeline.hpp/.cpp`. Only the **tracker** changes, that is, how the reference becomes a command the robot accepts:

| Bridge | Tracker | What it does |
|---|---|---|
| positions | `JointTracker` | checks the velocity, acceleration and jerk of each command as the robot does (finite differences); if the command fails the check, it converges to the reference with Ruckig |
| velocities | `JointVelocityTracker` | the same check, one order lower; it integrates the velocities as the robot does and brakes before the joint limits |
| torques | `JointTorqueTracker` | saturates at the torque limits and limits the derivative (the robot rejects discontinuities); in internal motions it is a joint impedance |
| Cartesian pose | `CartesianPoseTracker` | the robot derives the twist from two consecutive poses and checks the norms of translational, rotational and elbow velocity, acceleration and jerk: the tracker does the same; if the command fails the check, it converges with Ruckig |
| Cartesian velocity | `CartesianVelocityTracker` | the same check on the twist; it integrates the twist as the robot does |

If Ruckig finds no profile that reaches the target acceleration (this happens when it is exactly at the limit), the tracker retries aiming only at the target velocity, which is always reachable. Only if that fails too does the tracker count an error and hold the command.

## The loop in external mode

- The first command continues the robot's current one: `q_d` for the joints, `O_T_EE_c` for the poses, as in the libfranka examples.
- Each sample is played back `playout_delay_ms` after its stamp. The playback time is the robot clock (exactly 1 ms per cycle), aligned to the PC clock: this way network jitter does not reach the robot.
- Between samples the reference is a local quadratic fit (positions, poses, velocities) or a linear interpolation (torques). For sparse streams the fit window widens by itself.
- A sample the robot accepts is sent as it is. Otherwise the tracker adapts it, and the cycle is counted in `limited_cycles`.
- Short gaps in the stream (less than `stall_timeout_ms`) are bridged: positions, poses and velocities by constant-acceleration extrapolation, torques by holding the last sample. Beyond that, the robot stops smoothly with a jerk-limited braking (for torques, with a `-D·dq` damping). Then it holds its position; the stream restarts with new samples.
- A stream of positions or poses that starts more than `max_jump` away from the current command is rejected.
- If the end effector goes below `critical_height`, the robot stops smoothly and the loop ends.

## Notes on the Cartesian bridges

- Poses and twists are in the base frame, like `O_T_EE` and `O_dP_EE` in libfranka: the `frame_id` of the messages is not checked. The pose is that of the EE set with `flange_T_ee_*`.
- The sign of the quaternion does not matter: the bridge makes it continuous from one sample to the next.
- The commanded EE never goes below `critical_height` + 1 cm. A stream that goes lower is saturated at that height (with `clamp` false it stops the robot), like the joints at their limits. The check on the measured EE remains as a last line of defense.
- Elbow (`use_elbow_control`): if it is false, the robot chooses the elbow and `desired_elbow` is ignored. If it is true, the bridge also commands the position of joint 3: during streams it keeps it still (the messages have no field for the elbow), goals move it with `desired_elbow`. In this case always set `desired_elbow`, `.nan` to keep the elbow: the message default is 0, which is a valid position.
- The default Cartesian limits are low (`safe_acceleration_factor` and `safe_jerk_factor` 0.3): with high EE accelerations it is the joints that exceed their limits, and the bridge does not see it. In any case, near a singularity even a slow EE motion can trigger a joint reflex.

## Notes on the torque bridge

- The torques on the topic are **without gravity**, as in libfranka: the robot compensates gravity. A controller that includes gravity (for example one written for simulation) must remove it, otherwise it is applied twice.
- When it is not following a stream or a motion, the bridge holds the position with a joint impedance: `joint_stiffness`, `joint_damping` and the Coriolis torque of the model. The default gains are those of the libfranka `joint_impedance_control` example.
- In external mode the bridge adds nothing: the torque on the topic goes to the robot, within the torque and derivative limits.
- An external controller sees the state a few milliseconds late: with high gains, especially on the last joints, the loop becomes unstable. For a stiff control law use internal mode.
- The `gravity_compensation` preset leaves the robot free to be moved by hand, until `/panda/stop_motion`. The default collision thresholds (20 Nm) can trigger a reflex if you push it hard.

## Signals and shutdown

External causes never cut the bridge off abruptly:

| Event | What the bridge does |
|---|---|
| Ctrl+C (SIGINT), SIGTERM | smooth stop, then exits |
| terminal closed (SIGHUP) | like Ctrl+C |
| the GUI that launched it dies | it keeps commanding the robot: its log goes nowhere (SIGPIPE ignored). The next GUI reports it on Start |

The bridge is a process separate from the GUI, in the same process group: Ctrl+C in the launch terminal reaches both. Real time does not depend on this: the libfranka loop runs in a SCHED_FIFO 99 thread, which takes precedence over everything else. To isolate it further you need `rt_cpu` (a dedicated CPU), `lock_memory` and the NIC interrupts on that CPU.

## Where the bridges are in the code

There is no source file per bridge: the bridges are two families of shared code, instantiated for each command type.

| Part | File | Shared or specific |
|---|---|---|
| `main()` of the executable | `src/panda_{joint,cartesian}_*_bridge.cpp` | one per bridge, `main()` only |
| topics, action, presets, status | `bridge/joint_bridge.hpp/.cpp`, `bridge/cartesian_bridge.hpp/.cpp` | one per family; what changes between the bridges of a family is in the `...InterfaceTraits` at the top of the header |
| ROS parts shared by the two families | `bridge/bridge_common.hpp` | goals, events, stream statistics, status |
| state machine | `bridge/pipeline.hpp/.cpp` | shared by all |
| how a command becomes acceptable | `bridge/joint_tracker.hpp`, `joint_velocity_tracker.hpp`, `joint_torque_tracker.hpp`, `cartesian_tracker.hpp` | one per bridge (the two Cartesian ones in the same file) |
| connection, threads, published state | `bridge/bridge_base.hpp/.cpp` | shared by all |

## Files

### `uclv_panda_interfaces`

| File | Content |
|---|---|
| `msg/JointsCommandedPositions.msg` | joint position command |
| `msg/PandaRobotState.msg` | full robot state |
| `msg/BridgeStatus.msg` | bridge state and diagnostics |
| `srv/RunPreset.srv` | starts a preset (name, duration, amplitude) |
| `action/JointTraj.action` | joint goal: positions and duration |
| `action/CartTraj.action` | Cartesian goal: pose, duration, elbow (NaN keeps it) |

### `panda_utils`: bridge code

The headers are in `include/panda_utils/bridge/`, the sources in `src/bridge/`.

| File | Content |
|---|---|
| `types.hpp` | constants (7 joints, 1 ms cycle), `JointArray`, `JointFeedback` (robot state used by the joint trackers), clock reading |
| `franka_limits.hpp` | FER limits: joint, Cartesian and torque, from the libfranka values (`rate_limiting.h`) and from `robot_limits.hpp` |
| `cartesian_types.hpp` | poses, quaternions, conversions to and from `O_T_EE`, `CartesianState` and `CartesianFeedback` |
| `robot_backend.hpp/.cpp` | interface to `franka::Robot` for the five command types; `FakeRobotBackend` is the simulated robot of the tests |
| `rt_exchange.hpp` | lock-free queues and boxes to pass data between the loop and ROS |
| `state_conversions.hpp/.cpp` | from `franka::RobotState` to the messages; tables of the initial state printed on Configure |
| `bridge_base.hpp/.cpp` | lifecycle node shared by all bridges: connection, threads, publishers, recovery |
| `bridge_main.hpp` | shared `main()` of the executables: node and executor |
| `bridge_common.hpp` | ROS parts shared by the bridges: loop snapshot, events and log, stream statistics, status, action goals |
| `jitter_buffer.hpp` | samples sorted by stamp: local quadratic fit (positions, poses, velocities) or linear interpolation (torques) |
| `playback_clock.hpp` | playback time: robot clock aligned to the PC clock |
| `pipeline_types.hpp` | state machine types: configuration, counters, motion state, requests |
| `pipeline.hpp/.cpp` | state machine shared by all bridges, with neither ROS nor libfranka; one instance per tracker |
| `joint_tracker.hpp` | joint position tracker |
| `joint_velocity_tracker.hpp` | joint velocity tracker |
| `joint_torque_tracker.hpp` | torque tracker: limits, derivative, impedance, damping, hold |
| `cartesian_tracker.hpp` | pose and twist tracker, with minimum height and elbow |
| `joint_motions.hpp` | internal joint motions: quintic, "cosine bump", free (gravity compensation) |
| `cartesian_motions.hpp` | internal Cartesian motions: quintic (line and slerp), back and forth along an axis, circle |
| `joint_bridge.hpp/.cpp` | the joint bridges: topics, action, presets, status; one instance each for positions, velocities and torques |
| `cartesian_bridge.hpp/.cpp` | the Cartesian bridges: the same, one instance each for pose and velocity |

### `panda_utils`: other files

| File | Content |
|---|---|
| `src/panda_*_bridge.cpp` | the five executables, `main()` only |
| `src/simple_joint_traj.cpp` | reference client for joint positions (executable `simple_joints_traj`) |
| `src/simple_velocity_joint_traj.cpp` | reference client for joint velocities (executable `simple_velocity_traj`) |
| `src/simple_torque_cmd.cpp` | reference client for torques: zero torques at 1 kHz |
| `src/simple_cartesian_traj.cpp` | reference client for the Cartesian bridges: quintic to a pose (or by a displacement), as poses or as twists |
| `examples/joint_position_client.py` | minimal Python client example, at 100 Hz |
| `include/panda_utils/robot_limits.hpp` | Panda limits, without ROS dependencies |
| `include/panda_utils/constants.hpp`, `utils_func.hpp` | names, QoS and shared geometric functions |
| `gui/panda_gui_py.py` | the GUI (Tkinter + rclpy) |
| `launch/gui.launch.py` | launches the GUI |
| `src/panda_gui.cpp` | old Qt GUI, no longer used |

### Tests (`panda_utils/test/`)

| File | What it checks |
|---|---|
| `kinematics_checks.hpp` | helper: checks joint and Cartesian commands as the robot does (finite differences, norms) |
| `test_jitter_buffer.cpp` | ordering, duplicates, fit, extrapolation |
| `test_playback_clock.cpp` | clock jitter, drift and jumps |
| `test_joint_tracker.cpp` | positions: feasible references reproduced exactly, steps and braking within the limits, joint limits |
| `test_joint_velocity_tracker.cpp` | the same for velocities, plus position integration |
| `test_cartesian_tracker.cpp` | quaternions and conversions; poses and twists reproduced exactly, steps and braking within the limits, minimum height, elbow |
| `test_joint_position_pipeline.cpp` | streams with jitter, losses and duplicates, staircase streams, stalls, timeout, internal motions, cancel |
| `test_joint_velocity_pipeline.cpp` | velocity streams (including one that ends exactly at the end of a quintic, and a staircase one), stalls, internal motions that reach the position |
| `test_joint_torque_pipeline.cpp` | with a simulated robot: hold, streams with limits and derivative, damping, impedance, gravity compensation |
| `test_cartesian_pipeline.cpp` | pose streams (including with the quaternion sign flipped, and staircase ones) and twist streams, stalls, jumps, minimum height, goals, circle |
| `test_bridges.cpp` | the five complete bridges on the simulated robot: activate/deactivate cycles, streams, goals and presets |

The tests run on an isolated ROS domain (215-232, localhost only), so they never reach a bridge connected to the robot.

The style linters (uncrustify, flake8, pep257) are excluded in `CMakeLists.txt`: the code does not follow the ROS formatting style, and they would fail on every file, hiding the real problems. `lint_cmake`, `xmllint` and `cppcheck` stay active.
