# karr_control

Drive control for K.A.R.R., shared by the simulation and (later) the real robot.

```bash
ros2 launch karr_control control.launch.py                 # controllers + karr_drive (needs a controller_manager)
ros2 launch karr_control control.launch.py steering_mode:=four_wheel use_sim_time:=true
ros2 launch karr_control teleop.launch.py xterm:=true      # keyboard teleop in its own window
```

```
                ┌──────────────────────── karr_drive ────────────────────────┐
cmd_vel ──────► │ limits.py ──► kinematics.py ──► steering alignment check   │──► /steering_controller/commands ──► Gazebo
                │ speed, accel,   Ackermann       (slow wheels while steering │──► /wheel_controller/commands        (robot: motors)
                │ turn vs speed   angles+speeds    motors catch up)          │──► /cmd_vel_limited (debug)
joint_states ─► │ odometry (least squares over all 4 wheels)                  │──► /odom, TF odom → base_footprint
                └─────────────────────────────────────────────────────────────┘
```

On the real robot the two command topics will go to a small adapter for the ODrive S1 (drive) and Robstride 02
(steering) CAN packages instead of ros2_control; `karr_drive` stays the same.

## Steering modes

Set with `steering_mode` (launch argument or parameter file). Change it at standstill (refused while moving):
`ros2 param set /karr_drive steering_mode four_wheel`.

| Mode | Steering | Turns about |
|---|---|---|
| `front` (default) | Front wheels (Ackermann), rear locked straight | A point beside the robot, on the rear axle line |
| `four_wheel` | All four, double Ackermann: rear steers opposite to front | A point beside the robot centre: about half the turning radius of `front` |

Only `linear.x` and `angular.z` of `cmd_vel` are used. There is deliberately no sideways (crab) mode: switching into it
means stopping, and with this robot's size and inertia it's rarely worth it compared to planning a double-Ackermann
path.

In `front` mode the robot pivots about the rear axle, so `base_link` (the robot centre) slides slightly sideways in
turns (`wz × 0.356 m`). The odometry reports that honestly.

**This year's hardware has two steering motors (front only).** For the real robot, set `steered_modules: [fl, fr]` in
`drive.yaml` and list only `fl_steering, fr_steering` in `controllers.yaml` and in the `karr_ros2_control` xacro macro
(`steered_modules="fl fr"`). `four_wheel` then refuses to start. When the rear motors arrive, put them back in all three
places; nothing in the description or sim changes. The sim always has all four.

How the commands are computed is in [kinematics.py](karr_control/kinematics.py) (no ROS, unit tested in `test/`).

**Driving turns vs swing turns.** In `front` mode the turning centre sits on the rear axle line and can move in all
the way to the robot's centre line, which gives a *swing turn* about the middle of the rear axle (front wheels at ~72°,
rear wheels turning opposite ways), useful for tight parking. In `four_wheel` mode the same command turns in place.
The limiter keeps the two apart:

- **Driving** (`linear.x` ≥ 0.05 m/s): the turn radius never goes below `min_turn_radius` (1 m), however large
  `angular.z` is. A request like teleop's `u` key (0.5 m/s with 1 rad/s = 0.5 m radius) becomes a 1 m turn, and the
  steering eases in over about a second instead of jumping.
- **Swing turn** (`linear.x` below 0.05 m/s, `angular.z` non-zero, e.g. teleop `j`/`l`): only from standstill. If the
  robot is moving it first stops on its current path, then turns on the spot. Asking to drive again ends the swing
  first. `allow_swing_turn: false` disables it.

## Safety limits

`cmd_vel` is never used as-is. Every cycle (50 Hz) it passes through [limits.py](karr_control/limits.py), set in
`drive.yaml` under `limits:`. While driving, the turn is handled as a turning radius rather than a turn rate, so
speeding up or slowing down keeps the robot on the same arc.

| Limit | Default | Why |
|---|---|---|
| `max_speed` / `max_reverse_speed` | 2.68 / 1.0 m/s | 6 mph forward; slower backwards |
| `max_accel` / `max_decel` | 0.5 / 1.0 m/s² | No jerky starts; stops faster than it starts. Reversing stops first. |
| `max_lateral_accel` | 1.0 m/s² | **Turns widen as speed grows** (speed² / radius ≤ 1 m/s²). At 6 mph that's ≥ 7 m radius, ~6° of steering, so a hard turn request at speed can't throw the robot sideways. Tip-over is around 5 m/s². |
| `min_turn_radius` | 1.0 m | Tightest turn while driving; tighter needs a swing turn from standstill |
| `max_curvature_rate` | 1.0 1/m per s | Steering eases in: straight to the tightest turn takes 1 s |
| `max_angular_speed` | 1.0 rad/s | Turn rate cap (driving and swing turns) |
| `max_angular_accel` | 1.0 rad/s² | Swing turns start and stop gradually |
| `swing_speed_threshold`, `allow_swing_turn` | 0.05 m/s, true | See "Driving turns vs swing turns" above |
| `steering_alignment` | 0.1 / 0.5 rad | If a steering motor is more than 0.1 rad behind its target, the wheels slow down (stop at 0.5 rad) and the robot doesn't speed up until it catches up. Without steering feedback it doesn't drive at all. |
| `cmd_timeout` | 0.5 s | No `cmd_vel` → slow to a stop at `max_decel` |
| `mode_switch_max_speed` | 0.05 | `steering_mode` only changes at standstill |

`/cmd_vel_limited` shows what the robot is actually asked to do after all limits; compare it with `/cmd_vel` when the
robot seems slow or won't turn as tight as requested.

**These are not an emergency stop.** E-stop belongs to karr-safety-core (hardware + micro-ROS) and must cut the motors
independently of this node.

### For later

Things this package doesn't do yet, roughly in order of usefulness for the real robot:

- **State machine:** `idle → manual → autonomous`, plus `fault` and `estopped`, with clear rules for each transition
  (e.g. autonomous only after a heartbeat from the planner; any fault drops to idle).
- **Safety-core integration:** subscribe to its e-stop / heartbeat state; refuse to drive and zero commands while it says
  stop; report our own heartbeat back.
- **Command source priority:** `twist_mux` (or similar) so a teleop joystick always overrides Nav2, with its own timeout.
- **Motor feedback and faults:** CAN timeouts, ODrive/Robstride error codes, over-current and temperature; stop on loss of
  feedback from any motor.
- **Steering homing / calibration:** find the zero (straight ahead) on start-up and verify it against the ±165° stops.
- **Terrain-aware limits:** lower speed and lateral acceleration on slopes (Livox IMU pitch/roll) and on rough ground.
- **Obstacle-proximity speed scaling:** slow down near obstacles even when the planner doesn't (e.g. Nav2 collision monitor).
- **Slip detection:** compare wheel odometry with IMU yaw rate and GNSS; flag or limit when they disagree.
- **Battery-aware limits:** reduce acceleration at low voltage to avoid brown-outs.
- **Jerk limits** (smooth changes of acceleration) if the motion still feels abrupt on the real robot.

## Files

| File | What |
|---|---|
| `config/drive.yaml` | `karr_drive` parameters: steering mode, module geometry, wheel radius, safety limits, timeouts, frames |
| `config/controllers.yaml` | ros2_control: joint_state_broadcaster, steering (position) and wheel (velocity) group controllers |
| `urdf/karr_ros2_control.xacro` | `<ros2_control>` block; the robot xacro picks the hardware plugin (Gazebo or CAN) |
| `launch/control.launch.py` | Spawns the controllers and starts `karr_drive` |
| `launch/teleop.launch.py` | `teleop_twist_keyboard` on `/cmd_vel` |

## Topics

| Topic | Type | Dir |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (`TwistStamped` with `use_stamped: true`) | in |
| `/joint_states` | `sensor_msgs/JointState` | in |
| `/steering_controller/commands` | `std_msgs/Float64MultiArray`, rad, order = `steered_modules` | out |
| `/wheel_controller/commands` | `std_msgs/Float64MultiArray`, rad/s, order = `modules` | out |
| `/odom` | `nav_msgs/Odometry` (least-squares fit of all four wheels) | out |
| `/cmd_vel_limited` | `geometry_msgs/Twist`, the command after the safety limits | out |

Wheels keep their steering angle when stopped. A wheel that would
need more than ±90° steers the other way and spins backwards, so commands stay well inside the ±165° hard stop (330° of
travel, limited so the wiring can't wind up). If any wheel would exceed `max_wheel_speed`
(rad/s; in a turn the outer wheels spin faster than the robot's speed), all wheels slow down together so the path stays
the same.

## Teleop keys (teleop_twist_keyboard)

`i` forward, `,` back, `u`/`o` forward + turn (limited to a 1 m radius), `k` stop, `q`/`z` faster/slower. `j`/`l`
swing turn: in `front` mode about the middle of the rear axle (front wheels steer ~72°), in `four_wheel` about the robot
centre; the robot stops first if it was moving. `w`/`x` and `e`/`c` change only the speed or only the turn rate.
The Shift (holonomic) keys do nothing: sideways motion is ignored. The teleop window must have focus. teleop_twist_keyboard
only publishes on key presses, so hold the key down: 0.5 s after the last one the robot slows to a stop. Its default
speed is 0.5 m/s; `q` raises it (the limits above still apply).
