# karr_gazebo

Gazebo Harmonic simulation of K.A.R.R.

```bash
ros2 launch karr_gazebo sim.launch.py                                    # everything
ros2 launch karr_gazebo sim.launch.py steering_mode:=four_wheel gps_mode:=single cameras:=false
```

`sim.launch.py` includes the pieces below; each also runs on its own, so other launch files can reuse them.

| Launch file | What | Arguments |
|---|---|---|
| `sim.launch.py` | All of the below + rviz + teleop | all below, plus `rviz`, `teleop`, `steering_mode` |
| `world.launch.py` | Gazebo with a world | `world` (name in `worlds/` or path), `gui`, `verbosity` |
| `spawn.launch.py` | robot_state_publisher (sim xacro), spawn, bridges | `gps_mode`, `cameras`, `lidar`, `name`, `x`, `y`, `z`, `yaw` |
| `karr_control/control.launch.py` | Controllers + `karr_drive` | `steering_mode`, `use_sim_time` |

## How the robot gets into Gazebo

`urdf/karr_sim.urdf.xacro` includes the generated `karr_description/urdf/karr.urdf` unchanged and adds:
- the `<ros2_control>` block from `karr_control` with the `gz_ros2_control/GazeboSimSystem` plugin,
- wheel friction,
- the sensors from `urdf/sensors.xacro`, switched by the xacro args `gps_mode`, `cameras`, `lidar`.

The real robot never loads this file.

## Sensors

Gazebo's own sensors stand in for the real ones, and `ros_gz_bridge` (`config/bridge_*.yaml`) brings them into ROS under
**the same topic names and frame_ids the real drivers use**. Code written against the sim should run on the robot
unchanged. **No vendor packages are needed for the sim** (no Livox SDK, ZED SDK/CUDA or u-blox driver); those are only
installed on the robot.

| Sensor | Real driver | Sim topic(s) | frame_id | Sim model |
|---|---|---|---|---|
| Livox Mid-360 | `livox_ros_driver2` (`xfer_format: 0`) | `/livox/lidar` (PointCloud2), `/livox/imu` | `livox_frame` | `gpu_lidar` 720 × 32 rays, −7…+52° (mounted upside down), 0.1–40 m, 10 Hz; IMU 200 Hz |
| 3 × ZED X One S (wide) | `zed-ros2-wrapper` | `/<cam>/rgb/image_rect_color`, `/<cam>/rgb/camera_info` | `<cam>_camera_frame_optical` | 110° HFOV, 960 × 600, 10 Hz. `<cam>` = `left_zed`, `center_zed`, `right_zed` |
| u-blox ZED-F9P × 2 | u-blox driver | `/gps/left/fix`, `/gps/right/fix` (NavSatFix) | `left_gps_link`, `right_gps_link` | `navsat`, 10 Hz, 20 cm horizontal / 40 cm vertical noise (spotty RTK) |

- **Lidar:** the real Mid-360 has a non-repetitive scan pattern; the sim uses a regular grid with the same field of view.
  Mounted upside down, it sees part of the robot's own body, as the real one will. Filter those points downstream.
- **Cameras:** the zed wrapper publishes `<cam>_camera_frame_optical` on the robot; the sim xacro adds that frame itself.
  Topic names follow the 4.x wrapper; check them against the wrapper version used on the robot.
- **GNSS:** `gps_mode:=dual` (default) simulates both antennas, `single` the left one only. Dual-antenna heading (moving
  base / RELPOSNED) is not simulated yet; the two fixes are independent. The world origin's latitude/longitude is set in `worlds/basic.sdf`.

## Worlds

| World | Contents |
|---|---|
| `basic.sdf` | Flat ground, 3 in curb 4 m ahead, 15° ramp to the left, boxes and barrels |

## Physics, and why the wheels slipped

Gazebo moves the robot with the **collision shapes** from the URDF (a box for the body, a cylinder per wheel), not the
visual meshes. See "Which geometry is used where" in [karr_description](../karr_description/README.md).

Each simulation step (1 ms):
1. ros2_control sets each wheel joint's target speed (from `karr_drive`).
2. The **collision detector** finds where each wheel cylinder touches the ground and how far it sinks in.
3. The **physics solver** (DART) pushes back on those contacts, and **friction** (μ = 1 for rubber on ground) limits how
   much sideways and forward force each contact can carry. That friction force is what moves the robot.

**What went wrong:** with DART's default collision detector, a cylinder lying on a plane often gets only one contact
point that flickers from step to step. The wheels spun at the commanded speed but kept losing grip: driving 1 m by
odometry moved the robot about 0.4 m. **The fix** was switching to the Bullet collision detector, which keeps steady
contacts on cylinders:

```xml
<physics name="1ms" type="dart">
  <dart><collision_detector>bullet</collision_detector></dart>
</physics>
```

Now odometry and ground truth agree within ~10 % (the rest is real scrub while steering). **Every new world needs this
`<dart>` block.**

How to spot slip yourself: compare `/odom` (what the wheels think happened) with the GPS fix or Gazebo's model pose
(what really happened). If odometry runs far ahead, the wheels are slipping. Wheel friction (μ = 1) and sensor noise
values are first guesses.

## Performance

Without a GPU (default in the dev container), the three cameras are the slowest part. `cameras:=false` makes the sim
helps a lot. Watch the real-time factor at the bottom of the Gazebo window. Measured on an M-series MacBook (arm64, browser
GUI): ~25 % with everything on, ~30 % headless without cameras. A Linux laptop with `LIBGL_ALWAYS_SOFTWARE=0` and a
real GPU should do much better.
