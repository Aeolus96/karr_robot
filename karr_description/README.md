# karr_description

ROS 2 (Jazzy) robot description for **K.A.R.R.** (Keystone Autonomous Resource Rover), a mobile robot with independently steered and driven wheels, driven with Ackermann geometry (front-wheel or double
Ackermann; see karr_control).

```bash
ros2 launch karr_description display.launch.py        # rviz2 + joint sliders
ros2 launch karr_description description.launch.py    # robot_state_publisher only (for sim / bringup)
```

> `urdf/karr.urdf`, `config/footprint.yaml` and `meshes/` are **generated** from the SolidWorks export by
> `urdf_prep/tools/convert_sw_export.py` at the repository root. Don't edit them by hand.

## Frames

Conventions follow [REP-103](https://www.ros.org/reps/rep-0103.html): X forward, Y left, Z up, positive rotation counter-clockwise. Frame names follow [REP-105](https://www.ros.org/reps/rep-0105.html).

| Frame | Parent | Notes |
|---|---|---|
| `base_footprint` | (root) | Ground projection of the robot centre |
| `base_link` | `base_footprint` | Zero offset: the chassis origin is already at ground level |
| `{fl,fr,bl,br}_steering_link` | `base_link` | Steering modules at (±0.3556, ±0.2286, 0.3345) m |
| `{fl,fr,bl,br}_wheel_link` | `*_steering_link` | Wheel centre, 0.212 m below the steering pivot; axle along +Y |
| `left_gps_link`, `right_gps_link` | `base_link` | GNSS antenna mounting base; 0.6096 m baseline along Y |
| `livox_base_link` | `base_link` | Livox Mid-360 mounting face; lidar mounted **upside down**, connector toward the rear |
| `livox_frame` | `livox_base_link` | Point-cloud origin, 47 mm from the mounting face. Default `frame_id` of `livox_ros_driver2` |
| `livox_imu_link` | `livox_frame` | Built-in IMU (11.0, 23.29, −44.12) mm |
| `{left,center,right}_zed_camera_link` | `base_link` | ZED X One S mount point (centre of the 4 bottom holes), X = optical axis |

### ZED cameras

The [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) publishes each camera's internal frames
(`<name>_camera_center`, `<name>_camera_frame`, `<name>_camera_frame_optical`) below `<name>_camera_link`. Launch each camera
with a name that matches this URDF:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxones camera_name:=center_zed
```

The wrapper publishes its own camera model on a separate `robot_description` topic in the camera's namespace. It doesn't
interfere with this one. With more than one camera, or with robot_localization, set `pos_tracking.publish_tf:=false` (and
`pos_tracking.publish_map_tf:=false`) on every camera that should not own the `odom → base_link` transform.

## Joints

| Joint | Type | Axis | Limits |
|---|---|---|---|
| `*_steering` | revolute | +Z (positive = turn left) | ±165° (330° hard stop so the wiring can't wind up; 0 = straight ahead), 17 Nm (peak; 6 Nm continuous), 1.5 rad/s |
| `*_wheel` | continuous | +Y (positive = drive forward) | 30 Nm (estimate), 21.9 rad/s (= 6 mph at r = 0.1225 m) |

The joint list is also in `config/joint_names.yaml`.

## Physical properties

Masses are **placeholders** until the robot is built and weighed (total 100 kg): body 66.8 kg, steering modules and wheels
4 kg each, GNSS antennas 0.3 kg, Livox 0.265 kg, cameras 0.1 kg. The inertia tensors are the SolidWorks ones scaled to these
masses; the cameras use a solid-box approximation.

## Collision

- Body: one box, 0.991 × 0.787 m (the robot's footprint), from the chassis underside (0.132 m, which clears 3 in obstacles) to
  1.067 m (above the GNSS antennas).
- Wheels: cylinders, r = 0.1225 m, w = 0.0762 m.
- Steering modules and sensors have no collision geometry (they're inside the body box).

`config/footprint.yaml` holds the same outline as a rounded-rectangle polygon (84 mm corners) for Nav2 costmaps.

## Which geometry is used where

A robot model carries several *different* descriptions of its shape and mass. Each tool uses only some of them, and
mixing them up is a common source of confusion ("why does it slip / collide / plan like that?").

| What | Where it lives | Used by | Not used by |
|---|---|---|---|
| **Visual meshes** (detailed STL) | `<visual>` in `karr.urdf`, `meshes/` | rviz; Gazebo's rendering, so also what the **simulated cameras and lidar see** | Physics. Nothing pushes against a visual mesh. |
| **Collision shapes** (one box + 4 cylinders) | `<collision>` in `karr.urdf` | Gazebo **physics**: contacts with the ground and obstacles, and the **friction that drives the robot** | Rendering, sensors, Nav2 |
| **Inertia** (mass, centre of mass, inertia tensor) | `<inertial>` in `karr.urdf` | Gazebo physics: how the robot accelerates, tips, and loads each wheel | Everything else (robot_state_publisher ignores it) |
| **Joints and frames** | `<joint>` in `karr.urdf` | robot_state_publisher (TF), Gazebo, `karr_drive` (module positions in `drive.yaml` must match) | |
| **Footprint polygon** | `config/footprint.yaml` | Nav2 costmaps: path planning and obstacle avoidance | Gazebo, rviz robot model |
| **Friction, contact settings** | `karr_gazebo` (xacro `<gazebo>` tags, world `<physics>`) | Gazebo only | The real robot |

So in simulation the wheels are **cylinders**, not the tyre meshes. The robot moves because Gazebo spins each wheel
joint, finds where the cylinder touches the ground, and applies friction at that contact. If the contact is computed
badly, the wheel slips even though it is spinning at the right speed (see karr_gazebo's README for the slip we hit
and how it was fixed). Simple collision shapes are deliberate: they are fast and stable, while contacts on detailed
meshes are slow and jittery.

On the real robot none of the collision, inertia or friction data is used; the footprint (Nav2) and frames (TF) are.
