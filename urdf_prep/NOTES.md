# Notes: K.A.R.R. description

Decisions made while converting the SolidWorks export (Nov 2026), and what is still open.

## Problems found in the SolidWorks export (all handled by `tools/convert_sw_export.py`)

| Problem | Fix |
|---|---|
| Wheel part frames have the axle along Z, so wheels showed lying flat | Wheel joint `rpy="-1.5708 0 0"` → axle along +Y |
| Steering axis exported as −Z (positive = turn right) | Axis +Z (REP-103: positive = turn left) |
| GPS was a mirrored body in one part; the exporter needs separate parts. Both copies keep a +0.3048 m Y offset in their center of mass | COM Y set to 0 (mesh is centred on the frame) |
| Root link `base_link` has inertia, which KDL / robot_state_publisher ignores | Massless `base_footprint` root added |
| Export colors are unreliable (one re-export reset the body to the SolidWorks default) | Colors hard-coded in the converter |
| `base_link` mass edited in SolidWorks (236.9 kg) without the inertia following (computed at 100 kg) | Mass and inertia set in the converter |
| One camera exported with mass and inertia = 0 (Gazebo rejects this) | Camera inertia from a solid box of the body |
| `footprint_link` exported as an 808 kg solid; missing from later exports | Envelope dimensions stored in the converter |
| Fine meshes were 25.8 MB (Livox alone 260k triangles) | Re-exported coarse: 8.4 MB |

## Choices

- **`base_link` at ground level.** The CAD origin is on the ground plane, so `base_footprint → base_link` is identity.
  Kept it that way rather than moving `base_link` up to the axle height.
- **Collision = simple shapes.** One box for the body, cylinders for the wheels. They're faster and more stable in Gazebo than
  meshes. The box bottom is at the chassis underside (0.132 m), so 3 in (0.076 m) obstacles pass between the wheels.
  Breakover angle with 0.711 m wheelbase ≈ 40°, so 10–20° ramps don't scrape.
- **Nav2 footprint** comes from the same envelope, as a rounded-rectangle polygon (`config/footprint.yaml`). Nav2 uses this, not
  the URDF collision.
- **ZED cameras.** The URDF stops at `<name>_camera_link` (mount point = centre of the 4 holes, same as Stereolabs'
  definition). The zed-ros2-wrapper adds `camera_center`, `camera_frame` and `camera_frame_optical`. Camera names in the URDF:
  `left_zed`, `center_zed`, `right_zed`.
- **Livox Mid-360.** `livox_frame` (point-cloud origin, 47 mm from the mounting face per the user manual) matches the driver's
  default `frame_id`, so clouds need no extra transform. The IMU is published with the same `frame_id` by the driver;
  `livox_imu_link` documents its true position.
- **Wheel velocity limit** = 6 mph software/safety limit (21.9 rad/s), not the motor's maximum.

## Open items

- [ ] **Weigh the robot and parts**, then update the `MASS_*` values in the converter. The current total of 100 kg is a placeholder.
- [ ] **Wheel torque**: 30 Nm is an estimate for the 350 W Segway hub motors at 46 V. The older 146 kg robot climbed 20° on the
      same motors at 36 V, which needs ≥15 Nm per wheel. Replace it with the motor controller's current limit × torque constant.
- [ ] **Steering speed**: 1.5 rad/s is the exporter default; set it from the steering motor/gearbox.
- [ ] **Livox orientation**: check with a real scan that +X (away from the connector) faces forward. The Mid-360S is assumed
      to have the same origin height as the Mid-360 (same 65 × 65 × 60 mm body).
- [ ] **Next stage**: Gazebo (`<gazebo>` tags, wheel friction, sensor plugins) and ros2_control (swerve controller).
      Will need converting the URDF to xacro or adding a xacro wrapper that includes `karr.urdf`.
- [ ] Mesh detail: the Livox is still the largest mesh (76k triangles). It's fine for rviz; it could be simplified further for Gazebo visuals.
