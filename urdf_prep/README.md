# urdf_prep

Turns the SolidWorks URDF export into the `karr_description` package. This is the Windows / CAD side; colcon ignores
this folder.

```
karr_description_ros1/   Raw sw_urdf_exporter output (ROS 1). Re-export here, untouched.
tools/                   Converter and check scripts
NOTES.md                 Decisions made during conversion, and open items
```

## Updating the robot model

1. In SolidWorks, re-export into `urdf_prep/karr_description_ros1/` (same package name, overwrite).
2. Convert. Pure Python stdlib, runs anywhere, including Windows without Docker:
   ```
   python urdf_prep/tools/convert_sw_export.py
   ```
3. Check (in the container, needs ROS): `./dev.sh verify` or `.\dev.ps1 verify`. Every inertia line must say `ok`. Look
   at `urdf_prep/karr_views.png`, then `./dev.sh display`.
4. Commit the export and the regenerated package together.

Every hand fix, mass, joint limit, color and sensor frame lives in the CONFIG section of
[tools/convert_sw_export.py](tools/convert_sw_export.py). **Change values there, not in `karr.urdf`.** The converter
overwrites `karr_description/urdf/karr.urdf`, `config/footprint.yaml` and `meshes/` on every run. Launch, rviz and
package files are left alone.

What the converter does to the raw export:
- renames the robot to `karr` and the package to `karr_description`, and rewrites mesh paths
- adds the `base_footprint` root (REP-105)
- rotates the wheel frames so the axle is +Y, and flips the steering axis to +Z (REP-103)
- sets masses and limits from CONFIG; scales the SolidWorks inertia tensors to those masses
- fixes the mirrored-GPS center-of-mass offset (+0.3048 m)
- collision: one body box from the footprint envelope, raised to the chassis underside; cylinder wheels; nothing else
- named materials (LTU Blue #005EB8 at 50 % for the body)
- renames the ZED links to `<name>_camera_link`; adds `livox_frame` and `livox_imu_link`
- writes the Nav2 footprint polygon to `config/footprint.yaml`

## Tools

| Script | Purpose |
|---|---|
| `convert_sw_export.py` | Export → ROS 2 package (the source of truth for all fixes) |
| `verify.sh` | Convert + build + every check below (container) |
| `inertia_report.py` | Per-link mass, inertia validity (Gazebo rejects invalid tensors), total mass |
| `frame_report.py` | World pose and axis directions of chosen frames |
| `render_urdf.py` | Side, front and top views of the visual meshes to a PNG |
| `mesh_bbox.py` | Bounding box and triangle count of every STL in a folder |
