# media

Pictures of K.A.R.R. for the README, all generated from the robot model, so they can be remade whenever it changes.

| File | What | Made by |
|---|---|---|
| `karr_action.gif` | ~18 s looping action sequence at a cartoon warehouse loading dock | `render_karr.py action` + `make_gifs.sh` |
| `karr_turntable.gif` | 6 s studio turntable | `render_karr.py turntable` + `make_gifs.sh` |
| `karr.stl` | The whole robot as one STL (GitHub shows it in an interactive 3D viewer) | `urdf_prep/tools/make_preview_stl.py` |

## Rendering

Needs [Blender](https://www.blender.org/) 4.2 or newer (developed on 5.3; macOS: `brew install --cask blender`) and
ffmpeg. Renders use Cycles on the GPU when there is one (Metal, OptiX/CUDA, HIP, oneAPI), otherwise the CPU.

```bash
blender -b -P media/render_karr.py -- turntable          # frames -> media/frames/turntable/
blender -b -P media/render_karr.py -- action             # frames -> media/frames/action/
bash media/make_gifs.sh                                  # -> media/karr_turntable.gif, media/karr_action.gif
```

Quick checks before a full render:

```bash
blender -b -P media/render_karr.py -- action --still 0,100,200 --samples 16 --res 480x270   # a few PNGs
blender    -P media/render_karr.py -- action --no-render                                     # open it in Blender
```

`media/frames/` is git-ignored. The first render after installing Blender spends a few minutes compiling GPU kernels.

## How the action sequence is made

Nothing is keyframed by hand. In `render_karr.py`:

- **Robot:** built from `karr_description/urdf/karr.urdf` and its meshes, one Blender empty per URDF link/joint, so
  steering and wheel joints turn exactly like in rviz.
- **Story:** K.A.R.R. bursts out of dock door A, launches off the 1.2 m loading dock (small kicker lip, 360° spin,
  slow motion), slaloms three cones, drifts round the barrels, stops for a hero swing turn, drives up the dock ramp
  and back in through door B. Inside the building, out of sight, the path turns round to door A: a closed loop.
- **Track:** a closed spline (`TRACK`) with a height profile (dock, kicker, air, yard, ramp). Speed comes from the
  corner radius plus braking/acceleration into and out of the hero stop.
- **Set:** simple shapes and flat colours (warehouse, dock, ramp, cones, barrels, fence, trees, hills) at real sizes:
  cones 0.4 m (16 in), barrels 0.9 m (3 ft), dock 1.2 m (48 in).
- **Wheels:** steering angles and wheel speeds come from `karr_control`'s kinematics (front-wheel Ackermann), and the
  hero stop does the real swing turn about the rear axle centre. The jump, the 360° spin in the air, the drifts
  and the suspension bounce are artistic licence.
- **Slow motion:** story time runs at 45 % while airborne.
- **Camera:** a list of shots (angle relative to the direction of travel, distance, height, lens), blended smoothly
  around the loop and aimed at a smoothed version of the robot's position. While the robot is inside, the camera
  blends to a fixed view of both doors.
- **Perfect loop:** the path, speed profile, camera and shots are all periodic, so the last frame runs straight into
  the first.

To change the action, edit `TRACK`, `CONES`, `BARRELS`, the dock/ramp constants, the speeds, or the `shots` list.
