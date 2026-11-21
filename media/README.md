# media

Pictures of K.A.R.R. for the README, all generated from the robot model, so they can be remade whenever it changes.
Look, palette and camera rules: [docs/design/README.md](../docs/design/README.md).

| File | What | Made by |
|---|---|---|
| `karr_mission.gif` | ~21 s looping mission on a small island world (README hero, 480 px, 12 fps) | `render_karr.py mission` + `make_gifs.sh` |
| `karr_mission.mp4` | The same loop in HD (1280×720, 24 fps), linked under the GIF | `make_gifs.sh` |
| `karr_turntable.gif` | 6 s studio turntable | `render_karr.py turntable` + `make_gifs.sh` |
| `karr.stl` | The whole robot as one STL (GitHub shows it in an interactive 3D viewer) | `urdf_prep/tools/make_preview_stl.py` |

| Script | Role |
|---|---|
| `render_karr.py` | Command line, robot (from the URDF), turntable scene, helpers |
| `karr_world.py` | Set pieces: rounded matte buildings, trucks, forklift, drone, island, sky and the soft cinematic grade |
| `karr_mission.py` | The mission: robot path and timing, world animation, shot list |
| `make_gifs.sh` | Frames → size-tuned GIFs and the HD MP4 |

## Rendering

Needs [Blender](https://www.blender.org/) 5.x and ffmpeg. Cycles uses Metal on macOS and CUDA on Linux/WSL
(`KARR_DEVICE=CUDA` forces it; OptiX does not work in WSL).

```bash
blender -b -P media/render_karr.py -- turntable          # frames -> media/frames/turntable/
blender -b -P media/render_karr.py -- mission            # frames -> media/frames/mission/
bash media/make_gifs.sh                                  # -> karr_turntable.gif, karr_mission.gif, karr_mission.mp4
```

Quick checks before a full render:

```bash
blender -b -P media/render_karr.py -- mission --no-render                                 # timing + shot list only
blender -b -P media/render_karr.py -- mission --still 15,165,389 --samples 12 --res 480x270  # a few PNGs
blender    -P media/render_karr.py -- mission --no-render                                 # open it in Blender
```

The full mission render runs on the lab GPU PC (skill `lab-5090-render`): push `media/`, `karr_control/karr_control`
and `karr_description/{urdf,meshes}`, run the command above with `KARR_DEVICE=CUDA`, fetch `media/frames/mission`.
Known issue: WSL on the lab PC shuts down ~30 s after the last SSH call unless the `WSLHold` task is running, and it
does not always restart for back-to-back jobs. Start the job, then `Start-ScheduledTask WSLHold`; it holds WSL until the
job's tmux session ends. 1280×720 at 24 samples takes ~35 min for the mission on the RTX 5090. `media/frames/` is
git-ignored.

**GIF vs video.** GitHub renders repo GIFs inline and autoplays them (unless the viewer turned off animated-image
autoplay in GitHub's accessibility settings). Videos only embed when uploaded through GitHub's web editor, and they
don't autoplay. So the GIF is the hero and the MP4 is a link.

## How the mission is made

Nothing is keyframed by hand. In `karr_mission.py`:

- **World:** a floating island with a ring road, a parking lot with a drone pad, and a warehouse with an open loading
  platform at dock height (1.2 m). Two trucks are backed onto the platform; a ramp at the south end lets the robot up.
- **Story:** the drone lowers box A (small, orange EXPRESS sticker) onto K.A.R.R. → slalom → up the ramp → into the
  dark warehouse (boxes swap, hidden) → out with box B (bigger, dark HEAVY 40 kg sticker) → the forklift waiting half
  out of truck 1 lifts box B off the deck and reverses in → K.A.R.R. hops off the dock between the trucks (slow
  motion) → drift past the barrels → back to the pad as the next drone arrives.
- **Robot motion:** a closed spline (`TRACK`) with a speed profile (lot / ramp / platform speed zones, corner limits,
  braking into the two holds: pad and handoff). Steering angles and wheel speeds come from `karr_control`'s
  kinematics (front-wheel Ackermann).
- **Busy world:** three trucks lap the ring road, a second forklift unloads bay 2, dock lamps and forklift beacons
  blink. Everything is periodic in the loop length, so the last frame runs into the first.
- **Edit:** four continuous shots, three cuts. (A) overhead drone drop, swoop down beside the pad, robot pulls away
  past the lens | (B) low follow through the slalom, slight tilt | (C) locked-off wide of the ramp climb and the door,
  fly across to door B while the robot is hidden, glide with it to the forklift handoff | (D) head-on hop between the
  trucks, swing alongside, rise to an aerial chase past the barrels, settle on the opening overhead. Poses blend
  with easing (`blend()` in `karr_mission.py`); cameras are keyed on sub-frames so motion blur never smears a cut.
