# K.A.R.R. design: 3D renders and README media

Two standards, one palette:

| Medium | Standard | Where |
|---|---|---|
| Docs, diagrams, slides, HTML pages (2D) | **Field Manual**: IBM Plex, slate + one orange accent, doc numbers, signal-path diagrams | [`docs/visual-style/notes.md`](../visual-style/notes.md), tokens in `tokens.css` / `tokens.json` |
| 3D renders, README GIFs/videos | **Soft cinematic toy world** (this file) | `media/` (code), this file (look + decisions) |

The renders use the Field Manual colour tokens for everything that is "the robot or the brand" (slate body, orange
accents, ink), so a render next to a diagram reads as one project. Render-only colours (sky, haze, grass, cardboard,
trucks) live in `media/karr_world.py` and do not belong in 2D work.

## Palette in renders

| Role | Hex | Token | Used on |
|---|---|---|---|
| Robot body | `#5A6675` | `slate` | `base_link` mesh |
| Accent | `#E8590C` | `orange` | Steering modules, plate stripe, EXPRESS sticker, drone motors, pad ring, one ring-road truck |
| Ink | `#1B2430` | `ink` | Plate text, HEAVY sticker |
| Warm white | `#F2EFE6` | render-only | Plates, sticker text, pad, light props |
| Sky / haze | `#9CC6E6` → `#F6E3C8` | render-only | Gradient sky, mist pass |

Orange is used sparingly, as in the Field Manual: the robot's modules are the orange thing in frame. Docked trucks
and big props are not orange (an orange truck next to orange barrels blended into one mass).

## Materials

- Everything matte: roughness 0.5–0.9, metallic 0, no clear coat. The robot body gets a little subsurface and sheen
  so it reads as soft plastic. The original glossy, metallic robot looked out of place in the cartoon world.
- Rounded edges on every set piece (bevel modifier, 3–4 segments) and simple, slightly chunky shapes.
- Ground surfaces have texture: asphalt, concrete and the ramp use noise colour variation plus a voronoi bump.
- Robot details: off-white `KARR` plates with an orange stripe, front and rear.

## Light and grade

- Warm low sun (about 30°), soft shadows (sun angle ~7°), gradient sky.
- AgX, *Medium Low Contrast* look; distance haze from the mist pass; light bloom (Blender 5 compositor).
- Shallow depth of field on close shots (f/1.8–2.8), deeper on wides (f/5.6).

## Camera language

From the reference the user picked (JesseDriftwood, "Beni, the 4k camera robot", first minute):

- Cameras low, near the ground, wide lenses; occasional slight dutch tilt on follow shots.
- Prefer **camera moves over cuts**: swoops, glides, a fly-over while the robot is hidden, a rise to an aerial view.
  Keep hard cuts to a few, each on a motion beat.
- Locked-off wides are fine when the action crosses the frame (the ramp climb).
- Slow motion for the jump. Loops must be seamless: the last frame runs into the first.

## Decisions log (README media)

| Date | Decision | Why |
|---|---|---|
| 2026-11-19 | README hero is an animated render, not the inline STL; the full model is linked as `media/karr.stl` | GitHub's inline STL had to be decimated to ~3k triangles to fit the README size limit and looked rough |
| 2026-11-19 | Props at real scale (0.4 m cones, 0.9 m drums, 1.2 m dock) and a cartoon, not photoreal, world | Real-scale props make the robot read at its true size; cartoon keeps renders cheap and friendly |
| 2026-11-20 | Look "B": soft cinematic, slate body, orange modules, KARR plates; no university branding, never "swerve" | Picked from three style frames (clay / toy cinema / diorama) |
| 2026-11-20 | Small island world with a busy loading dock and a delivery mission (drone → warehouse → forklift) | The goal is to show the robot fitting into a busy industrial environment |
| 2026-11-20 | Trucks back onto an open loading platform; the robot uses a ramp and its own doors | That is how trucks dock at warehouses |
| 2026-11-20 | Edit v2: ~21 s, four continuous shots with three cuts, no hero swing turn, forklift waits half out of its truck, EXPRESS vs HEAVY boxes | Shorter (GIF size), fewer choppy cuts, the two boxes must read as different |
| 2026-11-20 | GIF stays as the README hero; an HD MP4 is committed and linked | GitHub only embeds videos uploaded through its web UI (and they don't autoplay); repo GIFs autoplay unless the viewer turned off animated-image autoplay in GitHub's accessibility settings |

Rejected along the way: glossy metallic robot; flat yellow cartoon warehouse with a big sign; diorama with ink
outlines and an orange robot (liked the island idea, not the look); parking-lot set with blurry stall lines.
