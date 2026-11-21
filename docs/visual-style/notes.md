# K.A.R.R. visual style: Field Manual

The standard for every 2D visual in the K.A.R.R. project: docs, diagrams, slides and HTML pages. It reads like a well-made component datasheet: neutral slate, one safety-orange accent, IBM Plex, and structure (doc numbers, numbered figures, footnotes) instead of decoration.

Agreed on 2026-11-20. An identical copy lives in `karr_robot/docs/visual-style/` and `karr-safety-core/docs/visual-style/`. Change both together.

| File | What |
|---|---|
| `notes.md` | This standard. Read it before making any visual. |
| `tokens.css` | Colors (light + dark), fonts, spacing, radii as CSS variables. Paste into any page. |
| `tokens.json` | The same tokens as data. |
| `signal-path.html` | Reference diagram: the chosen "Signal path" figure. |
| `build-up-deck.html` | Reference deck: the chosen build-up deck. |

3D renders and README GIFs (Blender materials, light, camera) are a separate topic: see `karr_robot/docs/design/README.md`.

## Rules in one screen

- **Type:** IBM Plex Sans for everything people read, IBM Plex Mono for data (topics, registers, doc numbers, units in tables). Google Fonts: `family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600;700`.
- **Casing:** sentence case for all headings and titles. No full-caps headings. Caps only for one short label with open space around it (e.g. above a slide title). Never put a caps line directly above dense text.
- **Color:** `sheet` on `paper`, `ink` text, `slate` for secondary text. `orange` is the only accent: one use per view. Semantic colors (`ok`, `stop`, `warn`) are separate from the accent and always come with a word.
- **Structure over decoration:** doc numbers (`KARR-SC-003 Rev C`), numbered figures, footnotes and spec tables. No gradients, no emoji, no drop shadows, small radii.
- **Content:** real names, units and paths (`/rc/joy`, `200 ms`, `0x00`). Never "Module A". Say K.A.R.R. (Keystone Autonomous Resource Rover). The drive is independent 4-wheel steering driven as Ackermann; never call it swerve. Don't mention the university.

## Tokens

| Token | Light | Dark | Use |
|---|---|---|---|
| `paper` | `#f7f8fa` | `#14171c` | Page background |
| `sheet` | `#ffffff` | `#1b1f25` | Document, table and slide surface |
| `slate-tint` | `#eceff3` | `#252a32` | Neutral diagram block, table header |
| `line` | `#d5dae1` | `#313842` | Hairlines |
| `ink` | `#1b2430` | `#e6e9ee` | Primary text, heavy rules |
| `slate` | `#5a6675` | `#9ba5b2` | Secondary text, captions, doc numbers |
| `orange` | `#e8590c` | `#f07a3a` | The accent: marks, rules, focus ring (not small text in light) |
| `orange-ink` | `#b4470a` | `#f39257` | Small orange text (labels, footnote markers) |
| `orange-tint` | `#fde9dd` | `#3a261a` | Decision block fill |
| `ok` / `ok-ink` / `ok-tint` | `#2b8a3e` / `#237032` / `#e2f1e5` | `#4cb563` / `#6cc77f` / `#18301e` | Permitted path: mark / text / fill |
| `stop` | `#c92a2a` | `#ef6b63` | Inhibit lines, e-stop text |
| `stop-fill` + `on-stop-fill` | `#c92a2a` + `#fff` | `#b8322b` + `#fff` | The safe-state block only |
| `stop-tint` | `#f9e1e1` | `#3a1d1d` | Quiet "results in all-off" row |
| `warn` | `#b7791f` | `#d9a23f` | Degraded states, marks only |

Type scale (px, size/line-height, weight): display 34/40 600, h1 26/32 600, h2 19/26 600, body 15/24 400 (max 68 characters a line), small 13/20, caption 12.5/18, code 13/20 mono, doc number 12/16 mono 500, label 11/14 mono 500 caps +0.08em. Headings get -0.01 to -0.015em tracking.

Spacing on a 4px base: 4, 8, 12, 16, 24, 32, 48. Radii: 3px for blocks and chips, 6px for sheets and slides.

## Diagrams

The default diagram is a **signal path**: one left-to-right figure that reads at a glance. See `signal-path.html`.

- **Flat fills, not outlines.** `slate-tint` = neutral module, `orange-tint` = decision or check, `ok-tint` = permitted result, `ink` with light text = hardware endpoint, `stop-fill` = the safe state.
- **Only the safe state is solid red**, so the eye finds it first.
- **Line language:** solid = data or control flow. Dashed `5 4` in `stop` = inhibit or failure path. Dotted `2 3` in `slate` = telemetry or observe-only (e.g. to ROS). Small filled arrowheads in the line's color.
- **Zones:** thin `line` rules with a small slate heading (e.g. "Core 0, control task") instead of boxes around groups.
- **Labels:** block titles in Plex Sans 600, details in Plex Mono at caption size. Long conditions go in footnotes, not inside blocks.
- **Caption every figure:** "Figure n." in 600 ink, then one sentence in slate.
- 5 to 12 blocks per figure. Split anything bigger.
- Timeline or swimlane views lack immediate clarity. Use them only for a specific explainer slide, not general docs.

## Slides

The default deck is a **build-up deck**. See `build-up-deck.html`.

- 16:9, drawn at 1280×720 and scaled to fit.
- Frame: `sheet` ground, one caps label in `orange-ink` at the top, a sentence-case title (52px, 600), and a footer row over a `line` rule with the deck's doc number on the left and the page count on the right.
- One idea per slide. Tables use the spec-table style with real units.
- Build-ups: the key figure assembles one group per slide with one sentence each. Earlier steps dim to about 30% and the new step fades in.
- Simple beats interactive when presenting. Live or interactive slides are hard to pull off on stage; keep those for web pages.
- The cover uses the palette band (ink, slate-tint, orange, a small stop-fill square) across the top edge, with the title in Plex Sans 600.
- When a deck needs .pptx or PDF export, build it in the claude.ai Slides artifact type using these rules.

## Components (class names in the design-system artifact)

- **Doc header:** title, doc number `KARR-<repo>-<nnn>` (`SC` = safety core, `RB` = karr_robot), revision letter and ISO date, closed by a 2px ink rule.
- **Spec table:** 3 to 4 columns, units in cells, slate header row, footnote markers in `orange-ink`.
- **Status chip:** a dot plus the firmware's own state word (ACTIVE, FAILSAFE, E-STOP ch6, WAITING, LOST). Never a dot alone.
- **Slide frame:** as above.

## When asking Claude for visuals

- Give 3 options from safe to bold, each built with real project content.
- Include one interactive option when it fits (e.g. a scenario explorer with toggles). The user likes that style for web pages even when it isn't picked.
- Render and check the result before showing it.

## Explored and parked

- **Rejected:** loud hazard / safety-sign look (yellow and black, heavy condensed caps); drafting-sheet grid with zone borders.
- **Parked:** the "lockout tag" component (orange tag with punch hole and black band) for safety facts. Revisit only with a real use.

## Live references (private claude.ai artifacts, owner only)

- Design system: https://claude.ai/artifact/HNLbY883KrMYTAjeByz8GN
- Diagram options: https://claude.ai/artifact/6gd4PqsstTV3Kw9VkoBgVN
- Deck options: https://claude.ai/artifact/8K2TXKw7aqs98ptL4KSBsE
