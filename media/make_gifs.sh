#!/usr/bin/env bash
# Turn rendered frames (media/frames/<scene>/f_####.png, 24 fps) into looping GIFs for the README, plus an HD MP4
# of the mission (linked from the README: GitHub does not embed repo videos inline, GIFs autoplay).
#   bash media/make_gifs.sh                # both scenes
#   bash media/make_gifs.sh mission 560    # one scene, GIF width 560
# Needs ffmpeg. GitHub serves GIFs as-is, so keep them small (aim ~10 MB).
set -euo pipefail
cd "$(dirname "$0")"
scenes=("${1:-turntable mission}")
width="${2:-}"
for s in ${scenes[@]}; do
  [[ -d frames/$s ]] || { echo "no frames/$s, render first"; continue; }
  # width / fps / ordered-dither scale per scene. Ordered (bayer) dithering compresses far better than error
  # diffusion for moving-camera footage.
  case $s in
    turntable) w=${width:-400}; fps=24; bayer=4 ;;
    mission)   w=${width:-480}; fps=12; bayer=5 ;;   # ~21 s loop: 12 fps keeps it ~10 MB
    *)         w=${width:-480}; fps=15; bayer=5 ;;
  esac
  ffmpeg -y -loglevel error -framerate 24 -i "frames/$s/f_%04d.png" \
    -vf "fps=$fps,scale=$w:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=256:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=$bayer:diff_mode=rectangle" \
    -loop 0 "karr_$s.gif"
  echo "karr_$s.gif  $(du -h "karr_$s.gif" | cut -f1)"
done
if [[ " ${scenes[*]} " == *" mission "* && -d frames/mission ]]; then
  # HD video: full render resolution, H.264, web-friendly (moov atom first)
  ffmpeg -y -loglevel error -framerate 24 -i frames/mission/f_%04d.png -c:v libx264 -preset slow -crf 20 \
    -pix_fmt yuv420p -movflags +faststart karr_mission.mp4
  echo "karr_mission.mp4  $(du -h karr_mission.mp4 | cut -f1)"
fi
