#!/usr/bin/env bash
# Turn rendered frames (media/frames/<scene>/f_####.png) into looping GIFs for the README.
#   bash media/make_gifs.sh              # both
#   bash media/make_gifs.sh action 640   # one scene, output width 640
# Needs ffmpeg. Frames are rendered at 20 fps; GitHub serves GIFs as-is, so keep them small (< ~10 MB).
set -euo pipefail
cd "$(dirname "$0")"
scenes=("${1:-turntable action}")
width="${2:-}"
for s in ${scenes[@]}; do
  [[ -d frames/$s ]] || { echo "no frames/$s, render first"; continue; }
  # width / fps / ordered-dither scale per scene: tuned for ~8 MB (action) and ~3.5 MB (turntable).
  # Ordered (bayer) dithering compresses far better than error diffusion for moving-camera footage.
  case $s in
    turntable) w=${width:-400}; fps=20; bayer=4 ;;
    action)    w=${width:-480}; fps=15; bayer=5 ;;
  esac
  ffmpeg -y -loglevel error -framerate 20 -i "frames/$s/f_%04d.png" \
    -vf "fps=$fps,scale=$w:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=256:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=$bayer:diff_mode=rectangle" \
    -loop 0 "karr_$s.gif"
  echo "karr_$s.gif  $(du -h "karr_$s.gif" | cut -f1)"
done
