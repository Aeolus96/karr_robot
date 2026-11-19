#!/usr/bin/env bash
# Regenerate karr.urdf from the SolidWorks export, build, and run all checks.
# Runs inside the dev container:  ./dev.sh verify   (Windows: .\dev.ps1 verify)
set -e
source /usr/local/bin/ws-env
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
T=$REPO/urdf_prep/tools
P=$REPO/karr_description
WS=$HOME/ros2_ws

echo "== convert"; python3 $T/convert_sw_export.py $REPO/urdf_prep/karr_description_ros1 $P
echo "== build";   cd $WS && colcon build --symlink-install --packages-select karr_description 2>&1 | tail -1
source install/setup.bash
echo "== check_urdf"; check_urdf $P/urdf/karr.urdf | tail -n +3
echo "== inertia";    python3 $T/inertia_report.py $P/urdf/karr.urdf
echo "== frames";     python3 $T/frame_report.py $P/urdf/karr.urdf base_link livox_frame livox_imu_link \
                        left_zed_camera_link center_zed_camera_link right_zed_camera_link left_gps_link right_gps_link
echo "== render";     python3 $T/render_urdf.py $P/urdf/karr.urdf $P/meshes $REPO/urdf_prep/karr_views.png
echo "   -> urdf_prep/karr_views.png"
