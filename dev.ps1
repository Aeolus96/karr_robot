<#
  K.A.R.R. dev container helper (Windows PowerShell).   Usage:  .\dev.ps1 <command> [args]

    build      Build the Docker image
    up         Start the container in the background
    down       Stop and remove the container (build volume is kept)
    shell      bash inside the container (ROS sourced, workspace at ~/ros2_ws)
    colcon     colcon build --symlink-install [args]
    deps       rosdep install for every package in the repo
    verify     Regenerate karr.urdf from the SolidWorks export, build, run all checks
    display    Build, then show K.A.R.R. in rviz2 with joint sliders (extra args go to ros2 launch)
    sim        Build, then start the Gazebo simulation (extra args go to ros2 launch, e.g. steering_mode:=four_wheel)
    teleop     Keyboard teleop in this terminal (when the sim runs with teleop:=false)
    test       Build and run the package tests
    glxtest    Print the OpenGL renderer the container sees
    reset      Remove container AND build volume (clean rebuild)

  GUI goes through WSLg by default. If that doesn't work:  $env:KARR_GUI='vnc'  then open
  http://localhost:6080/vnc.html
#>
param([Parameter(Position=0)][string]$Cmd = 'help',
      [Parameter(Position=1, ValueFromRemainingArguments=$true)][string[]]$Rest)

$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$gui = if ($env:KARR_GUI) { $env:KARR_GUI } else { 'wslg' }
$files = @('-f', 'docker/compose.yaml')
if ($gui -ne 'none') { $files += @('-f', "docker/gui-$gui.yaml") }

function DC { docker compose @files @args }
function Up {
  DC up -d
  if ($gui -eq 'vnc') { Write-Host 'GUI: http://localhost:6080/vnc.html?autoconnect=1&resize=remote' }
}
function In-Ws([string]$line) { DC exec ros ws-env bash -c $line }

switch ($Cmd) {
  'build'   { DC build }
  'up'      { Up }
  'down'    { DC down }
  'shell'   { Up; DC exec ros bash }
  'colcon'  { Up; In-Ws ('colcon build --symlink-install ' + ($Rest -join ' ')) }
  'deps'    { Up; In-Ws 'sudo apt-get update && rosdep install --from-paths src --ignore-src -y' }
  'verify'  { Up; DC exec ros bash /home/ubuntu/ros2_ws/src/karr_robot/urdf_prep/tools/verify.sh }
  'display' { Up; In-Ws ('colcon build --symlink-install && source install/setup.bash && ros2 launch karr_description display.launch.py ' + ($Rest -join ' ')) }
  'sim'     { Up; In-Ws ('colcon build --symlink-install && source install/setup.bash && ros2 launch karr_gazebo sim.launch.py ' + ($Rest -join ' ')) }
  'teleop'  { Up; In-Ws 'ros2 run teleop_twist_keyboard teleop_twist_keyboard' }
  'test'    { Up; In-Ws 'colcon build --symlink-install && colcon test --event-handlers console_direct+ && colcon test-result --verbose' }
  'glxtest' { Up; In-Ws 'glxinfo -B | grep -i renderer' }
  'reset'   { DC down -v }
  default   { (Get-Content $PSCommandPath -TotalCount 20) -join "`n" | Write-Host }
}
