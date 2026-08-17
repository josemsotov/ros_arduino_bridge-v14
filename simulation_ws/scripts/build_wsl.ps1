$ErrorActionPreference = 'Stop'

$workspaceWindows = Split-Path -Parent $PSScriptRoot
$linuxUser = (wsl.exe -d Ubuntu-24.04 -- whoami).Trim()
$workspaceWsl = "/home/$linuxUser/smart_trolley_sim_ws"
$sourceWsl = (wsl.exe -d Ubuntu-24.04 -- wslpath -a $workspaceWindows).Trim()

Write-Host "Synchronizing simulation sources to $workspaceWsl"
wsl.exe -d Ubuntu-24.04 -- bash -lc "mkdir -p '$workspaceWsl/src/follower_sim' '$workspaceWsl/src/robot_follower' && cp -a '$sourceWsl/src/follower_sim/.' '$workspaceWsl/src/follower_sim/' && cp -a '$sourceWsl/src/robot_follower/.' '$workspaceWsl/src/robot_follower/'"

Write-Host 'Building ROS 2 workspace in the Linux filesystem'
wsl.exe -d Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash && cd '$workspaceWsl' && colcon build --packages-select robot_follower follower_sim --symlink-install"

Write-Host "Build complete: $workspaceWsl"
