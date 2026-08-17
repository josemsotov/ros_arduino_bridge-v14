$ErrorActionPreference = 'Stop'

$linuxUser = (wsl.exe -d Ubuntu-24.04 -- whoami).Trim()
$workspaceWsl = "/home/$linuxUser/smart_trolley_sim_ws"

# Force the accelerated WSL renderer for Gazebo and RViz. Without these
# variables Mesa selects llvmpipe on this PC.
$command = "export GALLIUM_DRIVER=d3d12; export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA; unset LIBGL_ALWAYS_SOFTWARE; source /opt/ros/jazzy/setup.bash; source '$workspaceWsl/install/setup.bash'; ros2 launch follower_sim nav2_golf_demo.launch.py headless:=false rviz:=true kinect:=true"

Write-Host 'Starting Gazebo GUI + RViz + Nav2 with D3D12/NVIDIA acceleration'
wsl.exe -d Ubuntu-24.04 -- bash -lc $command
