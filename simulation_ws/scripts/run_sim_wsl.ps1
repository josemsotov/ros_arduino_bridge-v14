$ErrorActionPreference = 'Stop'

$linuxUser = (wsl.exe -d Ubuntu-24.04 -- whoami).Trim()
$workspaceWsl = "/home/$linuxUser/smart_trolley_sim_ws"

# Mesa does not select the WSL D3D12 backend automatically on this PC.
$command = "export GALLIUM_DRIVER=d3d12; export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA; source /opt/ros/jazzy/setup.bash; source '$workspaceWsl/install/setup.bash'; ros2 launch follower_sim sim_follower.launch.py"
wsl.exe -d Ubuntu-24.04 -- bash -lc $command
