# Viable Nav2 baseline

Validated on 2026-08-17 for the headless golf-trolley simulation under
Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic in WSL.

## Accepted configuration

- D3D12 hardware acceleration on the NVIDIA Quadro T2000. Do not launch
  Gazebo without the D3D12 environment variables: it falls back to llvmpipe.
- Regulated Pure Pursuit controller for the differential-drive trolley.
- Goal tolerances: 0.06 m position and 0.15 rad heading.
- Costmap inflation radius: 0.80 m, above the 0.7286 m circumscribed radius.
- Local voxel window: z=1.0..1.8 m, containing the lidar plane at z=1.35 m.
- Balanced AMCL: 90 beams and 500..1500 particles on the 0.05 m map.

Two consecutive reference runs reached `(1, -1, 0)` in 20.96 s and 22.47 s
wall time, with 6.67 cm and 6.29 cm physical position error. Neither run used
a recovery behavior or emitted costmap geometry errors.

## Real-time Gazebo + RViz profile

The visual profile uses a 200 Hz physics step (`max_step_size=0.005`), a
minimal Gazebo GUI, disabled shadows, RViz limited to 15 FPS, and the RViz
LaserScan display disabled by default. The lidar sensor itself remains active
for AMCL and Nav2.

Measured with both windows open:

- Idle real-time factor: 0.9962.
- Navigation real-time factor: 0.9698.
- Reference-goal physical error: 6.02 cm.
- AMCL-to-Gazebo position error: 0.78 cm.

## Build

```powershell
.\simulation_ws\scripts\build_wsl.ps1
```

## Run the headless system

```powershell
.\simulation_ws\scripts\run_nav2_wsl.ps1
```

This starts only the simulated robot. It does not command the Arduino or the
physical motors.

## Run Gazebo and RViz in real time

```powershell
.\simulation_ws\scripts\run_nav2_gui_wsl.ps1
```

The lightweight profile shows the robot in both applications. Enable the
LiDAR display manually in RViz only when scan visualization is needed.

## Re-run the benchmark

```powershell
$script = (wsl.exe -d Ubuntu-24.04 -- wslpath -a (Resolve-Path .\simulation_ws\scripts\benchmark_nav2_wsl.sh)).Trim()
wsl.exe -d Ubuntu-24.04 -- bash $script d3d12 /home/robotdev/smart_trolley_sim_ws 180
```

The runner waits for Nav2, records wall and simulation time, prints the AMCL
pose and Gazebo ground truth, and terminates the complete launch process group.
If `/clock` never starts, it aborts without sending a navigation goal.

## Operational acceptance criteria

- `NavigateToPose` returns `SUCCEEDED`.
- Physical position error is below 0.10 m.
- Physical yaw error is below 0.15 rad for pose goals.
- No `Failed to make progress`, recovery spin, inflation-radius error, or
  lidar-out-of-costmap warning is present.
- No `gz sim` process remains after an automated benchmark.
