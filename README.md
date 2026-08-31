# LIO_SLAM — Real-Time LiDAR-Inertial SLAM for UAV Subterranean Mapping

Real-time, resource-constrained 3D SLAM for autonomous UAVs operating in GPS-denied, unlit environments such as caves and mining tunnels — built around an edge-cloud split-compute architecture running FAST-LIO2 on a Raspberry Pi 5.

![Edge-cloud architecture overview](docs/images/architecture-overview.png)

This repository accompanies my Master of Engineering thesis at University College Dublin, and an associated conference paper.

## Why this exists

Subterranean environments such as collapsed mines, natural cave systems and disaster sites are GPS-denied and completely dark, ruling out standard navigation and most visual SLAM. LiDAR-inertial SLAM solves the perception problem, but state-of-the-art algorithms are built for desktop-class high end computing platforms. Running them natively on a lightweight UAV payload causes RAM saturation and CPU overload mid-flight which is a hard blocker for real deployment, not just a performance nuisance.

This project builds and physically validates a split-compute architecture that keeps the UAV alive with **survival odometry on the edge** (Raspberry Pi 5) while offloading the **heavy global mapping and loop closure/optimisation to a base station** and proves it works, in the dark, with real hardware.

## Dataset

Two sources of data underpin the results below:

- **[NCLT](https://robots.engin.umich.edu/nclt/)** — the University of Michigan's public North Campus Long-Term Vision and LiDAR benchmark, used to validate baseline odometry accuracy against RTK-GPS ground truth (1.146 km trajectory).
- **[UCD_LIO_SLAM_Dataset](https://github.com/Stepan-Letsko/UCD_LIO_SLAM_Dataset)** — a custom LiDAR-IMU dataset I collected with the physical payload described below, across three environments: a GPS-denied basement corridor, an open lake circuit, and the engineering building exterior. Released separately with full sensor extrinsics, topic descriptions, and download links.

## Results at a glance

![High-definition point cloud reconstructions](docs/images/Hd-scan-visuals.png)
*Close-up detail from the onboard 3D reconstructions. Left: Basement Corridor. Middle: Lake Circuit. Right: Engineering Building Exterior.*

| Environment | Closing Error (m) | Drift (% trajectory) | Avg Latency (ms) | Peak RAM (MB) | Frame Drop Rate |
|---|---|---|---|---|---|
| Basement Corridor | 0.5287 m | 0.2505% | 19.54 ms | 177.91 MB | <1% |
| Lake Circuit | 0.5887 m | 0.1169% | 77.42 ms | 453.59 MB | <1% |
| Engineering Exterior | 0.0764 m | 0.0175% | 70.73 ms | 497.54 MB | <1% |

All three physical deployments held sub-metre closing error with <1% frame drops — including under the near-darkness, GPS-denied conditions that directly represent the target cave scenario. Baseline accuracy was independently validated on the public NCLT benchmark, achieving an RMSE of 0.908 m over a 1.146 km trajectory (0.07% mean translational error).

The edge-cloud split is what makes this survivable on embedded hardware in the first place: an odometry-only configuration held a peak of **588 MB** RAM indefinitely, while running native global mapping on the same Raspberry Pi 5 grew unbounded to 8.5 GB and crashed with an out-of-memory error.

![Per-frame latency, RAM, and CPU over the full flight duration](docs/images/Results-time-series.png)
*Processing latency, RAM consumption, and CPU utilisation over time for all three physical deployments — latency stays well under the 100 ms real-time threshold throughout, and RAM plateaus rather than growing unbounded.*

### Parameter sweep and operating point selection

![FAST-LIO2 parameter sweep results](docs/images/parameter-sweep-heatmaps.png)
*25-configuration parameter sweep on the Raspberry Pi 5 characterising the RAM / latency / frame-drop trade-off across voxel resolution and point filter settings.*

Computational feasibility alone doesn't guarantee mapping accuracy — several configurations that satisfy the real-time constraint still produce unusable maps. The closing error heatmap below is what actually justifies the final `k = 5`, `v = 0.4 m` operating point: it's the smallest voxel size (i.e. densest, most accurate point cloud) that still holds sub-metre closing error.

![Closing error heatmap for the Lake Circuit environment](docs/images/Closing-error-heatmap.png)
*Closing error across the same 25 configurations — configurations at v ≤ 0.2 m diverge into tens-of-kilometres of error despite passing the real-time check, since frame drops break state propagation continuity.*

## Point cloud reconstructions vs. satellite imagery

![Point cloud reconstructions overlaid on satellite imagery](docs/images/point-cloud-reconstructions.png)
*Left: onboard-generated 3D reconstructions. Right: satellite imagery of the same site, for structural comparison. Top to bottom: UCD lake circuit, basement corridor, engineering building exterior.*

## How it works

![Edge-cloud data flow](docs/images/edge-cloud-dataflow.png)

- **Edge layer (onboard the UAV, Raspberry Pi 5):** sensor drivers, hardware PPS time synchronisation between LiDAR and IMU, motion deskewing, and the FAST-LIO2 IESKF for high-frequency local odometry. Also extracts keyframes and Scan Context descriptors for loop closure. This is the only processing that has to survive in flight.
- **Cloud / base station layer:** receives odometry poses and loop closure constraints over a 5 GHz ROS 2 telemetry link, runs GTSAM pose-graph optimisation, and reconstructs the final globally consistent 3D map. Visualisation is handled through Foxglove Studio.

This division of labour is what keeps peak onboard RAM under 500 MB indefinitely, instead of growing unbounded until the hardware crashes.

![Command centre view from Foxglove](docs/images/Foxglove-command-centre.png)
*Live operator view during a mission: real-time 3D point cloud, IMU acceleration/angular velocity plots, a 2D trajectory minimap, and remote start/stop recording controls — all streamed headless from the Pi 5 over the 5 GHz telemetry link.*

### Hardware

![Sensor payload](docs/images/hardware-payload.png)
![Custom 3D-printed sensor rig](docs/images/sensor-rig-cad.png)

| Component | Part |
|---|---|
| LiDAR | Hesai Pandar XT-16 (16-channel, 360°) |
| IMU | SBG Ellipse-D (>200 Hz, PPS-synchronised) |
| Edge compute | Raspberry Pi 5 (quad-core ARM Cortex-A76) |
| Mount | Custom 3D-printed rigid sensor payload |

### Software

- ROS 2 Humble, containerised with Docker
- FAST-LIO2 (ported and modernised from ROS 1 → ROS 2 Humble)
- Custom Hesai XT-16 driver with point cloud format adaptation
- Offline loop closure: Scan Context descriptor matching → ICP geometric verification → Open3D pose graph optimisation
- Foxglove Studio telemetry / visualisation pipeline
- A custom evaluation toolkit (benchmarking, parameter sweeps, closing-error and APE computation — see [Tooling](#tooling--useful-commands) below)

## Repository structure

```
.
├── docs/
│   ├── paper.pdf              # IEEE ITNAC 2026 conference paper
│   ├── thesis.pdf             # Full ME thesis submission
│   └── images/                # Figures used in this README
├── src/
│   ├── FAST_LIO_ROS2/          # FAST-LIO2 package + sensor config files (hesai.yaml, avia.yaml, velodyne.yaml)
│   ├── scripts/                # Benchmarking, sweep, and evaluation tooling (see below)
│   └── mission.launch.py       # Full-stack launch file (drivers + FAST-LIO2 + Foxglove bridge)
├── bags/                       # Recorded ROS 2 bag datasets
├── benchmark.py                # Parameter sweep + KPI evaluation entry point
├── Dockerfile                  # Container image definition
├── docker-compose.yml          # Container orchestration
└── entrypoint.sh                # Container startup script
```

## Getting started

```bash
git clone https://github.com/Stepan-Letsko/LIO_SLAM.git
cd LIO_SLAM

# build and start the containerised environment
docker compose build         # first run only, ~10-20 minutes
docker compose up -d
docker exec -it thesis_container bash

# inside the container: build the ROS 2 workspace
# -DHUMBLE_ROS=humble is required to fix the Livox driver path resolution on ROS 2 Humble
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
source install/setup.bash

# launch the full SLAM stack (drivers + FAST-LIO2 + Foxglove bridge)
ros2 launch src/mission.launch.py
```

### Automated launch on the physical UAV payload

In the field, there's no monitor or keyboard attached to the Raspberry Pi 5 — only a single SSH/Wi-Fi connection to a base station laptop. To make this practical, the entire stack is registered as a systemd service (`slam_autostart.service`) that runs automatically on every boot: it waits for Docker and networking to be ready, starts the container, and launches the full mission pipeline inside it — bringing the Pi from power-on to a fully operational SLAM pipeline in about 15 seconds, with zero manual steps. The Pi is also configured to broadcast its own 5 GHz Wi-Fi hotspot on boot, so the base station laptop can connect and start monitoring immediately.

```bash
# Check status / view live logs
sudo systemctl status slam_autostart.service
sudo journalctl -u slam_autostart.service -f

# Manually stop / start / disable / re-enable
sudo systemctl stop slam_autostart.service
sudo systemctl start slam_autostart.service
sudo systemctl disable slam_autostart.service   # won't start on next boot
sudo systemctl enable slam_autostart.service    # re-enable autostart
```

## Tooling & Useful Commands

A significant part of this project was building the tooling around FAST-LIO2 to make it testable, benchmarkable, and reproducible — not just getting the algorithm running once. The scripts below live in `src/scripts/`.

**Playing back a recorded bag against the algorithm** (two terminals inside the container — one runs the estimator, one plays the data):
```bash
# Terminal 1 — the estimator, waiting for data
ros2 launch fast_lio mapping.launch.py config_file:=hesai.yaml

# Terminal 2 — source the workspace first, or ROS won't recognise custom message types in the bag
source install/setup.bash
cd bags
ros2 bag play <bag_folder_name>
```

**Benchmarking raw processing time per frame** — reports average/min/max frame time over a full bag run:
```bash
python3 src/scripts/benchmark_fastlio.py bags/<bag_name> <config>.yaml
```

**Full analysis pipeline** — runs FAST-LIO2 against a bag and records odometry, point clouds, and trajectory in one pass:
```bash
python3 src/scripts/run_full_analysis.py bags/<bag_name> <config>.yaml --record all
# --record odometry   just /Odometry
# --record inputs     LiDAR + IMU passthrough only
# --no-map --skip-pcd  skip map/PCD generation for a faster run
```

**Parameter sweep** — the tool behind the RAM/latency/frame-drop heatmaps above, sweeping voxel size and point filter settings:
```bash
python3 src/scripts/run_parameter_sweep.py bags/<bag_name> --config src/FAST_LIO_ROS2/config/hesai.yaml --duration 60.0
python3 src/scripts/plot_sweep_heatmaps.py src/results/parameter_sweep/sweep_results_matrix.csv
```

**Frequency sweep** — characterises accuracy/performance sensitivity to LiDAR/IMU input rate:
```bash
python3 src/scripts/run_frequency_sweep.py bags/<bag_name> --config <config>.yaml --imu_topic /imu/data --lidar_topic /lidar_points --duration 120
```

**Closing error** — Euclidean distance between estimated start and end pose of a closed-loop trajectory, the core accuracy metric used for the physical UCD deployments:
```bash
python3 src/scripts/compute_closing_error.py <trajectory>.tum
```

**Absolute Pose Error vs. ground truth** (via the [evo](https://github.com/MichaelGrupp/evo) package) — used for the NCLT benchmark validation:
```bash
evo_ape tum groundtruth.tum estimate.tum -va --plot --plot_mode=xy
```

**Bag → PCD map export**:
```bash
python3 src/scripts/bag_to_pcd.py <bag_folder> --topic /cloud_registered --output final_map.pcd
```

**Converting a ROS 1 bag to ROS 2** (for testing against public ROS 1 datasets, e.g. NCLT):
```bash
rosbags-convert --src <bag_name>.bag --dst <output_folder> --dst-version 8
```

**Foxglove bridge** — streams the downsampled/registered cloud rather than raw `/lidar_points`, since full-resolution point clouds would saturate the Wi-Fi telemetry link:
```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p topic_whitelist:='["^(?!/lidar_points$).*"]'
```

## Documentation

- 📄 [Conference paper](docs/paper.pdf) — "On-Board Real-Time 3D LiDAR-IMU SLAM for UAV Subterranean Mapping"
- 📘 [Full thesis](docs/thesis.pdf) — "Real-Time LiDAR-Inertial SLAM for Resource-Constrained UAV Subterranean Mapping"

## Future work

- Onboard path-planning and obstacle avoidance for fully autonomous return-to-home when base station connectivity is lost during deep subterranean telemetry attenuation.

## Author

**Stepan Letsko** — ME Electronic & Computer Engineering, University College Dublin



