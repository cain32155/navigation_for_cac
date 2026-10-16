# SLAM Documentation: CAC 2026-27
---
## 1. Overview

SLAM (Simultaneous Localization and Mapping) allows the rover to build a map of the arena while tracking its own position within that map — at the same time.

The rover uses a **hybrid SLAM architecture**:
- **2D SLAM** via `slam_toolbox` using the LiDAR — produces a 2D occupancy grid for navigation
- **3D SLAM** via `RTABMap` using the RealSense depth camera — produces a 3D point cloud map

Both run simultaneously and share the same coordinate frame, with `slam_toolbox` acting as the master.

---

## 2. Hardware

| Sensor | Model | Role |
|---|---|---|
| LiDAR | RPLIDAR A3M1 | 2D laser scanning, 25m range |
| Depth Camera | Intel RealSense D435i | RGB + depth, 3D mapping |
| IMU | BNO085 | Orientation data |

---

## 3. Software

| Package | Version | Role |
|---|---|---|
| `slam_toolbox` | ROS 2 Jazzy | 2D SLAM |
| `rtabmap_ros` | ROS 2 Jazzy | 3D SLAM |
| `robot_localization` | ROS 2 Jazzy | EKF odometry |

---

## 4. Architecture

```
LiDAR ──────────────────────────→ slam_toolbox ──→ /map (2D)
                                        │
                                        └──→ map→odom transform (master)
                                                      │
RealSense ──→ RGB + Depth + CameraInfo → RTABMap ←───┘
                                              │
                                              └──→ /rtabmap/cloud_map (3D)

Wheel Encoders ──→
                   EKF ──→ /odometry/filtered ──→ slam_toolbox
IMU ────────────→                                 RTABMap

                              Nav2
                         ┌─────┴─────┐
                    Global Map    Local Costmap
                    (from /map)   (STVL layer)
                                  ↑        ↑
                               LiDAR   RealSense
```

`slam_toolbox` owns the `map → odom` transform. RTABMap publishes **no transforms** as it uses slam_toolbox's pose as its odometry input. This ensures both maps share the same coordinate frame and stay aligned.

---

## 5. Nodes

| Node | Package | Language | Role |
|---|---|---|---|
| `slam_toolbox` | `slam_toolbox` | C++ | 2D mapping and localization |
| `rtabmap` | `rtabmap_ros` | C++ | 3D mapping |
| `stvl` | `spatio_temporal_voxel_layer` | C++ | Instantaneous obstacle detection |

---

## 6. Topics

### 6.1 Inputs

| Topic | Message Type | From | Used By |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver | slam_toolbox, STVL |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense driver | RTABMap |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense driver | RTABMap, STVL |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RealSense driver | RTABMap |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF | slam_toolbox, RTABMap |
| `/imu/data` | `sensor_msgs/Imu` | Madgwick filter | RTABMap |

### 6.2 Outputs

| Topic | Message Type | To | What it is |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Nav2 | 2D occupancy grid |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | Visualization | 3D point cloud |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2 | Instantaneous obstacle layer |

---

## 7. Transforms

| Transform | Published By | What it means |
|---|---|---|
| `map → odom` | slam_toolbox | Where rover is in the arena |
| `odom → base_link` | EKF | How much rover has moved |
| `base_link → laser` | URDF | Where LiDAR is mounted on rover |
| `base_link → camera_link` | URDF | Where RealSense is mounted on rover |

---

## 8. Package Structure

```
slam/
├── launch/
│   └── slam.launch.py       ← Python, starts all SLAM nodes
├── config/
│   ├── slam_toolbox.yaml    ← slam_toolbox parameters
│   ├── rtabmap.yaml         ← RTABMap parameters
│   └── stvl.yaml            ← STVL parameters
├── src/                     ← C++ custom nodes
├── include/                 ← C++ headers
├── CMakeLists.txt           ← ament_cmake
└── package.xml
```

---

## 9. Configuration

### 9.1 slam_toolbox.yaml

```yaml
slam_toolbox:
  ros__parameters:
    mode: async
    max_laser_range: 25.0
    minimum_travel_distance: 0.15
    minimum_travel_heading: 0.15
    loop_search_maximum_distance: 3.0
```

### 9.2 rtabmap.yaml

```yaml
rtabmap:
  ros__parameters:
    Rtabmap/DetectionRate: "1"        # 1Hz — saves CPU
    Mem/STMSize: "30"                 # memory management
    Grid/FromDepth: "false"           # slam_toolbox handles 2D grid
    RGBD/ProximityBySpace: "true"
    max_depth: 4.0                    # RealSense reliable range
    Rtabmap/PublishTf: "false"        # slam_toolbox owns transforms
```

### 9.3 stvl.yaml

```yaml
spatio_temporal_voxel_layer:
  enabled: true
  voxel_decay: 15.0               # seconds before voxels fade
  decay_model: 0
  voxel_size: 0.05                # 5cm voxel resolution
  track_unknown_space: false
  observation_sources: lidar depth_camera
  lidar:
    topic: /scan
    sensor_type: laser_scan
    marking: true
    clearing: true
  depth_camera:
    topic: /camera/aligned_depth_to_color/image_raw
    sensor_type: depth_camera
    marking: true
    clearing: true
    max_obstacle_height: 2.0
    min_obstacle_height: 0.1
```

---

## 10. Operating Modes

| Mode | When | What Happens |
|---|---|---|
| **Mapping** | First run, exploring arena | Both SLAMs build maps, save to database |
| **Localization** | Competition run | Loads saved map, only tracks position — much lighter on CPU |

---

## 11. Dependencies on Other Teams

| What | From Which group | Why |
|---|---|---|
| URDF with sensor positions | URDF group | Transforms won't work without correct sensor positions |
| `/scan` topic working | LiDAR group | slam_toolbox and STVL cannot run without it |
| RealSense topics working | RealSense group | RTABMap and STVL cannot run without it |
| `/imu/data` working | IMU driver group | RTABMap orientation data |

---

## 12. Suggestions

EKF for odometry:
    Although UKF has more accuracy, it introduces extreme computational load on the system and EKF has a higher compatability with ROS2.  
    Also, the motion model isn't highly non-linear, making UKF overkill

Controls:
    PID on ESP32: low level wheel speed control
    RPP on Jetson: high level path following via Nav2