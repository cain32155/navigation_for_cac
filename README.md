# SLAM Package — CAC 2026-27

---

## Work Split

| Person | Responsibility | Files |
|---|---|---|
| Person 1 | slam_toolbox (2D SLAM) | `launch/slam_toolbox.launch.py`, `config/slam_toolbox.yaml` |
| Person 2 | RTABMap (3D SLAM) | `launch/rtabmap.launch.py`, `config/rtabmap.yaml` |
| Person 3 | STVL (Obstacle Layer) | `config/stvl.yaml` |

**Never edit someone else's files.**

---

## Package Structure

```
ros2_ws/
└── src/
    └── slam/
        ├── launch/
        │   ├── slam_toolbox.launch.py
        │   ├── rtabmap.launch.py
        │   └── stvl.launch.py
        ├── config/
        │   ├── slam_toolbox.yaml
        │   ├── rtabmap.yaml
        │   └── stvl.yaml
        ├── src/
        ├── include/
        │   └── slam/
        ├── CMakeLists.txt
        └── package.xml
```

---

## One Time Setup

```bash
# Clone the repo
git clone <repo_url>
cd Caterpillar_2027

# Switch to slam branch
git checkout slam

# Initialise rosdep (only if never done before on your machine)
sudo rosdep init
rosdep update
```

---

## Every Time You Start Working

```bash
cd ~/Caterpillar_2027/ros2_ws

# Pull latest changes from teammates
git pull origin slam

# Install any new dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source
source install/setup.bash
```

---

## After Editing Any File

```bash
cd ~/Caterpillar_2027/ros2_ws

# Rebuild
colcon build

# Source again
source install/setup.bash
```

---

## When To Run rosdep

Only run rosdep when:
- Setting up for the first time
- A teammate adds a new dependency to `package.xml`
- You see a missing package error during `colcon build`

---

## Pushing Your Changes

```bash
cd ~/Caterpillar_2027

# Check what you changed
git status

# Stage only your files
git add ros2_ws/src/slam/launch/<your_file>
git add ros2_ws/src/slam/config/<your_file>

# Commit
git commit -m "your message here"

# Push
git push origin slam
```

---

## Dependencies On Other Teams

| What | Topic | From |
|---|---|---|
| LiDAR scan | `/scan` | LiDAR driver team |
| Camera RGB | `/camera/color/image_raw` | RealSense driver team |
| Camera Depth | `/camera/aligned_depth_to_color/image_raw` | RealSense driver team |
| Camera Info | `/camera/color/camera_info` | RealSense driver team |
| Odometry | `/odometry/filtered` | EKF team |
| IMU | `/imu/data` | IMU driver team |
| Transforms | `base_link → laser`, `base_link → camera_link` | Description team |