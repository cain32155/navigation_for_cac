# cac_sim — Caterpillar Autonomy Challenge 2027 simulation

ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) simulation of the CAC 2027 arena and an
autonomous excavation rover: randomized boulders and real crater depressions,
marker-based localization that does not use the arena walls, Nav2 navigation
with the rover's real footprint, and a mission state machine that only advances
on verified arrival.

```
ros2 launch cac_sim cac_sim.launch.py seed:=42            # GUI + RViz, full autonomy
```

Contents: [Build and run](#build-and-run) · [Arena](#arena) · [Rover](#rover) ·
[Sensors, topics, TF](#sensors-topics-and-tf) · [Autonomy stack](#autonomy-stack) ·
[Mission flow](#mission-state-machine) · [Testing](#testing) ·
[Rule interpretations](#competition-rules-and-interpretations) ·
[Known limitations](#known-limitations) · [Validation status](#validation-status)

---

## Build and run

`$WS` below is the colcon workspace root that contains `src/cac_sim` — e.g. the
root of the `Caterpillar_2027` clone, or `~/MOVIS` on the development machine.
Validation output goes to `$WS/suite_results/` (git-ignored).

```bash
export WS=~/Caterpillar_2027        # your workspace root
cd $WS
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cac_sim
source install/setup.bash

ros2 launch cac_sim cac_sim.launch.py seed:=6                      # GUI + RViz
ros2 launch cac_sim cac_sim.launch.py seed:=6 headless:=true use_rviz:=false   # server only
ros2 launch cac_sim cac_sim.launch.py autonomy:=false              # no mission (teleop/tests)
ros2 launch cac_sim cac_sim.launch.py randomize:=false             # empty arena, fixed start
```

| Argument | Default | Meaning |
|---|---|---|
| `seed` | 42 | Seeds the start pose and the obstacle layout (fully reproducible). |
| `randomize` | true | `false`: no obstacles, fixed start pose. |
| `autonomy` | true | `false`: the mission state machine idles. |
| `use_rviz` | true | Start RViz. |
| `headless` | false | Gazebo server only. Rendering sensors still need a display (`DISPLAY`). |
| `egl` | false | Adds `--headless-rendering`; on the development machine this segfaults the RGB-D camera (Ogre2). |
| `localhost_only` | true | Sets `GZ_IP=127.0.0.1` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (see below). |
| `run_dir` | `<tmp>/cac_sim_runs/seed_N` | Where the generated world, meshes and resolved Nav2 params go. |
| `world` | `cac_arena.sdf` | Base world (its sand bed is regenerated per run). |

**Loopback transport.** On a flaky Wi-Fi interface gz-transport logged thousands of
"multicast: Network is unreachable" errors, the ROS bridge stopped receiving
`/clock`, sim time froze and Nav2's bond watchdog shut navigation down. The launch
therefore keeps Gazebo transport and ROS discovery on 127.0.0.1. Any tool you run
from another shell (`ros2 topic`, `gz topic`, the test tools) needs the same:

```bash
export GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Each run prints the generated layout, e.g.

```
[cac_sim] seed=6 start=(-3.38, -0.35, yaw=180 deg) attempts=1
  boulder_0  (+0.94, -0.06) d=0.379 centre_z=0.150
  crater_2   (-1.56, +0.58) d=0.480 depth=0.079 ...
```

`ros2 run cac_sim obstacle_spawner --print 1 2 3` prints layouts without starting anything.

---

## Arena

World frame (= ROS `map`): X along the 9 m length, start zone at −X; Y across the
5 m width; Z = 0 is the sand surface.

| Element | Geometry |
|---|---|
| Sand bed | 9 × 5 m, 0.30 m deep, rigid (no granular physics), μ = 0.9 |
| Walls | inner faces at x = ±4.50, y = ±2.50; 0.04 m thick, **0.80 m tall**; corners overlap; visual = collision |
| Start zone | x ∈ [−4.5, −2.7] |
| Obstacle zone | x ∈ [−2.7, 2.0] |
| Excavation zone | x ∈ [2.0, 3.5] |
| Construction zone | x ∈ [3.5, 4.5]; berm target (4.0, 0), r = 0.4 m (visual marker) |
| Zone lines | 3 cm painted strips at x = −2.7, 2.0, 3.5 (visual only) |
| Markers | 5 ArUco (DICT_4X4_50, 20 cm, on 25 cm boards) on the start-zone perimeter |

The rulebook text gives no zone sizes (only a reference diagram); the layout is in
`config/arena_params.yaml` and is the single source of truth for every node.

### Per-run generation (`cac_sim/arena_layout.py`)

The launch file generates the world **before** Gazebo starts:

1. **Start pose** — N/S/E/W heading (rulebook), footprint inside the start zone
   with 0.20 m margin, and room to turn in place (farthest corner + 0.10 m from
   walls and obstacles) for the marker-search spin.
2. **Obstacles** — 3 boulders (Ø 0.30–0.40 m, sunk 0–40 % of their radius:
   "varying heights") and 3 craters in the obstacle zone, plus 1 smaller boulder
   (Ø 0.20–0.30 m) in the excavation zone ("similar or smaller boulders may also be
   present"). Centres ≥ 0.9 m apart and ≥ 0.35 m inside their zone.
3. **Reachability check** — the real footprint, grown by 0.10 m (0.20 m from
   walls), must be able to drive from the start to the excavation, construction and
   return goals (search over x, y and 16 headings, with turning in place).
   Spacing alone does not guarantee this: with the original rules 7–10 % of
   layouts were unsolvable. Rejected layouts are redrawn deterministically
   (attempt k uses `Random(seed·1000003 + k)`; attempt 0 is the original stream,
   so seeds that already passed keep their layout).
4. **Craters as real depressions** — each crater is a closed STL mesh: a square
   sand patch with a cosine bowl (total width 0.40–0.50 m, depth 0.05–0.15 m), flush
   with the surface. The solid sand box is replaced by axis-aligned floor tiles
   around the patches (a box cannot have holes). Verified live: a 6 cm probe
   sphere settles at the bowl bottom (−0.115 m in a 0.117 m crater) and rests at
   +0.06 m on tiles and across tile/patch seams.

The old craters (a ring of 8 cm curbs plus a non-colliding disc buried inside the
solid sand) were physically drivable bumps and invisible to the depth camera; the
old full-zone coloured floor panels were removed because the depth camera rendered
them as an opaque floor over every crater.

---

## Rover

Stowed envelope 1.13 × 0.74 × 0.72 m, 50.5 kg (limits: 1.5 × 0.75 × 0.75 m, 80 kg;
`test_consistency.py`). Centre of mass 2.5 cm ahead of the wheel centre, 0.32 m high;
at rest the body pitches 1.3° nose-down on the rocker springs (measured,
`rover_physics_check.py`).

```
base_footprint (ground, between the wheels)
└─ base_link              chassis 0.90 × 0.50 × 0.20 m, underside 0.20 m above ground
   ├─ bin_link            (fixed) open-top sand bin, middle of the deck
   ├─ electronics_link    (fixed) enclosed electronics box, rear of the deck
   ├─ arm_mount_link      (fixed) front of the deck
   │  └─ boom_joint ─ boom_link ─ stick_joint ─ stick_link ─ bucket_joint ─ bucket_link
   ├─ left_rocker_joint ─ left_rocker_link
   │  ├─ front_left_wheel_joint ─ front_left_wheel_link
   │  └─ rear_left_wheel_joint  ─ rear_left_wheel_link
   ├─ right_rocker_joint ─ right_rocker_link
   │  ├─ front_right_wheel_joint ─ front_right_wheel_link
   │  └─ rear_right_wheel_joint  ─ rear_right_wheel_link
   ├─ lidar_link          (fixed) scan plane 0.10 m above ground
   ├─ imu_link            (fixed)
   └─ camera_link         (fixed) ─ camera_optical_link
```

Files: `urdf/rover_properties.xacro` (all numbers), `chassis.xacro`,
`suspension.xacro`, `wheels.xacro`, `arm.xacro`, `sensors.xacro`,
`gazebo_plugins.xacro`, assembled by `rover.urdf.xacro`.

### Suspension and drive

- **Rocker suspension:** one rocker per side pivots about the lateral axis on the
  chassis side (0.25 m above ground) and carries that side's front and rear wheel
  (wheelbase 0.70 m, track 0.66 m, wheel r = 0.12 m). Each rocker is an independent
  revolute joint (±0.30 rad) with a centring spring (300 N·m/rad) and damper; there
  is no differential bar, so the model stays a tree (no closed loop).
- **Skid-steer drive:** gz-sim's `DiffDrive` system velocity-commands all four
  wheel joints (two per side); `/cmd_vel` in, `/odom_raw` out. Verified live
  (`tools/system_check.py`): 0.2 m/s forward drives all four wheels at +1.67 rad/s;
  a 0.4 rad/s left turn drives the left pair at −1.1 and the right pair at +1.1 rad/s.
- **Wheel friction:** μ = 0.3 sideways, 1.0 rolling (equal friction cannot yaw in
  DART), plus **lateral slip compliance 0.003 m/s/N**. Without compliance the turn
  centre is indeterminate (equal wheel loads + Coulomb friction) and DART pivoted
  the rover about its front axle: base_footprint slid sideways at 0.18 m/s while
  spinning, which odometry cannot see. Measured after the fix: ≤ 0.007 m/s while
  spinning and 0.003 m/s in arcs. Faster in-place turns still slide: ~0.1 m/s at
  45–75 °/s (measured over 14 missions), so every in-place turn the stack commands
  is limited to 0.35 rad/s (mission search/survey/escape, Nav2 behaviors) and Nav2's
  Spin recovery is not used at all (see Autonomy stack).

### Excavator arm

Boom (0.36 m), stick (0.32 m) and bucket on three pitch joints, each held by a gz
`JointPositionController` (command `/arm/<joint>/cmd`, std_msgs/Float64, rad). The
boom pivots on a 5 cm pedestal above the deck (0.45 m above ground). Joint zero is
the **stowed** pose: boom 45° up, stick folded back and down in front of the boom,
bucket tucked against the stick, all above the camera's view and inside the
footprint's 0.66 m front. Poses (`config/arm_params.yaml`; bucket lip = point
(0.14, 0, −0.035) of `bucket_link`):

| Pose | boom, stick, bucket (rad) | Bucket lip |
|---|---|---|
| stow | 0, 0, 0 | over the deck front |
| carry | 0.30, −1.00, 2.10 | 1.02 m ahead, 0.43 m up, open side up |
| dig | 0.85, −1.25, 3.10 | 0.94 m ahead, 1.7 cm above the sand |
| scoop | 0.85, −1.25, 2.80 | 0.97 m ahead, 5 cm up (curl in) |
| pour | 0.30, −1.35, 0.90 | inverted, 0.91 m ahead, 0.58 m up |

`test/test_arm_sweep.py` runs forward kinematics of the generated URDF along both
mission sequences (stow → carry → dig → scoop → carry → stow and stow → carry →
pour → carry → stow, interpolated in joint space) and checks every step against the
chassis, bin, electronics box, wheels, rockers, sensors and the ground (1 cm
clearance; the bucket may reach 3 cm into the sand while digging), and that the
stowed arm is outside the camera's view. Gazebo does not self-collide links of one
model, so this test is what catches an arm passing through the body: the previous
arm geometry failed it (the stowed bucket sat ~8 cm inside the boom, and the boom
root passed ~3 cm through the chassis front when lowered, with its pivot only 2 cm
above the deck). Measured live with the current poses: joints settle within
0.08 rad, body tilt ≤ 3.2° during the sequences.
**No sand is simulated**: digging and dumping are kinematic motions plus a payload
counter (`payload_manager.py`), and the logs say "SIMULATED".

---

## Sensors, topics and TF

| Sensor | Topic(s) | Frame | Notes |
|---|---|---|---|
| 2D LiDAR (360°, 10 Hz, 0.12–8 m) | `/scan` | `lidar_link` | scan plane 0.10 m; rover hidden from it via visibility flags; wall and ground returns removed in `/scan_filtered` |
| IMU (100 Hz) | `/imu` | `imu_link` | gyro used; no absolute yaw (no compass) |
| RGB-D camera (640×480, 15 Hz, 66° HFOV, 15° down) | `/camera/image`, `/camera/depth_image`, `/camera/points`, `/camera/camera_info` | `camera_link` (cloud: x forward) | marker detection + craters |
| Wheel odometry | `/odom_raw` | `odom` → `base_footprint` | from DiffDrive |
| Joints | `/joint_states` | — | wheels, rockers, arm |

Derived topics: `/scan_filtered` (wall returns removed), `/crater_cells`
(confirmed crater cells, map frame), `/arena_boundary` (a-priori boundary grid),
`/localization/status` (incl. frame/TF-failure counters), `/localization/candidates`
(one JSON line per marker detection: ids, implied pose, range, outcome — for
validation tools), `/mission_state`, `/mission_status` (JSON), `/payload_level`,
`/arena/ground_truth_obstacles` (RViz only).

TF: `map → odom` (marker_localizer) → `base_footprint` (robot_localization EKF) →
`base_link` → sensor links (robot_state_publisher). Gazebo's own TF is not bridged.

---

## Autonomy stack

```
/camera/image ─► marker_localizer ─► map→odom (TF)        /scan + /imu ─► wall_filter
/odom_raw + /imu ─► EKF (robot_localization) ─► odom→base     (walls + ground returns
/camera/points ─► crater_detector ─► /crater_cells             removed) ─► /scan_filtered
arena_params ─► arena_boundary_publisher ─► /arena_boundary             │
                     │                          │                       │
                     ▼                          ▼                       ▼
            global + local costmaps: boundary (static) · obstacle (/scan_filtered)
                                     · crater (/crater_cells, never cleared) · inflation
                                                │
mission_state_machine ─ NavigateToPose ─► bt_navigator (no-spin BT, 2 Hz replan)
  LOCALIZE → SURVEY → legs                   ├─ planner_server: Smac Lattice (footprint-checked)
  goal checks vs /global_costmap/costmap     ├─ controller_server: Regulated Pure Pursuit
  escape / final turn ─► /cmd_vel            └─ collision_monitor ─► /cmd_vel ─► DiffDrive
```
TF tree: `map → odom → base_footprint → base_link → sensor links` (marker_localizer,
EKF, robot_state_publisher); Gazebo's TF is not bridged.

- **Localization (`marker_localizer.py`).** Replaces slam_toolbox. With wall returns
  removed a 2D LiDAR sees only three ~0.3 m boulders — far too little for scan
  matching — and the rulebook's answer is markers in the start area. Each ArUco
  frame with two or more markers is solved as ONE joint PnP over all their corners
  (map-frame object points; markers on different walls are not coplanar, which
  pins down yaw); a lone marker is solved with IPPE_SQUARE, keeping the solution
  consistent with the known camera height/tilt. Frames taken while turning faster
  than 0.25 rad/s are ignored. Initial fix = median of ≥ 3 multi-marker candidates
  (single-marker candidates only as a last resort, after 40 of them). After the
  initial fix, a detection is blended in (gain 0.3, with outlier gating; consistent
  outliers trigger re-initialisation, which also clears the costmaps and crater
  memory) only if it is accurate where it was taken — measured against Gazebo
  ground truth (`tools/marker_range_test.py`, the candidate log of `gt_monitor.py`):

  | Markers in view | Measured error | Used after the initial fix |
  |---|---|---|
  | 1 | 0.085 m / 5.3° at 0.5–1 m, 0.16 m at 2.5–3 m, 0.2–0.3 m beyond | never (`single_max_range` 0) |
  | 2 | 2–8 cm at 3.5 m, 0.15–0.35 m at 4–4.5 m | within 3.5 m (`pair_max_range`) |
  | ≥ 3 | 0.8 cm at 4–4.5 m | within 4.5 m (`multi_max_range`) |

  Single-marker updates (the old gain-0.08 blend) pulled the estimate 4° / 0.3–0.45 m
  off; without them the belief stayed within 2.7–4.2 cm / 0.4° over 5.7 m of driving. Between sightings the rover
  dead-reckons on the EKF (wheel velocities + gyro yaw). `map→odom` is stamped
  0.2 s ahead (as AMCL does) to avoid TF extrapolation errors.
- **Wall filter (`wall_filter.py`).** Every LiDAR hit within 0.30 m of the known
  arena boundary (in the localized map frame) becomes "no return" before the
  costmaps, the collision monitor and the perception node see it. Nothing is
  published until localization exists. It also removes **ground returns**: the scan
  plane is only 0.10 m up, so the 1.3° rest pitch (2–3° in the arm poses) puts the
  ground in the scan at 3–4.3 m; those hits formed arcs of phantom obstacles that
  closed every route (NO_VALID_PATH from everywhere). From the IMU's gravity-referenced
  roll/pitch the node computes each beam's ground-intersection range and drops
  returns beyond it minus 0.30 m. The gpu_lidar returns ground ~0.45° earlier than
  its beam axis (measured at rest: 3.21–3.29 m ahead where the axis predicts 4.35 m),
  which the model includes (`ground_beam_spread_deg`).
- **TF listeners.** `marker_localizer`, `wall_filter` and `crater_detector` look up
  transforms at each message's stamp, waiting up to 50 ms. With the listener on the
  node's own single-threaded executor the TF callbacks queued behind the sensor
  callback and the lookups failed (measured: 62 % of camera images, 55 % of scans,
  75–87 % of point clouds lost). Each now uses `TransformListener(buffer, None,
  spin_thread=True)`, i.e. its own node and thread (after: 1.7 %, 0 %, 0 %).
  Passing the node itself with `spin_thread=True` does not work: `rclpy.spin(node)`
  takes the node back into the main executor.
- **Boundary layer (`arena_boundary_publisher.py`).** A static grid, lethal within
  0.11 m of the walls (+ 0.06 m footprint padding = 0.17 m to a wall), loaded by both
  costmaps. It keeps the planner off the walls
  using the arena's known size instead of sensing the walls.
- **Crater detector (`crater_detector.py`).** Subsampled `/camera/points` →
  base frame → least-squares ground plane (absorbs chassis pitch) → points more
  than 1.5 cm below it within 0.3–2.2 m (2.5 cm missed most of 5–6 cm deep craters) → 5 cm map cells, confirmed after hits in
  2 frames → `/crater_cells`. The costmaps' `crater_layer` is marking-only, so
  LiDAR rays passing over a crater can never clear it. A planar LiDAR cannot see a
  depression at all. Before the first marker fix there is no map frame; hits are
  then counted in odom cells and moved into the map with the first map→odom (seed 10:
  a crater was in view only during the marker search and used to be discarded).
- **Nav2 (`config/nav2_params.yaml`).** Both costmaps use the real footprint
  `[[0.66, ±0.37], [−0.47, ±0.37]]` with **0.06 m padding**, inflation 0.85 m
  (beyond the 0.76 m turn-in-place radius; cost_scaling_factor 1.5). Smac rejects
  only poses whose footprint touches a lethal cell, so the padding is the hard
  clearance: with 0.02 m, planned paths passed 1.7–4 cm from boulders
  (`tools/planner_route_test.py`, seed 10). The planner weighs cost more
  (cost_penalty 4) so a wide route beats a narrow gap when both exist. Behavior tree:
  Nav2's default replanning/recovery tree **without the Spin recovery**
  (`config/behavior_trees/navigate_to_pose_no_spin.xml`) — recovery spins turned the
  rover, parked between a boulder and the north wall, to face the wall, after which
  every start was refused; the LiDAR is 360° anyway. Behavior-server rotations are
  limited to 0.35 rad/s. Global planner: **Smac Lattice** with the shipped
  differential-drive primitives (full-footprint collision checks, turning in
  place); NavFn plans for a point and would route through gaps the body cannot turn
  in. Controller: Regulated Pure Pursuit (0.20 m/s, rotate-to-heading); the behavior tree
  replans at 2 Hz and the global costmap updates at 4 Hz — a shallow crater is
  confirmed only ~1.5 m ahead, so reaction distance matters. Collision
  monitor on `/scan_filtered`. BT action-ack timeout 200 ms (Nav2's 20 ms default
  produced error-code-0 failures under load).

---

## Mission state machine

```
START → LOCALIZE → SURVEY → TRAVEL_TO_EXCAVATION → EXCAVATION → TRAVEL_TO_CONSTRUCTION → DUMP
      → (TRAVEL_TO_EXCAVATION … for mission.num_cycles) → RETURN → COMPLETE
any state → FAILED (terminal, reason in /mission_status)
```

| Transition | Earned by |
|---|---|
| START → LOCALIZE | odom→base TF exists (simulation + EKF running) |
| LOCALIZE → SURVEY | marker fix (`/localization/status` initialized); turns in place in steps (0.35 rad/s for 1.2 s, pause 1.0 s; frames taken while turning are ignored) while searching; 90 s timeout → FAILED |
| SURVEY → first leg | faced each of −60, −30, 0, 30, 60° (map frame, toward the obstacle zone) and held still 1.2 s, so the depth camera has swept the ground next to the start zone (≈ ±93° at 1.0–2.2 m) and craters there are in the costmap before the first plan; 60 s timeout → continue. Without it, a crater 1.2 m from the start was seen only if the marker search happened to spin past it (an immediate marker fix → a wheel in the crater) |
| TRAVEL_* → task | Nav2 **SUCCEEDED** *and* localized pose within 0.30 m / 0.60 rad of the goal *and* the leg's zone condition holds (the localized footprint ≥ 0.10 m across the excavation/construction line — `zone_cross_margin`, against localization error — every corner ≥ 0.10 m from the east wall; return: centre in the start zone) |
| EXCAVATION / DUMP → next | arm sequence reached each pose (within 0.10 rad, 10 s per pose) and the dwell time elapsed |

Goals (arena frame, `config/arena_params.yaml` → `mission:`): excavation (2.35, 0, 0°)
so the bucket digs at x ≈ 3.29 inside the excavation zone; construction
(3.15, 0, 0°) so the stowed front is 0.31 m into the construction zone (0.11 m even
if Nav2 stops 0.20 m short) and 0.69 m from the wall while the pour lands on the
berm target at x ≈ 4.06; return (−3.55, 0, 180°),
facing the start-zone markers. The old construction goal (the berm centre, x = 4.0)
put the footprint through the east wall at every heading but ±90°.

Failure handling: no hops — each leg's goal goes straight to Nav2 and the global
planner chooses the route. Each leg gets 3 attempts (240 s each). Error code 0
(rejected/cancelled/BT timeout) → retry the same goal; GOAL_OCCUPIED / NO_VALID_PATH
or "arrived but zone condition failed" → retry with an alternate goal that still
satisfies the zone condition and is free in the current costmap; then FAILED. Legs
wait for Nav2 to be ACTIVE instead of spending attempts. The arm is stowed and
verified before every leg. COMPLETE is published only after every leg was verified.

- **Blocked goals.** Before a goal is sent, and every second while it is active, it
  is checked with the planner's own test (a lethal cell under the padded footprint).
  A blocked candidate is skipped before sending; an active goal blocked twice in a
  row is cancelled and counts as a GOAL_OCCUPIED attempt, so the next try uses a
  free alternate. (Seed 10's primary excavation goal is 3 mm from the excavation
  boulder: blocked as soon as the boulder is mapped. Nav2 used to keep replanning
  and recovering instead of failing.) Results of cancelled or replaced goals are
  ignored (goal sequence numbers).
- **START_OCCUPIED.** Nav2's default tree runs no recovery for it. Before the retry
  the mission makes an escape motion (`mission_logic.escape_motion`): a turn in
  place of 5–60° or a straight drive of 0.05–0.25 m, at ≤ 0.35 rad/s / 0.08 m/s,
  closed-loop on the localized pose with a 15 s timeout. A candidate is accepted only
  if no intermediate pose covers a lethal cell the start does not already cover, the
  end pose is clear with padding + 3 cm, and its centre cell is below inscribed
  cost. If lethal cells are visible under the start, the smallest such motion is
  used; if none are (the planner refused a start this costmap copy shows as free),
  the motion ending at the lowest footprint cost. Escapes are bounded by the leg's 3
  attempts.
- **Final turn.** The lattice planner reaches a goal heading only by driving a
  2.6–3.3 m loop when the rover is already at the goal position but turned
  (`planner_route_test`, seed 14: all tested rotation penalties and expansion
  lengths) — through ground the camera never saw. Within 0.25 m of the goal and more
  than 0.40 rad off its heading the mission cancels the Nav2 goal and turns in place
  (≤ 0.35 rad/s, sweep checked against the costmap, 20 s timeout); arrival and the
  zone condition are then verified as for any Nav2 success.
- **Ground-truth check (validation only).** `tools/gt_monitor.py` records the
  Gazebo pose at every leg the mission reports as arrived and re-checks the zone rule
  on it; `suite_report.py` fails a seed unless all three legs were really in their
  zones, whatever the state machine reported.

---

## Testing

Unit tests (no Gazebo):

```bash
cd $WS/src/cac_sim && python3 -m pytest test/
```

`test_arena_layout.py` (determinism, obstacle counts/sizes/zones/spacing, start
rules, every emitted layout reachable, blocked/narrow layouts rejected, watertight
crater mesh, floor tiling, generated world, marker rules), `test_mission_logic.py`
(arrival/zone conditions incl. the crossing margin, bounded retries, alternate
goals, footprint cost check, START_OCCUPIED escape on the seed-10 geometry),
`test_consistency.py` (Nav2 footprint = arena params = URDF collision envelope,
competition envelope, every low rover visual hidden from the LiDAR after
URDF→SDF conversion, mission vs Nav2 tolerances and padding, wall/start margins,
spin-rate limit, LiDAR height), `test_arm_sweep.py` (arm sequences vs rover, ground
and camera view), `test_crater_detector.py` (pre-fix odom cells merged into the map),
`test_wall_filter.py` (ground-return geometry, calibrated against the measured
returns).

Gazebo tools (`tools/`, installed to `share/cac_sim/tools`; export the loopback
variables first):

| Tool | What it checks |
|---|---|
| `run_seed_suite.sh <out> <seeds…>` | fresh sim per seed, full mission, ground-truth monitor, summary table |
| `gt_monitor.py --seed N --out P` | localization error, footprint–wall/boulder distance, wheels in craters, crater detection, final state |
| `rover_physics_check.py --out P` | rest pose, forward/spin/arc/reverse tracking, spin drift, arm pose tracking and body tilt (`autonomy:=false`) |
| `crater_view_test.py --seed N --crater crater_k` | localize, park 2.0/1.5/1.0 m from a crater, report detected cells and false cells |
| `suite_report.py <out>` | PASS = COMPLETE, every leg confirmed in its zone by ground truth, no wall contact, boulder contact or wheel in a crater |
| `with_sim.sh <out> <timeout> "<launch args>" <cmd…>` | run one tool against a fresh headless sim and stop it afterwards |
| `system_check.py --out P` | topic rates, every link in TF, one parent per frame, `/tf` publishers, wheel directions (forward / left turn), rocker limits (`autonomy:=false`) |
| `marker_range_test.py --out P` | marker-fix error vs range and marker count against ground truth, believed-pose error while backing away (`randomize:=false autonomy:=false`) |
| `planner_route_test.py --seed N --out P` | localize + survey, then ComputePathToPose from given starts under given parameter sets; ground-truth clearance of each path (`autonomy:=false`) |
| `seed_diagnostic.py --seed N --out P` | per camera frame whether each crater was seen and usable; costmap/plan/footprint snapshots at every planner failure |

`run_seed_suite.sh` writes `seed_N_summary.json` (and `_candidates.csv`, the marker
fixes vs ground truth) as each seed finishes; `SUITE_RESUME=1` skips seeds that
already have a summary, so an interrupted suite reruns only the missing ones. It
force-stops Gazebo/Nav2/cac_sim processes before every seed: never run two suites
(or a suite and another sim) at once.

```bash
export GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
bash install/cac_sim/share/cac_sim/tools/run_seed_suite.sh $WS/suite_results/my_suite 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
bash install/cac_sim/share/cac_sim/tools/run_seed_suite.sh $WS/suite_results/seed10 10      # one seed
T=install/cac_sim/share/cac_sim/tools
bash $T/with_sim.sh $WS/suite_results/syscheck 300 "randomize:=false autonomy:=false" python3 $T/system_check.py --out $WS/suite_results/syscheck/check
bash $T/with_sim.sh $WS/suite_results/physics 500 "randomize:=false autonomy:=false" python3 $T/rover_physics_check.py --out $WS/suite_results/physics/physics
```

---

## Competition rules and interpretations

From `CAC_2027_Rulebook.pdf`:

- **Walls (Autonomy Rules 3–4, rule 10c):** walls must not be used for mapping,
  navigation or collision avoidance. Implemented: no wall returns reach mapping,
  costmaps, the collision monitor or perception; localization uses start-zone
  markers. *Open question:* the boundary keepout uses the arena's published size
  (not sensing) — whether judges accept such a virtual boundary is not settled by the
  rulebook. Without it, wall safety would rest on localization alone.
- **Markers:** ≤ 5, ≤ 25 × 25 cm, ≤ 60 cm high, one on the wall attached to the start
  zone, the others on "the bin frame structure along the perimeter of the starting
  zone (2 sides)" — interpreted as the north and south boundaries of the start zone.
- **Start:** random position and orientation, aligned N/S/E/W. Remote-control
  repositioning inside the start zone to "lock in" localization is allowed; the
  generator's turn-in-place clearance stands in for it.
- **Obstacles:** ≥ 3 boulders 30–40 cm with varying heights, ≥ 3 craters up to
  40–50 cm wide with varying depth, smaller boulders possible in the excavation zone.
  Crater depth is not specified (5–15 cm assumed).
- **Not traversable / contact penalties:** craters and boulders must be avoided
  (−30 per occurrence); the suite counts both from ground truth.
- **Sand:** must be acquired in the excavation zone and carried (not bulldozed) to
  the construction zone; here the dig and pour points are placed accordingly, but
  sand itself is not simulated.
- **Zone order and sizes** come from the reference diagram, which is not in the
  PDF's text.

---

## Known limitations

- **Dead reckoning between marker fixes.** Markers are only in the start zone, so in
  the obstacle and excavation zones the pose comes from wheel velocities + gyro. Any
  wheel slip (e.g. a wheel dropping into an undetected crater) becomes a persistent
  offset until the rover sees the markers again on the return leg. The wall filter
  and the boundary layer both rely on this pose.
- **Crater detection is forward-only and partial.** The camera sees the ground from
  ~0.35 m ahead of the bumper to ~2.2 m, within ±33°. The survey covers the ground
  next to the start zone; elsewhere a crater is seen only while driving toward it.
  From 1–1.5 m only the far wall of a bowl is visible, so a crater is mapped as
  13–17 cells of its ~78 (planned paths passed 2–3 cm from the true rim of a
  partly mapped crater in `planner_route_test`); the footprint padding and
  inflation are the only margin for the unseen near slope. **Crater incursions do
  occur**: in the 2026-10-12 21:02 suite a wheel entered a crater on seeds 12 (a
  5.9 cm deep crater mapped only at its core) and 14 (a planner loop through ground
  the camera never saw), and seed 12 repeated it once after the first fixes. A
  shallow crater is confirmed only ~1.5 m ahead, so detection is late; the depth
  threshold (1.5 cm), the final in-place turn, faster replanning and the lower
  cruise speed reduce but do not remove this risk — see Validation status.
- **Localization between the zones.** After the initial fix only multi-marker
  detections are used (two markers within 3.5 m, three within 4.5 m), so the rover
  dead-reckons through the obstacle and excavation zones and is corrected only when
  it faces the start zone again.
- **Nav2 start refusal not explained once.** In one seed-10 run the planner refused
  a start ("Start occupied") although no lethal cell lay under the padded footprint
  in any recorded costmap; inscribed cost under the footprint was tested and is not
  the cause. That run followed a recovery spin, which has since been removed; the
  mechanism inside Smac is unverified.
- **Ground-return model.** The LiDAR ground filter assumes flat ground and a
  calibrated 0.45° beam spread; on real terrain a slope ahead would be removed as
  ground (and an obstacle beyond the predicted ground range is not seen until
  closer).
- **Arm self-collision is checked geometrically, not simulated** (gz does not collide
  links of one model); the joint-space interpolation in `test_arm_sweep.py`
  approximates the controllers' real paths.
- **Rocker articulation is not exercised** by the flat sand bed: on flat ground the
  rockers sit at −0.023 rad; behaviour on rough terrain is untested.
- **No granular physics.** Sand is rigid; craters cannot be filled; the arm does not
  move material; berm volume is not simulated.
- **Boulders are static** (pushing them is impossible, as is detecting a push).
- **Rocker suspension** is spring-centred per side instead of a differential bar
  (a bar would close a kinematic loop, which URDF cannot express); it is a four-wheel
  rocker, not a rocker-bogie.
- **Gazebo GUI:** in one launch with the GUI the server never advertised its
  services; server-only (`headless:=true`) runs were reliable. See Validation status.
- `slam_toolbox_params.yaml` is kept for reference but SLAM is no longer launched.

---

## Validation status

Full report: [`docs/validation_2026-10-12.md`](docs/validation_2026-10-12.md).

> **The final full 15-seed regression has not been run on the current code.** The
> last complete run predates the crater depth threshold (1.5 cm), the final in-place
> turn, 2 Hz replanning, the 4 Hz global costmap and the 0.20 m/s cruise speed.

| Run (2026-10-12) | Code | Result |
|---|---|---|
| Unit tests (`pytest test/`) | current | **49 passed** |
| `colcon build` in a fresh workspace | current | builds |
| `system_check.py`, `rover_physics_check.py` | model unchanged since | pass (sensors, TF, drive directions, rockers, arm tracking, tilt ≤ 3.2°) |
| Full 15-seed suite (21:02) | before the last 5 changes | **13/15 pass**; 15/15 COMPLETE with all legs confirmed by ground truth; 0 wall/boulder contacts; crater incursions on seeds 12 and 14 |
| Seed 10, repeated | same | 4/4 pass (3 repeats + the suite) |
| Seeds 12, 14, 10 | + depth threshold + final turn | 14 and 10 pass; **12 fails again** (wheel in a shallow crater) |
| Seed 12 with the diagnostic tool | same | pass |
| Latest four changes (2 Hz replan, 4 Hz costmap, 0.20 m/s) | current | **not run in simulation** |

Previous baseline (2026-10-11, older code): 14/15, seed 10 failing; worst
localization error 0.319 m (now 0.179 m), closest boulder 0.075 m (now 0.133 m).
