# r2_stack — R2 Visual Servo Stack

ABU Robocon 2026 "Kung Fu Quest". R2 collects Kung-Fu Scrolls (KFS, 350 mm
cardboard cubes) from the Meihua Forest blocks. The planner already knows
*which* block has *which* KFS — vision only handles the **final lateral
alignment** of the gripper to the cube before a pick. Classical OpenCV +
PCL + plain ROS 2 Humble; no ML, no Isaac ROS.

Camera: Intel RealSense D435i on a Jetson Orin Nano. Key constraints baked
into the defaults: 0.30 m minimum depth, `aligned_depth_to_color` only
(never raw depth), IMU settle check before measuring, 15-frame centroid
averaging, laser emitter at max power against venue IR.

## Packages

| Package | Language | What it does |
|---|---|---|
| `r2_interfaces` | rosidl | `AlignAndPick.action` (servo) and `LocateKfs.action` (localizer). Any future custom msg/srv/action belongs here — one interfaces package per workspace. |
| `r2_perception` | C++ / PCL | `kfs_localizer_node`: depth → point cloud → cluster → averaged KFS centroid. |
| `r2_servo` | Python | `pick_servo_node`: turns the centroid into a strafe correction. Action server `align_and_pick`. |
| `r2_calibration` | Python | Offline ChArUco extrinsic calibration (`base_link → camera_link`) + verification mode. |
| `r2_bringup` | launch/config | Launch files, parameter files, static TF from `config/camera_extrinsics.yaml`. |

`r2_perception` and `r2_servo` are servo-only. `r2_bringup`,
`r2_calibration` and `r2_interfaces` are shared foundations: bringup owns
the camera driver and the TF tree for the whole robot, calibration makes
any camera→robot coordinate conversion trustworthy (green detection
included, if it ever needs metric positions).

## Data flow

```
                 realsense2_camera (robot only, use_camera:=true)
                    │ /camera/aligned_depth_to_color/image_raw
                    │ /camera/color/camera_info
                    │ /camera/imu
                    ▼
   ┌─ r2_perception/kfs_localizer_node ──────────────────────────┐
   │ 1. wait IMU settle (<0.05 rad/s)                            │
   │ 2. project depth px → 3D, window Z∈[0.30,1.20] m, Y band    │
   │ 3. voxel grid + EuclideanClusterExtraction                  │
   │ 4. keep cluster with ~350 mm face extents                   │
   │ 5. compute3DCentroid, average 15 frames, reject if spread   │
   └──────────────┬───────────────────────────────────────────────┘
                  │ LocateKfs result: PointStamped in camera optical frame
                  │ (also published on /r2_perception/kfs_centroid)
                  ▼
   ┌─ r2_servo/pick_servo_node  (action: align_and_pick) ────────┐
   │ MEASURE → TRANSFORM (tf2 camera→gripper at msg timestamp,   │
   │ never Time(0)) → EVALUATE (gripper-frame Y = strafe error)  │
   │ → COMMAND_STRAFE → RE_MEASURE (≤2 attempts, ≤15 mm residual)│
   └──────┬───────────────────────────────┬───────────────────────┘
          │ strafe_backend: nav2          │ strafe_backend: cmd_vel
          ▼                               ▼
   Nav2 NavigateToPose             /cmd_vel Twist (timed, open loop,
   (relative lateral goal)          bridged to the Teensy) — FALLBACK
```

TF tree (static transforms from `r2_bringup`, values from
`camera_extrinsics.yaml`):

```
base_link ──► camera_link ──► camera_color_optical_frame   (driver publishes
    │                                                        the internal part)
    └───────► gripper_link
```

Nothing hardcodes camera-to-gripper offsets — everything goes through TF,
so recalibration is a config change, not a code change.

## How it integrates with mission_fsm

The FSM side was reworked in this same effort (files in `../mission_fsm/`):

1. **Planner** (`mission_fsm/path.py`): `generate_actions()` now emits
   3-tuples `(action, comment, meta)`. Pick-related primitives carry
   structured `meta = {'block': <id>, 'height': 'A'|'B'|'C'}` so the
   executor never parses block numbers out of comment strings.

2. **Executor** (`mission_fsm/forest_executor_node.py`): no longer
   fire-and-forget. It is a timer-driven sequencer (nothing blocks in
   callbacks, same rule as `NavInterface`) that executes ONE primitive at
   a time:

   * `VISUAL_SERVO_BLOCK` → sends an `AlignAndPick` goal to
     `pick_servo_node` with block id + height from `meta`, waits for the
     result. One retry on failure, then a clean `error` on
     `/fsm/area_status` (FSM dashboard decides retry/abort).
   * **Every other primitive** (`FORWARD_INIT`, `STRAFE_*`, `CLIMB_*`,
     `PICK_BLOCK_*`, `ROTATE_*`) → published as JSON on `/teensy/command`,
     then the executor waits for a completion ack.

3. **Teensy ack contract** (`/teensy/ack`, `std_msgs/String` JSON):

   ```json
   {"sequence": <sequence from the command>, "status": "done"}
   ```

   `"status": "error"` if the motion failed. The real bridge must publish
   this when the motion has **physically completed**, not when the command
   was forwarded. The placeholder `teensy_command_node` acks instantly so
   the whole chain dry-runs without hardware.

4. **Area flow** stays as before: `fsm_node` publishes the Area 2 start on
   `/fsm/area_command`; the executor plans, sequences, and publishes
   `area_complete` on `/fsm/signal` when the last primitive finishes.

Full chain for one KFS:

```
fsm_node ──/fsm/area_command──► forest_executor
   ▲                                │  step-by-step
   │ /fsm/signal: area_complete     ├─► /teensy/command ─► Teensy bridge ─► /teensy/ack
   └────────────────────────────────┤
                                    └─► align_and_pick ─► pick_servo ─► locate_kfs ─► kfs_localizer
```

The pick sequence itself is **not implemented yet** — `align_and_pick`
currently aligns only (`enable_pick_command: false`); the subsequent
`PICK_BLOCK_*` primitive goes to the Teensy like any other command.

## Build & run

```bash
cd <repo root>            # the repo root IS the colcon workspace root
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# dev machine (no camera): nodes start and idle gracefully
ros2 launch r2_bringup servo_stack.launch.py

# robot: also starts the D435i driver
ros2 launch r2_bringup servo_stack.launch.py use_camera:=true
```

Dry-run the whole Area 2 chain without hardware (placeholder bridge acks
instantly; the servo step fails cleanly after its timeout — expected):

```bash
ros2 run mission_fsm teensy_command &
ros2 run mission_fsm forest_executor &
ros2 topic pub -1 /fsm/area_command std_msgs/msg/String \
  "data: '{\"command\": \"start\", \"area\": \"AREA_2\",
           \"r1_blocks\": [4,6,9], \"r2_blocks\": [1,5,8,11], \"fake_block\": 7}'"
```

## Hardware bring-up checklist (deferred until robot + camera connected)

Order matters — do not skip or reorder:

1. RealSense driver up; confirm aligned depth + IMU in RViz2.
2. `lsusb -t` — must show 5000M (USB 3.0). Fix the cable before anything else.
3. Extrinsic calibration (warm the camera 5+ min first):
   ```bash
   ros2 run r2_calibration extrinsic_calibrator capture --session /tmp/calib.yaml
   ros2 run r2_calibration extrinsic_calibrator solve --session /tmp/calib.yaml \
       --board-pose X Y Z ROLL PITCH YAW --output r2_stack/r2_bringup/config/camera_extrinsics.yaml
   ros2 run r2_calibration extrinsic_calibrator verify --extrinsics ... --cube-pos X Y Z
   ```
   Verify at **0.35, 0.5 and 0.8 m**; all residuals must be < 10 mm.
   Re-verify any time the mount is touched.
4. Fill in `base_link_to_gripper_link` in `camera_extrinsics.yaml` from CAD.
5. Tune the localizer on a real cube: `y_band_a/b/c` in
   `r2_perception/config/kfs_localizer_params.yaml` (wide-open defaults
   until the mount is final), confirm stable centroids under bright light.
6. Single-shot open-loop servo test, then enable the re-measure loop,
   then wire into a full Area 2 run.

Known-failure mitigations already in the code: measurement happens before
the blind zone (<0.30 m), IMU settle gate, 15-frame averaging with spread
rejection, emitter at 360 for venue IR, no `sleep()` anywhere in launch —
nodes wait for topics instead.

## Key parameters

| File | Parameter | Default | Meaning |
|---|---|---|---|
| `kfs_localizer_params.yaml` | `min_depth_m` / `max_depth_m` | 0.30 / 1.20 | D435i trust window |
| | `centroid_avg_frames` | 15 | frames averaged per measurement |
| | `max_spread_mm` | 25 | reject measurement if per-frame centroids spread wider |
| | `y_band_a/b/c` | wide open | height band per block tier — **tune on robot** |
| `pick_servo_params.yaml` | `strafe_backend` | `nav2` | `nav2` (primary) or `cmd_vel` (fallback) |
| | `align_tolerance_mm` | 10 | done when \|offset\| below this |
| | `re_measure_threshold_mm` | 15 | max acceptable final residual |
| | `max_strafe_m` | 0.30 | sanity clamp — larger error aborts |
| `forest_executor` (params) | `ack_timeout_s` | 30 | per-primitive Teensy ack deadline |
| | `servo_timeout_s` | 90 | per-servo-attempt deadline |
