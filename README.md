# Greenhouse AMR

Autonomous mobile robot that patrols greenhouse aisles, docks at a
grow-table's visual target, and logs a plant-health reading — built as a
personal portfolio project (ROS 2 Humble + Gazebo Fortress).

![Greenhouse AMR demo](docs/media/demo.gif)
*(GIF pending — see [docs/media/README.md](docs/media/README.md))*

## Why this project

This started as a way to demonstrate, outside of proprietary work, the parts
of a mobile-robotics stack I actually build day to day: simulation,
navigation, VSLAM, and vision-based docking/alignment, tied together by a
small state machine. Different robot, different world, same underlying
skills — including the sensing approach: no lidar, camera-only mapping
and obstacle avoidance via RTAB-Map, the same division of labor as the
forklift project's RealSense/ZED + RTAB-Map stack.

## What it does

1. **Navigate** — the robot follows a waypoint route through greenhouse
   aisles using Nav2, avoiding obstacles (people, carts, dropped pots).
2. **Detect** — the mast RGB-D camera looks for the dock_target plate
   mounted on each grow table (HSV color + contour detection, no coded
   marker) as the robot approaches, and samples depth *at that pixel* to
   get a metric position — the same per-point-depth reasoning as the
   forklift project's fork/pallet alignment.
3. **Dock / align** — once the target is tracked, a visual-servo
   controller aligns the robot laterally to a standoff offset from the
   table, deciding "aligned" via a Schmitt-hysteresis + debounce
   evaluator (not a single fixed threshold) — the same
   "detect → approach → align → commit" pattern used in pallet docking.
4. **Log** — on a successful alignment, the robot records a simulated
   reading (e.g. soil-moisture/temperature stub) tagged with the table ID
   and continues its route.

## Architecture

```
                     ┌─────────────────────┐
                     │   route_manager      │  (state machine)
                     │  IDLE → TRAVEL →      │
                     │  SEARCH → ALIGN →     │
                     │  LOG → RESUME         │
                     └─────┬───────┬─────────┘
                           │       │
              Nav2 goals   │       │  align cmd_vel override
                           ▼       ▼
                 ┌──────────────┐ ┌──────────────────┐
                 │    Nav2      │ │  dock_target      │
                 │ (BT nav)     │ │  (vision servo)   │
                 └──────┬───────┘ └─────────┬─────────┘
                        │                   │
                        │        ┌──────────┴─────────┐
                        │        │  greenhouse_vslam   │
                        │        │  rgbd_sync ->       │
              map->odom │        │  rgbd_odometry ->   │
              /map      │        │  rtabmap            │
                        │        └──────────┬──────────┘
                        ▼                   │  color+depth+info
                 ┌──────────────────────────┴───┐
                 │       gz_ros2_control         │
                 │  diff-drive base + pan RGB-D  │
                 └───────────────┬───────────────┘
                                 ▼
                     Gazebo Fortress (greenhouse world)
```

No lidar: RTAB-Map (`greenhouse_vslam`) publishes `map -> odom` itself
from the camera feed, and Nav2's costmaps read obstacles from the same
camera's point cloud -- there's no AMCL and no separate 2D scan sensor.

## Packages

| Package | Purpose |
|---|---|
| `greenhouse_description` | URDF/xacro robot model (diff-drive base, pan-tilt RGB-D camera — no lidar), RViz config |
| `greenhouse_world` | Gazebo world (greenhouse aisles, grow tables with a dock_target plate each), model assets |
| `greenhouse_navigation` | Nav2 params (camera-fed costmaps, no AMCL), waypoint route definitions |
| `greenhouse_vslam` | RTAB-Map pipeline (`rgbd_sync` → `rgbd_odometry` → `rtabmap`) — localization and mapping off the same camera |
| `greenhouse_vision` | Color/depth dock-target detection + hysteresis-based lateral alignment controller |
| `greenhouse_route_manager` | Patrol state machine (`route_manager` node) — drives Nav2 to each table, hands off to vision for docking, logs a reading, resumes |
| `greenhouse_bringup` | Top-level launch files that tie everything together, Docker setup |

## Status

Work in progress, built incrementally in public. Checklist:

- [x] Repo scaffold, packages, licensing
- [x] Robot URDF (diff-drive + pan RGB-D camera) renders in RViz
- [x] Greenhouse Gazebo world with a dock_target plate at each grow table
- [x] Robot spawns and drives in simulation (teleop), all controllers
      activate cleanly (verified in a running container)
- [ ] RTAB-Map VSLAM (`greenhouse_vslam`) run end-to-end against real
      robot motion — nodes start clean, loop-closure/map quality untested
- [ ] Nav2 stack tuned, waypoint route follow
- [ ] Dock-target detection (color + per-point depth) confirmed against
      a live camera feed — nodes start clean, detection itself untested
      end-to-end yet
- [ ] Lateral alignment controller (visual servo + hysteresis evaluator)
      — written, not yet driven against a live detection
- [x] Route manager state machine tying it together (written, not yet run
      end-to-end against live Nav2 — see `greenhouse_route_manager`)
- [x] Docker Compose one-command dev container (idle by default, exec in
      and launch pieces by hand)
- [ ] Demo GIF / video in this README

## Quickstart

```bash
docker compose -f docker/docker-compose.yaml up -d --build
docker exec -it greenhouse_amr_sim bash
```

This builds the workspace; the container itself stays idle so you launch
pieces by hand (one `ros2 launch ...` per terminal), same dev-container
workflow as the forklift project. See
[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) for the exact launch
sequence and the native (non-Docker) build.

## Stack

- ROS 2 Humble
- Gazebo Fortress (`ros_gz`, `gz_ros2_control`)
- Nav2 (no AMCL — RTAB-Map handles localization)
- RTAB-Map (`rtabmap_ros`) for VSLAM off the RGB-D camera
- OpenCV (HSV color threshold + contour detection, per-point depth
  unprojection) for dock-target detection — no fiducial markers
- Docker / Docker Compose

## License

MIT — see [LICENSE](LICENSE).
