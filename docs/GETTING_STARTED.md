# Getting started

## Option A — Docker (recommended)

Requires Docker and, for the GUI (Gazebo/RViz), an X server.

```bash
# allow the container to open windows on your display (Linux host)
xhost +local:docker

# build the image + start the container (idle, doesn't launch anything yet)
docker compose -f docker/docker-compose.yaml up -d --build

# open a shell inside it
docker exec -it greenhouse_amr_sim bash
```

This builds a Humble + Gazebo Fortress image and colcon-builds the
workspace, but the container itself just sits idle (`sleep infinity`) —
same dev-container style as the forklift project. From inside the shell,
launch pieces individually, one per `docker exec -it greenhouse_amr_sim
bash` terminal, exactly like the sections below:

```bash
ros2 launch greenhouse_bringup sim.launch.py       # terminal 1: Gazebo + RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=diff_drive_base_controller/cmd_vel_unstamped  # terminal 2: drive it
ros2 launch greenhouse_vision vision.launch.py      # terminal 3: dock-target detection + alignment
ros2 launch greenhouse_vslam rtabmap.launch.py      # terminal 4: VSLAM (map -> odom)
ros2 launch greenhouse_route_manager route_manager.launch.py  # terminal 5
```

(teleop is `ros2 run`, not `ros2 launch` -- `ros2 launch` pipes the child
process's stdio to capture logs, which breaks `teleop_twist_keyboard`'s
raw keyboard reads with a `termios.error`. `ros2 run` keeps the real tty.)

`src/` is bind-mounted into the container, so edits on the host show up
inside without rebuilding the image — just re-run `colcon build
--symlink-install` inside the container to pick them up.

## Option B — Native (Ubuntu 22.04 + ROS 2 Humble)

Install ROS 2 Humble, Gazebo Fortress, and the packages listed in
[`docker/Dockerfile`](../docker/Dockerfile) (that file doubles as the
authoritative dependency list), then:

```bash
mkdir -p ~/greenhouse_ws && cd ~/greenhouse_ws
ln -s /path/to/greenhouse_amr/src src   # or clone straight into ~/greenhouse_ws/src

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Sanity-check the robot model (no sim needed)

```bash
ros2 launch greenhouse_description view_robot.launch.py
```

You should see the robot in RViz; the `joint_state_publisher_gui` sliders
let you move the camera pan joint.

### Full simulation

```bash
ros2 launch greenhouse_bringup sim.launch.py
```

In a second terminal, drive it manually (`ros2 run`, not `ros2 launch` --
see the note above):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=diff_drive_base_controller/cmd_vel_unstamped
```

### Vision pipeline

With the sim running, in another terminal:

```bash
ros2 launch greenhouse_vision vision.launch.py
```

Point the robot at one of the grow-table dock_target plates (drive it
with teleop, or rotate `camera_pan_joint`) and watch
`/dock_target_detector/debug_image` in RViz/rqt (the contour + centroid
it found, and confidence/depth as text), or echo `/alignment/status`
while enabling tracking:

```bash
ros2 topic echo /dock_target_detector/confidence
ros2 topic pub --once /table_alignment_controller/enable std_msgs/msg/Bool "{data: true}"
```

### VSLAM (RTAB-Map)

There's no lidar and no static map file -- localization and mapping both
come from the RGB-D camera via RTAB-Map:

```bash
ros2 launch greenhouse_vslam rtabmap.launch.py
```

This runs `rgbd_sync -> rgbd_odometry -> rtabmap` (see the architecture
diagram in the top-level README). Drive the robot around with teleop and
check it's actually working:

```bash
ros2 topic echo /tf --once           # look for a map -> odom transform
ros2 topic hz /map                    # occupancy grid RTAB-Map is building
```

Nodes start clean, but a full end-to-end run (drive a loop, confirm loop
closure, check map quality) hasn't been done yet -- see the checklist in
the top-level README.

### Navigation

Nav2 bring-up (waypoint route follow) is still being wired together — see
the checklist in the top-level README. `greenhouse_navigation/config`
already has a tuned `nav2_params.yaml` (costmaps read obstacles from
`/camera/points`, no AMCL) and the aisle waypoint route, but `rtabmap`
above needs to be running and producing `/map` + `map -> odom` before
`route_manager` below can complete a full patrol.

### Full patrol (route manager)

Once Nav2 is bringing up cleanly against a map of the greenhouse world:

```bash
ros2 launch greenhouse_route_manager route_manager.launch.py
```

This drives the whole cycle from `README.md`'s architecture diagram:
Nav2 to each table -> hand off to `greenhouse_vision` for docking ->
log a stub reading on `/table_readings` -> move to the next table.
Watch it with:

```bash
ros2 topic echo /route_manager/state
ros2 topic echo /table_readings
```
