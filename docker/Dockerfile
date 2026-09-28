FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Fortress + ros_gz + gz_ros2_control + Nav2 + RTAB-Map (VSLAM,
# camera-only -- no lidar) + teleop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    ros-humble-cv-bridge \
    ros-humble-tf-transformations \
    python3-opencv \
    python3-yaml \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    xterm \
    htop \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src ./src

RUN . /opt/ros/humble/setup.sh && \
    rosdep update --rosdistro=humble && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro=humble || true

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# So a plain `docker exec -it <container> bash` (no entrypoint involved)
# also has the workspace sourced, same as a forklift-style dev container.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# Idle by default -- exec into the container and launch things by hand
# (sim.launch.py, teleop.launch.py, vision.launch.py, ...) each in its own
# terminal, same workflow as the forklift dev container.
CMD ["sleep", "infinity"]
