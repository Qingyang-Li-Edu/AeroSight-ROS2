FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive \
    LIBGL_ALWAYS_SOFTWARE=1 \
    DISPLAY=:1 \
    G1_ROOT=/ws/AeroSight-ROS2

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-tf2-ros \
        python3-pykdl python3-numpy \
        xvfb x11vnc novnc websockify fluxbox \
        mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/AeroSight-ROS2
EXPOSE 6080
ENTRYPOINT ["/ws/AeroSight-ROS2/docker/entrypoint.sh"]
